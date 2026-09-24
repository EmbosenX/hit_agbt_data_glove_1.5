/*******************************************************************************
 * @file        app_sample_task.c
 * @brief       传感器采集任务实现，FreeRTOS任务/队列/互斥锁全部静态内存分配
 * @author      
 * @date        2026-07-21
 * @version     V1.0
 ******************************************************************************/
#include "app_sample_task.h"
#include <string.h>
#include "adc_sample.h"
#include "app_fingers_task.h"
#include "app_force_calib.h"    /* 灏存标定：按部位 LUT 换算（AppForceCalib_ApplyPart） */

/* ====================== 可配置参数 ====================== */
/* 静止钳零死区（减零点门限）：减零点后增量 <= 本值 判为残余应力/零漂，直接输出 0；
 * 取 0 = 关闭死区（当前取值），判据退化为 value > calib ? value-calib : 0。
 * 作用于全部 15 个区域，标定采集（AD 域）与力值输出共用同一减零点口径。
 * 取值权衡：过大会压掉真实的轻触小信号（并抬高标定采集的低量程下限）；关闭后
 * 静止零漂会如实进入快照与协议应答，需由上位机显示层自行滤波。
 * 可用编译器 -DSAMPLE_ZERO_DEADZONE=... 覆盖本默认值 */
#ifndef SAMPLE_ZERO_DEADZONE
#define SAMPLE_ZERO_DEADZONE   (0U)
#endif

/* ====================== 静态内存缓冲区（全局静态RAM，不占用堆） ====================== */
/* 1. 采集任务静态栈 & TCB控制块 */
static StackType_t m_auSampleTaskStack[SAMPLE_TASK_STACK_SIZE];
static StaticTask_t m_stSampleTaskTcb;

/* ====================== 对外全局变量定义 ====================== */
TaskHandle_t        m_hSampleTask = NULL;
/* 共享采样缓存互斥锁（定义见 APP_Sample_TaskCreate，声明见 app_sample_task.h）：
   - 写侧：本任务采集处理段（Sample_DoCollect/SnapshotZero/ApplyCalib）、fingers 任务
     Fingers_ParseSlaveFrame 写 cfg 手指区，二者各自以互斥临界段写入；
   - 读侧：本任务轮末发布的免锁快照（TacConfig_PublishSnapshot/GetSnapshot，见
     app_tac_config.c），供 usart 任务 Official 应答读取，不再与该锁竞争。
   锁粒度 = 整轮写临界段，保证每轮发布出去的都是完整一致成品帧（无撕裂） */
SemaphoreHandle_t   m_hSampleDataMutex = NULL;

/* fingers→sample「本轮采集已完成」专用握手信号量（声明见 app_sample_task.h）：
   把"usart 发令采集"与"fingers 回报完成"拆到两条互不干扰的等待通道上，
   避免 usart 的发令通知提前满足本任务对 fingers 完成回报的等待 */
SemaphoreHandle_t   m_xFingerDoneSem = NULL;

/* ====================== 私有静态局部变量 ====================== */
/* 共享采样缓存互斥锁静态控制块（不占用堆） */
static StaticSemaphore_t m_stSampleDataMutex;
/* fingers→sample 完成握手信号量静态控制块（不占用堆） */
static StaticSemaphore_t m_stFingerDoneSem;
/* 本轮"发布源"（成品帧）指针：
 *  - NULL      = 发布采样工作区（未启用力值换算，或零点基准尚未就绪）；
 *  - 非 NULL   = 指向换算成品帧 s_stOutCfg（力值成品帧，单位见标定核心输出单位宏）。
 * 由 Sample_ApplyCalib() 每轮设置；轮末发布时其内容已整帧拷入快照缓冲，
 * 故无需跨轮保持，也无需加锁（仅采样任务读写，发布在同一临界段内完成） */
static const tac_data_cfg_t *s_pPublishSrc = NULL;
#if TAC_CALIB_EN
/* 重新零点请求：1=待执行零点校准（下一采集轮 cfg 处于整手 raw 窗口内快照零基准） */
static volatile uint8_t s_u8RecalibReq = 0U;
/* 零点基准就绪标志：首次整手快照成功后置 1；就绪前不做减零点与力值换算 */
static volatile uint8_t s_u8CalibReady = 0U;
#if (APP_FORCE_CALIB_APPLY_EN != 0U)
/* 力值换算成品帧（与采样工作区**分离**：换算只写本缓冲，工作区恒保存
   "减零点后的 AD 增量"）。tac_data_cfg_t 为 pack(1) 纯字节矩阵结构，静态分配
   避免占用任务栈；每轮换算前从工作区整结构 memcpy 一份再就地改写。
   关闭换算（APPLY_EN=0）时本缓冲无使用者，故整块裁掉：既避免
   "#177-D declared but never referenced" 告警，也省下 1241B ZI */
static tac_data_cfg_t s_stOutCfg;
#endif
/* 输出量纲提示（WEIGHT / AD_DELTA）只打印一次，避免每轮刷屏 */
static uint8_t s_u8OutModeLogged = 0U;
#endif


/* ====================== 内部函数前置声明 ====================== */
static void Sample_InitCaliber(void);
static void Sample_TaskEntry(void *pvParam);
static void Sample_DoCollect(void);
static void Sample_CalibrateZero(uint8_t row, uint8_t col, void *frame, void *calib);
#if TAC_CALIB_EN
static uint8_t *Sample_AreaPtr(tac_data_cfg_t *pTacData, tac_area_t u8Area);
static void Sample_SnapshotZero(void);
static void Sample_ApplyCalib(void);
#endif
/**
 * @brief 静态创建采集任务、消息队列、数据互斥锁
 */
void APP_Sample_TaskCreate(void)
{
    /* 静态创建共享采样数据互斥锁（须先于采样/应答对 cfg 的并发访问创建；
       fingers/sample 写侧与 usart 任务 Official 读侧共用同一把锁） */
    m_hSampleDataMutex = xSemaphoreCreateMutexStatic(&m_stSampleDataMutex);
    if (m_hSampleDataMutex == NULL)
    {
        DDL_Printf("Sample Data Mutex Static Create Failed!\r\n");
    }

    /* 静态创建 fingers→sample 完成握手二值信号量（初值 0：首轮开始即需等 fingers 回报） */
    m_xFingerDoneSem = xSemaphoreCreateBinaryStatic(&m_stFingerDoneSem);
    if (m_xFingerDoneSem == NULL)
    {
        DDL_Printf("Sample Finger-Done Sem Static Create Failed!\r\n");
    }

    /* 静态创建采集主任务 */
    m_hSampleTask = xTaskCreateStatic(
        Sample_TaskEntry,
        "SampleTask",
        SAMPLE_TASK_STACK_SIZE,
        NULL,
        SAMPLE_TASK_PRIORITY,
        m_auSampleTaskStack,
        &m_stSampleTaskTcb
    );
    /* 静态资源创建容错判断 */
    if((NULL == m_hSampleTask))
    {
        DDL_Printf("Sample Task Static Resource Create Failed!\r\n");
    }
}

#if TAC_CALIB_EN
/* ====================== 重新零点控制接口 ====================== */
/**
 * @brief  请求重新零点校准（由采集任务在下一采集周期执行）
 */
void Sample_RequestRecalib(void)
{
    s_u8RecalibReq = 1U;
}

/**
 * @brief  查询是否有待处理的零点校准请求
 */
uint8_t Sample_IsRecalibRequested(void)
{
    return s_u8RecalibReq;
}

/**
 * @brief  清除零点校准请求标志
 */
void Sample_ClearRecalibRequest(void)
{
    s_u8RecalibReq = 0U;
}
#endif /* TAC_CALIB_EN */

/**
 * @brief 采集任务主循环入口
 * @note  由 usart 任务空闲节拍发令驱动（10ms 周期，见 app_usart_task.c
 *        AppTaskUsart）：收到发令后通知手指任务采集→等其完成回报→拇指/掌心 ADC
 *        采样→整手零点快照→逐区域标定计算，完成后再等下一次发令。采集轮耗时超过
 *        发令周期时任务通知会积压，退化为按采集能力连续运行（不做节流）。
 *        cfg 整轮写入段持 m_hSampleDataMutex，与 fingers 任务写手指区互斥；轮末在
 *        释放该锁前发布免锁快照（TacConfig_PublishSnapshot），供 usart 应答读取。
 */
static void Sample_TaskEntry(void *pvParam)
{
#if SAMPLE_LOOP_TIME_DEBUG
    /* 整轮采集耗时调试计时（开启 SAMPLE_LOOP_TIME_DEBUG 时有效） */
    TickType_t tLoopStartTick = 0U;
		uint32_t tick_cnt=0;
#endif
    /* 底层ADC硬件初始化 */
		ADC_SAMPLE_Init();
		vTaskDelay(300);
    /* 注：启动零点不在此采集——此刻手指任务(SPI 从机)尚未回传、cfg 手指区全 0。
       首轮待手指任务完成后 cfg 手指区写入本轮回传 raw、Sample_DoCollect() 写入本地
       raw 时，方在下方采集轮内对整手执行零点快照（Sample_SnapshotZero） */
		/************************ 通知外部任务，进行采集 ************************/
		if (xAppFingersTaskHandle != NULL) {
				xTaskNotifyGive(xAppFingersTaskHandle);
		}
		/************************ 等待手指任务回报本轮采集完成 ************************/
		/* 【握手通道分离】等待 fingers「本轮采集已完成」专用信号量，而非任务通知：
		   m_hSampleTask 通知通道已被 usart 任务独占用于采集发令，共用同一通知计数会让
		   发令提前满足本等待，导致 fingers 未写完 cfg 手指区就开始整轮处理（读到旧值） */
		if (m_xFingerDoneSem != NULL)
		{
			(void)xSemaphoreTake(m_xFingerDoneSem, portMAX_DELAY);
		}
		else
		{
			/* 兜底：信号量未就绪时不永久阻塞，按 5ms 自节拍继续（仅极端降级路径） */
			vTaskDelay(pdMS_TO_TICKS(5));
		}
		Sample_InitCaliber();
    while(1)
    {
				/************************ 等待 usart 任务采集发令（保持任务通知通道） ************************/
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
				/* 整轮耗时打点起点：须早于通知手指任务，否则漏掉手指任务采集段 */
#if SAMPLE_LOOP_TIME_DEBUG
        tLoopStartTick = xTaskGetTickCount();
#endif
				/************************ 通知外部任务，进行采集 ************************/
        if (xAppFingersTaskHandle != NULL) {
            xTaskNotifyGive(xAppFingersTaskHandle);
        }
				/************************ 等待手指任务回报本轮采集完成 ************************/
        /* 【握手通道分离】同上：只等 fingers 完成信号量，绝不复用 m_hSampleTask 通知通道 */
        if (m_xFingerDoneSem != NULL)
        {
            (void)xSemaphoreTake(m_xFingerDoneSem, portMAX_DELAY);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        /* cfg 整轮写临界段开始：本地采样 + 整手零点快照 + 逐区域标定合并为一个原子
           写窗口；轮末在该窗口内发布免锁快照，保证 usart 任务应答读到的是完整一致
           成品帧而非部分更新帧；fingers 任务写手指区与本任务经握手链串行，不会与
           本临界段互持同一把互斥锁而死锁 */
        if (m_hSampleDataMutex != NULL)
        {
            (void)xSemaphoreTake(m_hSampleDataMutex, portMAX_DELAY);
        }
        /* 采集使能时执行ADC采样 */
        Sample_DoCollect();
#if TAC_CALIB_EN
        /* ============ 阶段一：零点快照（启动首轮 / 收到重新零点命令时） ============ */
        /* cfg 此刻为整手 raw（手指区=从机本轮回传、拇指/掌心=本地直采）。要求本轮两从机
           均有效：若任一从机 SPI 失败或帧被丢弃，cfg 手指区仍为上一轮已处理旧值，不可当
           零基准，保留请求待下轮（采集窗口）重试 */
        if ((!s_u8CalibReady) || Sample_IsRecalibRequested())
        {
            if (Fingers_IsLastRoundValid())
            {
                Sample_SnapshotZero();
                s_u8CalibReady = 1U;
                Sample_ClearRecalibRequest();
                /* 零点基准已重做：清空"区域 AD 总和"平滑状态，避免把旧基准确立时期
                   累加的历史值继续平滑（否则重新零点后会有一段爬升/残留） */
                AppForceCalib_ResetSmoothState();
                DDL_Printf("CALIB_ZERO_READY\r\n");
            }
        }
        /* ============ 阶段二：每轮标定计算（零点基准就绪后） ============ */
        /* 基准未就绪：cfg 保持原始 raw（供零点快照与上位机观测）；
           就绪后本阶段执行「减零点」，再按部位标定表换算：部位已标定（0x07 PUT
           下发过 LUT）则输出力值，未标定则保持「减零点后的 AD 增量」，
           供上位机采集标定数据（见 Sample_ApplyCalib） */
        if (s_u8CalibReady)
        {
            Sample_ApplyCalib();
        }
#endif
        /* 轮末发布：整轮已写完，把本轮**成品帧**（发布源）发布为免锁快照，供官方协议
           应答（app_official_proto.c get_ori_data/get_force）直接取用，读侧不再争抢
           m_hSampleDataMutex；发布必须在释放互斥锁之前，保证发布出去的是完整帧。
           发布源为 NULL（零点未就绪等早退路径）时退化为发布采样工作区 */
        TacConfig_PublishSnapshotFrom((NULL != s_pPublishSrc) ? s_pPublishSrc
                                                             : (const tac_data_cfg_t *)GetTacDataCfg());
        /* cfg 整轮写临界段结束 */
        if (m_hSampleDataMutex != NULL)
        {
            (void)xSemaphoreGive(m_hSampleDataMutex);
        }
        /* 整轮耗时打点终点（临界段已释放）：[通知手指采集 -> 此处] 为一整轮采集耗时；
           TickType_t 无符号相减天然处理节拍回绕，portTICK_PERIOD_MS=1 时 tick 差值即毫秒 */
#if SAMPLE_LOOP_TIME_DEBUG
        /* 本开关默认跟随 LL_PRINT_ENABLE（DDL_OFF 时 DDL_Printf 为空函数、无任何输出），
           耗时由 Tick 差值算出后直接内联进打印实参 */
				tick_cnt = (unsigned int)((xTaskGetTickCount() - tLoopStartTick) * portTICK_PERIOD_MS);
        DDL_Printf("SAMPLE_LOOP_MS=%u\r\n",tick_cnt);
#endif
    }
}


/**
 * @brief 执行单次多路ADC采集
 */
static void Sample_DoCollect(void)
{
		tac_data_cfg_t* pTacData = GetTacDataCfg();
	
		SampleGetFrame(TAC_AREA_THUMB_DIP,
									TacAreaRange[TAC_AREA_THUMB_DIP].row_end-TacAreaRange[TAC_AREA_THUMB_DIP].row_start+1,
									TacAreaRange[TAC_AREA_THUMB_DIP].col_end-TacAreaRange[TAC_AREA_THUMB_DIP].col_start+1,
									pTacData->thumb_dip);
	
		SampleGetFrame(TAC_AREA_THUMB_MCP,
									TacAreaRange[TAC_AREA_THUMB_MCP].row_end-TacAreaRange[TAC_AREA_THUMB_MCP].row_start+1,
									TacAreaRange[TAC_AREA_THUMB_MCP].col_end-TacAreaRange[TAC_AREA_THUMB_MCP].col_start+1,
									pTacData->thumb_mcp);


		SampleGetFrame(TAC_AREA_PALM,
									TacAreaRange[TAC_AREA_PALM].row_end-TacAreaRange[TAC_AREA_PALM].row_start+1,
									TacAreaRange[TAC_AREA_PALM].col_end-TacAreaRange[TAC_AREA_PALM].col_start+1,
									pTacData->palm);
}

/**
 * @brief  采集标定原始数据：把本地 ADC 原始值读入标定工作区（GetTacCalibData）
 * @note   仅本地 ADC 通道（拇指 DIP/MCP、掌心），手指通道由 SPI 从机提供；
 *         供上位机采集标定数据使用，不参与运行期力值换算
 */
static void Sample_InitCaliber(void)
{
		tac_data_cfg_t* pTacData = GetTacDataCfg();
		tac_data_cfg_t* pTacCalib = GetTacCalibData();
		SampleGetFrame(TAC_AREA_THUMB_DIP,
									TacAreaRange[TAC_AREA_THUMB_DIP].row_end-TacAreaRange[TAC_AREA_THUMB_DIP].row_start+1,
									TacAreaRange[TAC_AREA_THUMB_DIP].col_end-TacAreaRange[TAC_AREA_THUMB_DIP].col_start+1,
									pTacCalib->thumb_dip);
		SampleGetFrame(TAC_AREA_THUMB_MCP,
									TacAreaRange[TAC_AREA_THUMB_MCP].row_end-TacAreaRange[TAC_AREA_THUMB_MCP].row_start+1,
									TacAreaRange[TAC_AREA_THUMB_MCP].col_end-TacAreaRange[TAC_AREA_THUMB_MCP].col_start+1,
									pTacCalib->thumb_mcp);
		SampleGetFrame(TAC_AREA_PALM,
									TacAreaRange[TAC_AREA_PALM].row_end-TacAreaRange[TAC_AREA_PALM].row_start+1,
									TacAreaRange[TAC_AREA_PALM].col_end-TacAreaRange[TAC_AREA_PALM].col_start+1,
									pTacCalib->palm);
		memcpy(pTacCalib->index_dip,pTacData->index_dip,sizeof(pTacData->index_dip));
		memcpy(pTacCalib->index_pip,pTacData->index_pip,sizeof(pTacData->index_pip));
		memcpy(pTacCalib->index_mcp,pTacData->index_mcp,sizeof(pTacData->index_mcp));
		memcpy(pTacCalib->middle_dip,pTacData->middle_dip,sizeof(pTacData->middle_dip));
		memcpy(pTacCalib->middle_pip,pTacData->middle_pip,sizeof(pTacData->middle_pip));
		memcpy(pTacCalib->middle_mcp,pTacData->middle_mcp,sizeof(pTacData->middle_mcp));
		memcpy(pTacCalib->ring_dip,pTacData->ring_dip,sizeof(pTacData->ring_dip));
		memcpy(pTacCalib->ring_pip,pTacData->ring_pip,sizeof(pTacData->ring_pip));
		memcpy(pTacCalib->ring_mcp,pTacData->ring_mcp,sizeof(pTacData->ring_mcp));
		memcpy(pTacCalib->little_dip,pTacData->little_dip,sizeof(pTacData->little_dip));
		memcpy(pTacCalib->little_pip,pTacData->little_pip,sizeof(pTacData->little_pip));
		memcpy(pTacCalib->little_mcp,pTacData->little_mcp,sizeof(pTacData->little_mcp));
}
#if TAC_CALIB_EN
/**
 * @brief 返回 cfg/calib 结构中指定节级区域矩阵的首字节地址
 * @param  pTacData 目标数据源（GetTacDataCfg()/GetTacCalibData() 返回值）
 * @param  u8Area   节级区域枚举（TAC_AREA_THUMB_DIP ~ TAC_AREA_PALM）
 * @return 区域矩阵首字节地址；非法区域或空指针返回 NULL
 */
static uint8_t *Sample_AreaPtr(tac_data_cfg_t *pTacData, tac_area_t u8Area)
{
    if (pTacData == NULL)
    {
        return NULL;
    }

    switch (u8Area)
    {
        case TAC_AREA_THUMB_DIP:     return (uint8_t *)pTacData->thumb_dip;
        case TAC_AREA_THUMB_MCP:     return (uint8_t *)pTacData->thumb_mcp;
        case TAC_AREA_INDEX_DIP:     return (uint8_t *)pTacData->index_dip;
        case TAC_AREA_INDEX_PIP:     return (uint8_t *)pTacData->index_pip;
        case TAC_AREA_INDEX_MCP:     return (uint8_t *)pTacData->index_mcp;
        case TAC_AREA_MIDDLE_DIP:    return (uint8_t *)pTacData->middle_dip;
        case TAC_AREA_MIDDLE_PIP:    return (uint8_t *)pTacData->middle_pip;
        case TAC_AREA_MIDDLE_MCP:    return (uint8_t *)pTacData->middle_mcp;
        case TAC_AREA_RING_DIP:      return (uint8_t *)pTacData->ring_dip;
        case TAC_AREA_RING_PIP:      return (uint8_t *)pTacData->ring_pip;
        case TAC_AREA_RING_MCP:      return (uint8_t *)pTacData->ring_mcp;
        case TAC_AREA_LITTLE_DIP:    return (uint8_t *)pTacData->little_dip;
        case TAC_AREA_LITTLE_PIP:    return (uint8_t *)pTacData->little_pip;
        case TAC_AREA_LITTLE_MCP:    return (uint8_t *)pTacData->little_mcp;
        case TAC_AREA_PALM:          return (uint8_t *)pTacData->palm;
        default:                     return NULL;
    }
}

/**
 * @brief 整手零点快照：把 cfg 当前整手 raw 复制为 tac_calib_data 零基准
 * @note  两结构同型且 pragma pack(1)，整结构 memcpy 安全；仅当本轮两从机均
 *        有效（cfg 手指区=本轮回传 raw）时由采集轮调用，保证零基准为同一
 *        "两从机空载帧 + 本地直采"窗口内的统一快照
 */
static void Sample_SnapshotZero(void)
{
    memcpy((void *)GetTacCalibData(), (const void *)GetTacDataCfg(), sizeof(tac_data_cfg_t));
}

/**
 * @brief 每轮标定计算：整手「减零点」+ 按部位标定表（LUT）换算力值（双缓冲）
 * @note  区域 0..TAC_AREA_MAX 连续且与 tac_area_t 枚举序一致；区域矩阵在
 *        cfg/calib 结构中行优先连续存储，矩阵行列数取自 TacAreaRange，
 *        与 Sample_AreaPtr 返回的首地址组合即可遍历整个区域；
 *        **量纲约定（重要）**：
 *          - 采样工作区 `GetTacDataCfg()` 永远只保存「减零点后的 AD 增量」，
 *            本函数不做就地换算；
 *          - 力值换算只在独立成品帧 s_stOutCfg 上做（先整结构拷贝再改写），
 *            并把发布源 s_pPublishSrc 指向它；换算输出的单位为
 *            克重 / APP_FORCE_CALIB_OUT_UNIT_DIV（默认 2g 单位；标定表与
 *            PUT 下发仍是克重 g，该非对称关系见 app_force_calib.h）；
 *        这样可避免"换算结果被写回工作区、下一轮又被当作 AD 增量再减零点/再查表"
 *        的二次换算与量纲污染（尤其某轮某区域未刷新 raw、如 SPI 失败保留旧值时）。
 *        阶段二受 APP_FORCE_CALIB_APPLY_EN 与标定"完整性门控"双重约束：仅当上电判定
 *        的标定表完整（AppForceCalib_IsCalibComplete()==true）时 6 个部位才按
 *        「区域 AD 总和 -> LUT 整数插值（克重）-> 截断除以输出单位宏 -> 按各点 AD 占比
 *        分摊 -> 单点限幅 255」换算力值（默认 2g 单位，单点上限 255 即 510g）；
 *        不完整则整手保持
 *        「减零点后的 AD 增量」供上位机采集标定数据（判定与插值全部收口在标定核心内）。
 *        本函数是每轮唯一执行零点减除的位置。
 */
static void Sample_ApplyCalib(void)
{
    tac_data_cfg_t *pTacData  = GetTacDataCfg();
    tac_data_cfg_t *pTacCalib = GetTacCalibData();
    tac_area_t u8Area;
    uint8_t u8Rows;
    uint8_t u8Cols;
    uint8_t u8Part;

    /* 阶段一：整手 15 个区域逐一「减零点」（工作区 cfg：raw -> AD 增量）。
       默认发布源 = 工作区（未换算时成品帧就是"减零点后的 AD 增量"） */
    s_pPublishSrc = (const tac_data_cfg_t *)pTacData;

    for (u8Area = (tac_area_t)0; u8Area < TAC_AREA_MAX; u8Area = (tac_area_t)((uint8_t)u8Area + 1U))
    {
        u8Rows = (uint8_t)(TacAreaRange[u8Area].row_end - TacAreaRange[u8Area].row_start + 1U);
        u8Cols = (uint8_t)(TacAreaRange[u8Area].col_end - TacAreaRange[u8Area].col_start + 1U);
        Sample_CalibrateZero(u8Rows, u8Cols,
                             Sample_AreaPtr(pTacData, u8Area),
                             Sample_AreaPtr(pTacCalib, u8Area));
    }

    /* 阶段二：按部位查标定表换算力值（部位 1~6），**结果只写独立成品帧 s_stOutCfg**。
       门控为"全有全无"：标定表不完整（上电判定一次，见 AppForceCalib_IsCalibComplete）
       时整手都不换算，发布源保持工作区（「减零点后 AD 增量」），供上位机采集标定
       数据；完整时才逐部位换算（部位内部仍会按自身 mask 判空早退）。
       APP_FORCE_CALIB_APPLY_EN=0 时换算整体不参与编译（当前默认 0，置 1 启用） */
#if (APP_FORCE_CALIB_APPLY_EN != 0U)
    if (true == AppForceCalib_IsCalibComplete())
    {
        /* 先整结构拷贝（pack(1) 纯字节矩阵，memcpy 安全），再在副本上就地换算 */
        (void)memcpy((void *)&s_stOutCfg, (const void *)pTacData, sizeof(tac_data_cfg_t));

        for (u8Part = 1U; u8Part <= APP_FORCE_CALIB_PART_MAX; u8Part++)
        {
            AppForceCalib_ApplyPart(u8Part, &s_stOutCfg);
        }

        s_pPublishSrc = (const tac_data_cfg_t *)&s_stOutCfg;   /* 发布源切换为力值成品帧 */

        if (0U == s_u8OutModeLogged)
        {
            s_u8OutModeLogged = 1U;
            DDL_Printf("[CAL] output=FORCE (unit=1/%ug, calib complete)\r\n",
                       (unsigned int)APP_FORCE_CALIB_OUT_UNIT_DIV);
        }
    }
    else if (0U == s_u8OutModeLogged)
    {
        s_u8OutModeLogged = 1U;
        DDL_Printf("[CAL] output=AD_DELTA (calib incomplete, skip conversion)\r\n");
    }
#else
    if (0U == s_u8OutModeLogged)
    {
        s_u8OutModeLogged = 1U;
        DDL_Printf("[CAL] output=AD_DELTA (APP_FORCE_CALIB_APPLY_EN=0)\r\n");
    }
    (void)u8Part;   /* 关闭换算时 u8Part 无使用者，显式丢弃以消除告警 */
#endif
}
#endif /* TAC_CALIB_EN */

/**
 * @brief 传感器零点校准流程（对区域矩阵做「减零点」，下溢钳 0）
 * @param row   区域矩阵行数（行优先存储时外维）
 * @param col   区域矩阵列数（行优先存储时每行元素数）
 * @param frame 区域矩阵数据首地址（原始 raw，就地改写为减零点后的 AD 增量）
 * @param calib 零基准矩阵首地址（与 frame 同尺寸，不修改）
 * @note  矩阵在 cfg/calib 结构中行优先连续存放，flatten 索引 offset=row*col+col_idx，
 *        与结构体二维数组 [row][col] 语义一致；以 void* 接收首地址，兼容
 *        结构体内定长二维数组（如 pTacData->thumb_dip）与 Sample_AreaPtr 返回的扁指针。
 *        判据：value > (calib + SAMPLE_ZERO_DEADZONE) 才输出 value-calib，否则输出 0；
 *        死区用于抑制静止零漂（零漂在 59~141 点的区域求和后会被放大，并在 LUT 低端
 *        最陡斜率处被整段放大成读数跳变）。SAMPLE_ZERO_DEADZONE=0 即关闭死区。
 */
static void Sample_CalibrateZero(uint8_t row, uint8_t col, void *frame, void *calib)
{
		uint8_t *pu8Frame = (uint8_t *)frame;
		uint8_t *pu8Calib = (uint8_t *)calib;
		const uint8_t zero_offset = SAMPLE_ZERO_DEADZONE; /* 静止钳零死区：增量未超过「零基准+死区」时钳 0（见宏定义处说明，0=关闭） */
    for (int i = 0; i < col; i++) 
		{
			for(int j = 0; j < row; j++)
			{
					uint16_t value = pu8Frame[j * col + i];
					value = (value > (pu8Calib[j * col + i]+zero_offset)) ? (value - (pu8Calib[j * col + i])) : 0;
					pu8Frame[j * col + i] = (uint8_t)(value&0xff);
			}
    }
}
