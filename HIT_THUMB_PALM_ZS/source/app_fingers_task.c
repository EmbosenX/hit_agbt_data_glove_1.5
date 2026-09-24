/*******************************************************************************
 * @file        app_fingers_task.c
 * @brief       触觉手指业务任务实现
 * @author      
 * @date        2026-07-21
 * @version     V1.0
 ******************************************************************************/
#include "app_fingers_task.h"
#include "spi_fingers.h"
#include "app_sample_task.h"
#include "app_tac_config.h"
#include <string.h>
/*******************************************************************************
 * 私有宏定义
 ******************************************************************************/
#define FINGER_COUNT        5U
#define ADC_PAYLOAD_MAX     SPI_MAX_PAYLOAD_LEN
#define FINGER_SPI_AREA_NUM        6U  /* 单从机一帧内的区域段数（食指+无名指 / 中指+小指 各 6 段） */
#define FINGER_SPI_SEG_HEAD_LEN    3U  /* 单区域段头长度：区域标识 + 行数 + 列数 */
#define FINGER_LOG_OK_PERIOD       200U  /* SPI 正常轮摘要打印周期：每 200 轮输出一次，避免阻塞单轮耗时 */

typedef struct {
    uint8_t  u8Status;
    uint16_t u16DataLen;
    uint8_t  au8Data[ADC_PAYLOAD_MAX];
} finger_data_t;

/*******************************************************************************
 * 私有全局变量
 ******************************************************************************/
static StaticTask_t xAppFingersTaskTcb;
static StackType_t  uxAppFingersTaskStack[FINGER_TASK_STACK_SIZE];
TaskHandle_t xAppFingersTaskHandle = NULL;

/* SPI 采集轮完成计数：两从机均完成一轮采集后自增，用于降频摘要打印 */
static uint32_t s_u32OkRoundCnt = 0U;

/* 本轮 SPI 采集数据有效标志：两从机均成功采集并解析时置 1。
   volatile：跨任务访问（Fingers 任务写、Sample 任务读），由 fingers→sample 完成握手
   信号量 m_xFingerDoneSem 保证读写时序（give 前写本标志、take 后读本标志）。
   采样任务执行零点快照前须确认该标志，避免把上一轮已处理旧值当作零基准。 */
static volatile uint8_t s_u8LastRoundValid = 0U;

/*******************************************************************************
 * 静态内部函数声明
 ******************************************************************************/
static void Fingers_TaskEntry(void *pvParameters);
static uint8_t *Fingers_AreaBufPtr(tac_data_cfg_t *pTacData, tac_area_t u8Area);
static int32_t Fingers_ParseSlaveFrame(uint8_t u8SlaveIdx, const finger_data_t *pstcData);
/*******************************************************************************
 * 全局共享数据（SPI采集结果，供其他任务读取）
 ******************************************************************************/
/* 5个手指的采集数据缓存，SPI任务写，其他任务读 */
finger_data_t g_stcFingerData[FINGER_COUNT];
/*******************************************************************************
 * 对外接口实现
 ******************************************************************************/
/**
 * @brief 创建手指采集任务、队列、信号量
 */
void APP_Fingers_TaskCreate(void)
{
    xAppFingersTaskHandle = xTaskCreateStatic(
        Fingers_TaskEntry,
        "app fingers task",
        FINGER_TASK_STACK_SIZE,
        NULL,
        FINGER_TASK_PRIORITY,
        uxAppFingersTaskStack,
        &xAppFingersTaskTcb
    );
		if(xAppFingersTaskHandle!=NULL){DDL_Printf("AppFingersTask success!\r\n");}
}



/**
 * @brief 查询最近一轮 SPI 采集（两从机）是否均成功完成采集与解析
 * @return 1 = 本轮 index/ring 与 middle/little 两从机均 SPI_OK 且帧解析通过；
 *         0 = 任一从机 SPI 失败或帧校验被丢弃（cfg 手指区域保留上一轮旧值）
 * @note  标志在 give 完成握手信号量前写入（Fingers 写 / Sample 读由 m_xFingerDoneSem
 *        同步，无竞态），采样任务执行零点快照前须确认，避免把已处理旧值当作零基准。
 */
uint8_t Fingers_IsLastRoundValid(void)
{
    return s_u8LastRoundValid;
}

/*******************************************************************************
 * @brief  手指采集任务主体：等通知 -> 轮询两从机 SPI 读帧 -> 缓存解析 -> 上报完成
 * @param  pvParameters  未使用
 * @note   由采样任务（xAppFingersTaskHandle 通知）驱动，每轮对两片从机各做一次 SPI 读取
 *         （读帧 -> 校验帧头/CRC -> 解析入 cfg；从机1=食指/无名指，从机2=中指/小指）；
 *         任一从机通讯失败或帧校验失败即把本轮标记为无效（s_u8LastRoundValid=0），
 *         异常时打印告警；运行期不退出，最终通过 m_xFingerDoneSem 向采样任务回报本轮完成。
 ******************************************************************************/
static void Fingers_TaskEntry(void *pvParameters)
{
    (void)pvParameters;
    uint8_t  u8SlaveIdx=1;
    int32_t  s32Ret;
	
    Fingers_SPI_Config();
	
    for (;;) {
        /************************ 第一步：等待外部任务通知启动采集 ************************/
        /* pdTRUE：接收后清零通知值，等价于二值信号量；portMAX_DELAY：永久阻塞 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* 本轮起始假定两从机均有效；任一从机 SPI 失败或帧解析失败则清零 */
        s_u8LastRoundValid = 1U;
        /* 依次轮询从机 */
        for (u8SlaveIdx = 1U; u8SlaveIdx <= 2; u8SlaveIdx++) 
				{
            finger_data_t *pstcData = &g_stcFingerData[u8SlaveIdx - 1U];

            /* 发送读取命令并读取应答（帧头/CRC 校验由 Fingers_SPI_Transact 内部完成） */
            s32Ret = Fingers_SPI_Transact(u8SlaveIdx, SPI_CMD_READ_DATA,
                                          (DATA_FRAME_BYTE_NUM == 0U) ? ADC_FMT_8BIT : ADC_FMT_16BIT,
                                          ADC_PAYLOAD_MAX, pstcData->au8Data,
                                          &pstcData->u16DataLen, &pstcData->u8Status);

            if (s32Ret != SPI_OK) {
                pstcData->u8Status = 0xFFU; /* 标记通讯失败 */
                pstcData->u16DataLen = 0U;
                s_u8LastRoundValid = 0U; /* 本轮从机采集失败，数据无效 */
                /* 通讯失败即时告警（失败非常态，打印阻塞可接受） */
                DDL_Printf("SPI_ERR slave=%u ret=%d!\r\n", (unsigned int)u8SlaveIdx, (int)s32Ret);
            }
            else if (Fingers_ParseSlaveFrame(u8SlaveIdx, pstcData) != 0) {
                s_u8LastRoundValid = 0U; /* 帧校验失败整帧丢弃，cfg 手指区域保留旧值 */
            }
            else {
                /* 解析成功：本轮该手指区域数据已更新 */
            }
        }
        /************************ 第二步：SPI 采集轮周期摘要（降频打印） ************************/
        /* 降频摘要：两从机全部完成后仅每 FINGER_LOG_OK_PERIOD 轮输出一次 */
        s_u32OkRoundCnt++;
        if ((s_u32OkRoundCnt % FINGER_LOG_OK_PERIOD) == 0U) {
            /* 降频摘要：仅轮计数（加固前的摘要形式，无链路健康计数） */
            DDL_Printf("SPI_OK round=%u\r\n", (unsigned int)s_u32OkRoundCnt);
        }
        /************************ 第三步：通知外部控制任务，采集已完成 ************************/
        /* 【握手通道分离】本轮完成经 fingers→sample 专用信号量 m_xFingerDoneSem 上报，
           不复用 m_hSampleTask 的任务通知通道（该通道被 usart 采集发令独占）：
           否则采样任务对"本轮完成"的等待可能被 usart 发令提前满足，从而在其尚未
           消费完本轮 cfg 手指区时就开始下一轮整手处理，读到上一轮旧手指数据 */
        if (m_xFingerDoneSem != NULL) {
            (void)xSemaphoreGive(m_xFingerDoneSem);
        }
    }
}

/*******************************************************************************
 * 从机SPI载荷解析（缓存至全局 tac_data_cfg 对应手指区域矩阵）
 ******************************************************************************/
/**
 * @brief  返回 tac_data_cfg 中指定区域矩阵的首字节地址（仅食指/中指/无名指/小指区域）
 * @param  pTacData 目标数据源（由 GetTacDataCfg() 返回）
 * @param  u8Area   区域枚举（TAC_AREA_INDEX_DIP ~ TAC_AREA_LITTLE_MCP）
 * @return 区域矩阵首地址；区域不受支持时返回 NULL
 */
static uint8_t *Fingers_AreaBufPtr(tac_data_cfg_t *pTacData, tac_area_t u8Area)
{
    if (pTacData == NULL) {
        return NULL;
    }

    switch (u8Area) {
    case TAC_AREA_INDEX_DIP:    return (uint8_t *)&pTacData->index_dip[0][0];
    case TAC_AREA_INDEX_PIP:    return (uint8_t *)&pTacData->index_pip[0][0];
    case TAC_AREA_INDEX_MCP:    return (uint8_t *)&pTacData->index_mcp[0][0];
    case TAC_AREA_MIDDLE_DIP:   return (uint8_t *)&pTacData->middle_dip[0][0];
    case TAC_AREA_MIDDLE_PIP:   return (uint8_t *)&pTacData->middle_pip[0][0];
    case TAC_AREA_MIDDLE_MCP:   return (uint8_t *)&pTacData->middle_mcp[0][0];
    case TAC_AREA_RING_DIP:     return (uint8_t *)&pTacData->ring_dip[0][0];
    case TAC_AREA_RING_PIP:     return (uint8_t *)&pTacData->ring_pip[0][0];
    case TAC_AREA_RING_MCP:     return (uint8_t *)&pTacData->ring_mcp[0][0];
    case TAC_AREA_LITTLE_DIP:   return (uint8_t *)&pTacData->little_dip[0][0];
    case TAC_AREA_LITTLE_PIP:   return (uint8_t *)&pTacData->little_pip[0][0];
    case TAC_AREA_LITTLE_MCP:   return (uint8_t *)&pTacData->little_mcp[0][0];
    default:                    return NULL;
    }
}

/**
 * @brief  解析单个从机的 SPI 回复载荷，并按区域段缓存至 tac_data_cfg 手指矩阵
 * @param  u8SlaveIdx 从机序号：1 = 食指+无名指（HIT_INDEX_RING），2 = 中指+小指（HIT_MIDDLE_LITTLE）
 * @param  pstcData   SPI 读取结果（au8Data 已剥离应答头，即从机 Adc_ReadFrame 输出的 6 段载荷）
 * @retval 0  解析成功，数据已缓存
 * @retval -1 参数错误或帧格式不符（整帧丢弃，不写脏旧数据）
 *
 * @note  解析分两遍：第一遍只读校验全部 6 段（tag 顺序、行列非 0 且不超过主控矩阵
 *        容量、长度不越界），全部通过后第二遍才统一写入，保证一帧要么全部生效要么
 *        全不生效。从机源矩阵行/列可小于主控目标（如小指区域列宽差 1 列），此时按
 *        行优先逐行"贴左"存放，目标区域先清零避免残留旧行/旧列。
 * @note  第二遍写 tac_data_cfg 前取 m_hSampleDataMutex（写毕释放），与 sample 任务
 *        采集写临界段、usart 任务读应答共用同一把锁，防止读到本帧部分写入的半成品。
 *        第一遍只读校验源缓冲不触碰 cfg，无需持锁。
 */
static int32_t Fingers_ParseSlaveFrame(uint8_t u8SlaveIdx, const finger_data_t *pstcData)
{
    /* 从机序号 -> 区域段顺序映射表（段序即区域标识 tag 0x01~0x06） */
    static const tac_area_t s_astSlaveAreaMap[2U][FINGER_SPI_AREA_NUM] = {
        { TAC_AREA_INDEX_DIP,  TAC_AREA_INDEX_PIP,  TAC_AREA_INDEX_MCP,
          TAC_AREA_RING_DIP,   TAC_AREA_RING_PIP,   TAC_AREA_RING_MCP },        /* u8SlaveIdx = 1 */
        { TAC_AREA_MIDDLE_DIP, TAC_AREA_MIDDLE_PIP, TAC_AREA_MIDDLE_MCP,
          TAC_AREA_LITTLE_DIP, TAC_AREA_LITTLE_PIP, TAC_AREA_LITTLE_MCP }       /* u8SlaveIdx = 2 */
    };
    const tac_area_range_t *pstcRange;
    tac_data_cfg_t *pTacData;
    const uint8_t *pu8Seg;
    uint8_t *pu8Dst;
    uint16_t u16ElemBytes;
    uint16_t u16ExpRows;
    uint16_t u16ExpCols;
    uint16_t u16SrcRows;
    uint16_t u16SrcCols;
    uint16_t u16Row;
    uint32_t u32Off;
    uint8_t u8Idx;

    if ((u8SlaveIdx < 1U) || (u8SlaveIdx > 2U) || (pstcData == NULL)) {
        return -1;
    }
    if (pstcData->u16DataLen < FINGER_SPI_SEG_HEAD_LEN) {
        return -1;
    }

    pTacData = GetTacDataCfg();
    if (pTacData == NULL) {
        return -1;
    }
    /* 每个传感点字节数：DATA_FRAME_BYTE_NUM=0 单字节；=1 双字节 */
    u16ElemBytes = (uint16_t)((DATA_FRAME_BYTE_NUM != 0U) ? 2U : 1U);

    /* ============ 第一遍：只读整帧校验，任一不符则整帧丢弃 ============ */
    u32Off = 0U;
    for (u8Idx = 0U; u8Idx < FINGER_SPI_AREA_NUM; u8Idx++) {
        /* 1. 校验段头完整性（剩余长度 >= tag+行+列） */
        if ((u32Off + FINGER_SPI_SEG_HEAD_LEN) > pstcData->u16DataLen) {
            return -1;
        }
        pu8Seg = &pstcData->au8Data[u32Off];

        /* 2. 区域标识必须与段序一致（从机按 0x01~0x06 固定顺序发送） */
        if (pu8Seg[0] != (uint8_t)(u8Idx + 1U)) {
            return -1;
        }

        /* 3. 以主控区域范围表为准求目标矩阵容量（期望行/列） */
        pstcRange = &TacAreaRange[s_astSlaveAreaMap[u8SlaveIdx - 1U][u8Idx]];
        u16ExpRows = (uint16_t)(pstcRange->row_end - pstcRange->row_start + 1U);
        u16ExpCols = (uint16_t)(pstcRange->col_end - pstcRange->col_start + 1U);

        /* 4. 行/列声明须非 0 且不超出主控矩阵容量（兼容从机源矩阵窄于目标的情形） */
        if ((pu8Seg[1] == 0U) || (pu8Seg[2] == 0U)) {
            return -1;
        }
        if ((pu8Seg[1] > (uint8_t)u16ExpRows) || (pu8Seg[2] > (uint8_t)u16ExpCols)) {
            return -1;
        }
        u16SrcRows = pu8Seg[1];
        u16SrcCols = pu8Seg[2];

        /* 5. 校验本段矩阵数据长度不越出整帧 */
        if ((u32Off + FINGER_SPI_SEG_HEAD_LEN +
             (uint32_t)u16SrcRows * u16SrcCols * u16ElemBytes) > pstcData->u16DataLen) {
            return -1;
        }
        u32Off += (uint32_t)FINGER_SPI_SEG_HEAD_LEN +
                  (uint32_t)u16SrcRows * u16SrcCols * u16ElemBytes;
    }

    /* ============ 第二遍：整帧校验通过后统一写入（写 cfg 临界段持 m_hSampleDataMutex） ============ */
    if (m_hSampleDataMutex != NULL)
    {
        (void)xSemaphoreTake(m_hSampleDataMutex, portMAX_DELAY);
    }
    u32Off = 0U;
    for (u8Idx = 0U; u8Idx < FINGER_SPI_AREA_NUM; u8Idx++) {
        pu8Seg = &pstcData->au8Data[u32Off];
        u16SrcRows = pu8Seg[1];
        u16SrcCols = pu8Seg[2];

        pstcRange = &TacAreaRange[s_astSlaveAreaMap[u8SlaveIdx - 1U][u8Idx]];
        u16ExpRows = (uint16_t)(pstcRange->row_end - pstcRange->row_start + 1U);
        u16ExpCols = (uint16_t)(pstcRange->col_end - pstcRange->col_start + 1U);

        pu8Dst = Fingers_AreaBufPtr(pTacData, s_astSlaveAreaMap[u8SlaveIdx - 1U][u8Idx]);
        if (pu8Dst == NULL) {
            if (m_hSampleDataMutex != NULL)
            {
                (void)xSemaphoreGive(m_hSampleDataMutex);
            }
            return -1;
        }
        /* 先清零整个目标区域，避免源小于目标时残留旧行/旧列 */
        memset(pu8Dst, 0, (size_t)u16ExpRows * u16ExpCols * u16ElemBytes);
        /* 逐行贴左存放：目标行宽 ExpCols >= 源行宽 SrcCols，坐标同为 buffer[row][col] 行优先 */
        for (u16Row = 0U; u16Row < u16SrcRows; u16Row++) {
            memcpy(&pu8Dst[(size_t)u16Row * u16ExpCols * u16ElemBytes],
                   &pu8Seg[FINGER_SPI_SEG_HEAD_LEN] + (size_t)u16Row * u16SrcCols * u16ElemBytes,
                   (size_t)u16SrcCols * u16ElemBytes);
        }
        u32Off += (uint32_t)FINGER_SPI_SEG_HEAD_LEN +
                  (uint32_t)u16SrcRows * u16SrcCols * u16ElemBytes;
    }
    if (m_hSampleDataMutex != NULL)
    {
        (void)xSemaphoreGive(m_hSampleDataMutex);
    }

    return 0;
}




