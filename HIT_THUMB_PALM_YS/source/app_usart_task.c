#include "app_usart_task.h"
#include "app_usart_tx_task.h"   /* 发送出口 AppUsartTxTask_Send / 队列清空 / 发送空闲查询 */
#include "usart_bsp.h"
#include "main.h"
#include "app_sample_task.h"
#include "app_tac_config.h"
#include "app_official_proto.h"
#include "app_glove_proto.h"      /* 灏存上传与标定协议：3C 0x07 命令 + 0x02 上传组帧 */
#include "app_period_timer.h"    /* 周期节拍：caliber 上传/125Hz 上报定时器启停与时间戳 */
#include "app_iap.h"
#include <string.h>

/******************************************************************************
 * 架构说明：
 *  - 接收：BSP 接收超时(RTO)中断登记双缓冲段后仅 give 接收信号量(m_xRxSem)，
 *    ISR 内不做解析；usart 任务 take 信号量后以 GetRxFrame 把段拷到任务缓冲，
 *    在任务上下文完成 0xAA 攒帧/切帧、3C 标定命令校验、UPGRADE 识别与
 *    OfficialProto_Process 解析应答（单实例串行，无 ISR 抢占冲突）。
 *  - 发送：发送队列与发送引擎由独立任务 app_usart_tx_task 承担（见其文件头）。
 *    业务出口（0xAA 官方应答 / 3C ACK）统一走 AppUsartTxTask_Send() 入队
 *    (非阻塞、线程安全)后即返回；发送任务作为唯一发送者逐帧"阻塞取帧 →
 *    USART_BSP_Send 启动 DMA → 等 TX_CPLT 发送完成通知 → 下一帧"，接收解析与
 *    线上发送两条时间线并行，互不拖慢。
 *  - 周期节拍：caliber 周期上传(10ms)与官方协议 125Hz 周期上报(8ms)两个节拍由
 *    app_period_timer 的软件定时器回调驱动（守护任务上下文，见该模块说明）；
 *    本任务只在其 start/stop 与 0x15 开关路径上启停对应定时器。
 *  - 空闲驱动：任务无接收处理、发送侧空闲、且周期节拍近期未驱动过采集时，按
 *    USART_IDLE_DRIVE_PERIOD_MS 空闲节拍通知 sample 任务驱动一轮采集，
 *    保持 tac 数据缓存最新供随时查询。
 ******************************************************************************/

/******************************************************************************
 * 静态任务资源（仅本文件可见，静态分配）
 ******************************************************************************/
static StaticTask_t  xUsartTaskTcb;
static StackType_t   uxUsartTaskStack[USART_TASK_STACK_SIZE];
TaskHandle_t  xUsartTaskHandle = NULL;

/* 接收完成二值信号量：RTO 中断 give，usart 任务 take 后消费接收段 */
static StaticSemaphore_t m_xRxSemBuffer;
SemaphoreHandle_t m_xRxSem = NULL;

/* 发送队列/发送引擎见 app_usart_tx_task.c：本任务只经 AppUsartTxTask_Send() 入队；
   出队发送、等待发送完成通知、队列清空与发送空闲查询见 app_usart_tx_task.h */

/* 断帧自愈：段间隔超 20ms 判定链路断帧 */
#define USART_AA_FRAME_TIMEOUT_MS (20U)
/* usart 任务空闲节拍(ms)：无接收段/无待发帧时按此周期驱动一轮采集，
 * 与采样周期 SAMPLE_PERIOD_MS=10ms 保持一致 */
#define USART_IDLE_DRIVE_PERIOD_MS (5U)

/******************************************************************************
 * 官方协议 125Hz 周期上报（协议 v1.8 set_touch_sensor_cycle_sw，0x30/0x15）
 *  - 开关上电默认关闭（app_official_proto.c s_u8CycleReportEn=0），上位机可发
 *    0x30/0x15 关闭/开启，应答回显当前状态；
 *  - 开启时按 OFFICIAL_CYCLE_REPORT_PERIOD_MS(8ms=125Hz) 主动投递一轮
 *    get_ori_data 风格 4 分包帧（cmd_set=0x30/cmd_id=0x10/cmd_type=0x02，
 *    与查询应答逐字节同格式，上位机零成本复用解析）；
 *  - 节拍实现：app_period_timer 的 8ms 自动重载软件定时器回调（本任务仅负责
 *    启停定时器）；
 *  - 带宽：整轮 4 帧共 814B，8ms 周期在 3000000 波特(8N1)链路上约占 2.71ms(≈34%)；
 *  - 暂停场景：caliber 周期上传中、升级命令触发后（判定见 app_period_timer 周期
 *    上报回调；启停见 OfficialProto_SetCycleReport 与本文件 caliber start/stop）。
 ******************************************************************************/

/******************************************************************************
 * 标定周期上传模式（caliber mode）上位机文本命令定义
 *  - "caliber mode start"：进入周期上传模式，按 USART_CALIBER_UPLOAD_PERIOD_MS
 *    周期主动上传 6 帧 3C 灏存格式触觉数据帧（Channel 0x02，descriptor 0x11~0x61），
 *    并屏蔽 0xAA 官方协议 / Y-modem；0x07 标定/查询/擦除帧在模式内正常受理
 *    （标定流程：start -> 采集 -> PUT 标定表 -> 状态查询 -> stop）；
 *  - "caliber mode stop" ：退出周期上传模式，清空残留待发帧并恢复全部入站协议。
 * 命令为 ASCII 子串匹配（区分大小写，不要求换行结尾），与既有 UPGRADE 升级命令
 * 识别风格一致；RTO 分段可能把命令切断，故按段逐字节累积到定长滑动窗口后再匹配。
 *
 * 上传内容沿用当前既有流程产出的数据（已减零点；已标定部位已换算力值，默认 2g 单位）。
 ******************************************************************************/
#define USART_CALIBER_CMD_START         "caliber mode start"
#define USART_CALIBER_CMD_STOP          "caliber mode stop"
#define USART_CALIBER_ASCII_BUF_LEN     (32U)   /* 滑动窗口长度（>= 长命令 18 字节） */
#define USART_CALIBER_FRAME_NUM         (6U)    /* 一轮上传帧数：5 指 + 掌心（每部位一帧） */

/******************************************************************************
 * 0xAA 请求攒帧缓冲（仅 usart 任务上下文访问，单实例无并发）
 * USB 桥可能把一帧拆成多段到达，解析入口把段拼入 s_au8AaAcc；攒齐整帧后由
 * AppUsart_HandleAaTask 直接调用 OfficialProto_Process（任务上下文，串行）。
 * 断帧超时自愈在每次新段到达时于解析入口判定。
 ******************************************************************************/
static uint8_t    s_au8AaAcc[USART_BSP_MAX_FRAME_LEN];   /* AA 攒帧缓冲(1KB) */
static uint16_t   s_u16AaAccLen = 0U;
static TickType_t s_u32AaAccTick = 0U;                   /* 最近写入攒帧缓冲的时刻 */

/******************************************************************************
 * 3C 攒帧缓冲（仅 usart 任务上下文访问，单实例无并发）
 * 灏存外层帧：3C 3C | Channel | Flags | Length(u16 LE) | Payload | CRC16(LE,
 * 只校验 Payload) | 3E 3E；完整帧长度 = Length + 10。PUT 写表 Payload 最长 512B
 * （3 区域×62 点为 751B 超容量，>512 的 Length 直接按失步丢弃），帧最长 522B，
 * USART_BSP_MAX_FRAME_LEN(1KB) 足够。断帧超时自愈与 AA 攒帧同策略。
 ******************************************************************************/
static uint8_t    s_au83cAcc[USART_BSP_MAX_FRAME_LEN];   /* 3C 攒帧缓冲(1KB) */
static uint16_t   s_u163cAccLen = 0U;
static TickType_t s_u323cAccTick = 0U;                   /* 最近写入攒帧缓冲的时刻 */

/******************************************************************************
 * caliber 周期上传模式状态（usart 任务写 / 定时器回调读，跨上下文共享）
 ******************************************************************************/
/* volatile：usart 任务置位(caliber start/stop)、定时器守护任务回调读取(10ms 上传
   节拍与 8ms 周期上报互斥判定)，跨上下文必须保证可见性（单字节读写原子，无需临界区） */
static volatile uint8_t s_u8CaliberUploadEn = 0U;   /* 1=周期上传中（屏蔽非 stop 的一切入站协议） */

/******************************************************************************
 * ASCII 命令滑动窗口（仅 usart 任务上下文访问）
 * 注：两个周期节拍的状态与组帧工作缓冲均由 app_period_timer 自持（定时器回调
 *     上下文，与本任务隔离），本文件不持有节拍时间戳
 ******************************************************************************/
static uint8_t    s_au8AsciiAcc[USART_CALIBER_ASCII_BUF_LEN];  /* ASCII 命令滑动窗口 */
static uint16_t   s_u16AsciiAccLen       = 0U;

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void AppTaskUsart(void *pvParameters);
static void Usart_ParseFrameTask(const uint8_t *pu8Data, uint16_t u16Len);
static void AppUsart_HandleAaTask(void);
static void AppUsart_Parse3cTask(const uint8_t *pu8Data, uint16_t u16Len);
/* caliber 周期上传模式 */
static void     AppUsart_ParseAsciiCmdTask(const uint8_t *pu8Data, uint16_t u16Len);
static uint8_t  Usart_AsciiFind(const char *pcszCmd);

/******************************************************************************
 * 对外接口实现
 ******************************************************************************/

/**
 * @brief  初始化 USART 任务：BSP 硬件 + 接收信号量 + 静态任务
 * @retval 1=创建成功，0=失败（信号量或任务创建失败）
 * @note   发送队列与发送任务由 APP_UsartTx_TaskCreate() 创建，须先于本函数调用；
 *         可在调度器启动前调用（不阻塞）
 */
uint8_t APP_Usart_TaskCreate(void)
{
    /* 1. 硬件初始化 */
    USART_BSP_Init();

    /* 2. 创建接收二值信号量(静态)：RTO 中断 give，usart 任务 take */
    m_xRxSem = xSemaphoreCreateBinaryStatic(&m_xRxSemBuffer);
    if (m_xRxSem == NULL) {
        return 0U;
    }

    /* 3. 创建静态任务（发送队列与发送任务由 APP_UsartTx_TaskCreate 创建，须先于本函数调用） */
    xUsartTaskHandle = xTaskCreateStatic(
        AppTaskUsart,
        "UsartTask",
        USART_TASK_STACK_SIZE,
        NULL,
        USART_TASK_PRIORITY,
        uxUsartTaskStack,
        &xUsartTaskTcb
    );

    if (xUsartTaskHandle != NULL) {
        DDL_Printf("UsartTask success!\r\n");
        return 1U;
    }
    return 0U;
}

/**
 * @brief  查询 caliber 周期上传模式是否使能（跨上下文访问接口）
 * @retval 1=周期上传中，0=未使能
 * @note   定时器回调（10ms 上传节拍 / 8ms 周期上报互斥判定）在守护任务上下文读取
 *         该标志，usart 任务在 start/stop 命令路径写入，故标志声明为 volatile。
 */
uint8_t AppUsart_IsCaliberUploadEn(void)
{
    return (s_u8CaliberUploadEn != 0U) ? 1U : 0U;
}

/******************************************************************************
 * @brief  任务主体：接收调度器（接收消费 + 空闲驱动采集）
 * @param  pvParameters  未使用
 * @note   本任务最高应用优先级；阻塞点仅"等接收信号量(上限 5ms)"，
 *         调度器启动后由 main() 创建，运行期不退出。
 * 每轮：
 *  1) 等接收信号量(超时=空闲节拍)：收到则排空双缓冲全部待处理段，逐段
 *     Usart_ParseFrameTask 完成攒帧/3C/升级/官方协议解析，应答帧经
 *     AppUsartTxTask_Send 入发送队列（由 app_usart_tx_task 发送任务逐帧发出，
 *     本任务不再等待发送完成）；
 *  2) 若本轮无接收处理、且发送侧空闲（无待发帧、无帧在发）、且周期节拍近期未
 *     驱动过采集，通知 sample 任务驱动一轮采集。
 *
 * 周期节拍已迁出：caliber 周期上传(10ms)与官方协议 125Hz 周期上报(8ms)由
 * app_period_timer 的两个软件定时器回调驱动（守护任务上下文，见该模块说明），
 * 本任务只在其 start/stop 与 0x15 开关路径上启停对应定时器
 * （见 AppUsart_ParseAsciiCmdTask 与 OfficialProto_SetCycleReport）。
 ******************************************************************************/
static void AppTaskUsart(void *pvParameters)
{
    static uint8_t au8RxBuf[USART_BSP_MAX_FRAME_LEN];   /* 段拷贝目标(函数级 static，避大栈帧) */
    uint16_t u16RxLen = 0U;
    uint8_t u8Busy;                                     /* 本轮是否处理过接收(决定是否驱动采样) */
    (void)pvParameters;

    /* 首轮：启动周期节拍（此处调度器已运行，定时器命令必定被守护任务处理）；
       按当前周期上报开关启动 8ms 上报定时器，caliber 定时器待模式进入时启动 */
    AppPeriodTimer_Start();

    for (;;) {
        u8Busy = 0U;

        /* 1. 等接收信号量或空闲节拍唤醒(本任务最高应用优先级，等待时让出 CPU)
              等待上限固定 USART_IDLE_DRIVE_PERIOD_MS(5ms)：周期节拍由软件定时器
              承担，本循环无需为节拍收敛等待上限。 */
        if (xSemaphoreTake(m_xRxSem, pdMS_TO_TICKS(USART_IDLE_DRIVE_PERIOD_MS)) == pdTRUE) {
            u8Busy = 1U;
            /* 排空双缓冲中全部待处理段：二值信号量可能合并多次 give(最多 2 段)，
               以 IsRxFrameDone 轮询消费保证不丢段。每段：拷贝取段→任务内解析→释放 */
            while (USART_BSP_IsRxFrameDone() != 0U) {
                u16RxLen = USART_BSP_GetRxFrame(au8RxBuf, sizeof(au8RxBuf));
                if (u16RxLen > 0U) {
                    Usart_ParseFrameTask(au8RxBuf, u16RxLen);
                }
                USART_BSP_ClearRxFlag();   /* 释放本段(背压停收时自动恢复接收) */
            }
        }

        /* 2. 发送由 app_usart_tx_task 任务承担（USART 发送通道唯一发送者）：本任务
              解析出的数据帧与周期节拍产出的帧（由 app_period_timer 回调入队）统一
              经该任务逐帧发出，故本任务不会阻塞在发送上。 */

        /* 3. 空闲节拍驱动采集：本轮未处理接收(空闲)、且发送侧空闲
              (AppUsartTxTask_IsIdle)、且周期节拍近期未驱动过采集
              (AppPeriodTimer_GetLastDriveTick 跨上下文时间戳，避免定时器刚驱动完
              采集又立刻补驱动一轮)时，通知 sample 任务驱动一轮 SPI+ADC 采集
              (fingers 由 sample 再通知、完成回报链条不变)，保持共享数据缓存最新，
              供上位机随时查询实时数据(请求-应答)及 125Hz 周期上报取用。 */
        if ((u8Busy == 0U) && (AppUsartTxTask_IsIdle() != 0U) &&
            ((xTaskGetTickCount() - AppPeriodTimer_GetLastDriveTick())
              >= pdMS_TO_TICKS(USART_IDLE_DRIVE_PERIOD_MS))) {
            if (m_hSampleTask != NULL) {
                xTaskNotifyGive(m_hSampleTask);
            }
        }
    }
}

/**
 * @brief  接收段解析入口：段路由 + 攒帧 + 升级/3C 直接执行
 * @param  pu8Data  本段数据首地址
 * @param  u16Len   本段长度
 * @note   路由规则（调用上下文：usart 任务，单实例串行）
 *         - caliber 周期上传模式：ASCII 文本命令识别 + 3C 灏存帧（0x07 标定/查询/擦除）；
 *         - 段以 0xAA 开头或已有 AA 半帧在攒 → 官方协议攒帧路径；
 *         - 否则：UPGRADE 升级识别 / 3C 灏存帧(校验后交 app_glove_proto 执行)
 */
static void Usart_ParseFrameTask(const uint8_t *pu8Data, uint16_t u16Len)
{
    if ((pu8Data == NULL) || (u16Len == 0U)) {
        return;
    }

    /* caliber 周期上传模式：识别文本命令（"caliber mode stop" 是唯一出口）+
       3C 灏存帧；屏蔽 0xAA 官方协议与 Y-modem 升级识别 */
    if (s_u8CaliberUploadEn != 0U) {
        AppUsart_ParseAsciiCmdTask(pu8Data, u16Len);
        if (s_u8CaliberUploadEn != 0U) {
            AppUsart_Parse3cTask(pu8Data, u16Len);
        }
        return;
    }

    /* 断帧自愈：攒帧中途断尾超过 20ms 仍无后续段 -> 清空攒帧缓存(每次新段到达时判定) */
    if ((s_u16AaAccLen > 0U) &&
        ((xTaskGetTickCount() - s_u32AaAccTick) > pdMS_TO_TICKS(USART_AA_FRAME_TIMEOUT_MS))) {
        s_u16AaAccLen = 0U;
    }

    /* 段路由：0xAA 开头或已有 AA 半帧在攒 -> 官方协议攒帧路径（拆段点不在帧头时，
       后续段可能以任意字节开头，必须继续拼入攒帧缓冲）；否则为 3C 灏存/升级命令 */
    if ((pu8Data[0] == 0xAAU) || (s_u16AaAccLen > 0U)) {
        if (((uint32_t)u16Len + s_u16AaAccLen) > sizeof(s_au8AaAcc)) {
            s_u16AaAccLen = 0U;   /* 溢出防呆：丢弃整段，重新同步 */
        } else {
            memcpy(&s_au8AaAcc[s_u16AaAccLen], pu8Data, u16Len);
            s_u16AaAccLen += u16Len;
            s_u32AaAccTick = xTaskGetTickCount();
            AppUsart_HandleAaTask();   /* 切出整帧直接 OfficialProto_Process */
        }
    } else {
        /* 文本命令识别(caliber 模式开关)：与 UPGRADE 同为 ASCII 子串匹配，
           先于协议解析；本段若命中 start，则不再走升级/3C 解析（后续段由
           函数开头早退拦截） */
        AppUsart_ParseAsciiCmdTask(pu8Data, u16Len);
        if (s_u8CaliberUploadEn != 0U) {
            return;
        }
        /* 升级命令识别(app_iap.c 为纯 memcmp，任务上下文直接执行擦写/软复位) */
        if (1U == APP_IAP_CheckUpgradeCmd(pu8Data, u16Len)) {
            /* 升级触发即停发周期上报：避免擦写/复位窗口内与 Y-modem 抢串口
               (EnterUpgradeMode 会写标志并软复位，此后本任务不再运行) */
            OfficialProto_SetCycleReport(0U);
            APP_IAP_EnterUpgradeMode();
        } else {
            /* 3C 灏存帧（0x07 标定表下发/状态查询/擦除）攒帧校验并执行 */
            AppUsart_Parse3cTask(pu8Data, u16Len);
        }
    }
}
/**
 * @brief  0xAA 攒帧切帧：从攒帧缓冲连续切出完整 AA 请求帧并调用
 *         OfficialProto_Process 解析应答
 * @note   任务内串行单实例，无 ISR 抢占冲突；应答帧由 OfficialProto_Process
 *         内部经 AppUsartTxTask_Send 入发送队列，由 app_usart_tx_task 发送任务
 *         统一出队发送，保证多包应答帧序
 */
static void AppUsart_HandleAaTask(void)
{
    while (s_u16AaAccLen > 0U) {
        uint16_t u16Skip = 0U;
        uint16_t u16Need;

        /* 丢弃前导非 0xAA 垃圾字节（异常保护，正常不触发） */
        while ((u16Skip < s_u16AaAccLen) && (s_au8AaAcc[u16Skip] != 0xAAU)) {
            u16Skip++;
        }
        if (u16Skip > 0U) {
            memmove(s_au8AaAcc, &s_au8AaAcc[u16Skip], s_u16AaAccLen - u16Skip);
            s_u16AaAccLen -= u16Skip;
            if (s_u16AaAccLen == 0U) {
                break;
            }
        }
        if (s_u16AaAccLen < 3U) {
            break;   /* 帧头未齐，等待后续段到达再拼 */
        }
        /* len 越界(合法范围 12~254)=帧头损坏，无边界可依：逐字节丢，对齐下一 0xAA */
        if ((s_au8AaAcc[2] < 12U) || (s_au8AaAcc[2] > 254U)) {
            memmove(s_au8AaAcc, &s_au8AaAcc[1], s_u16AaAccLen - 1U);
            s_u16AaAccLen -= 1U;
            continue;
        }
        u16Need = (uint16_t)(s_au8AaAcc[2] + 2U);
        if (s_u16AaAccLen < u16Need) {
            break;   /* 半帧：攒齐后由下一次段到达触发 */
        }
        /* 攒齐整帧 -> 任务上下文直接解析并组应答（应答经发送队列，在 memmove 前调用） */
        (void)OfficialProto_Process(&s_au8AaAcc[0], u16Need);
        s_u16AaAccLen -= u16Need;
        if (s_u16AaAccLen > 0U) {
            memmove(s_au8AaAcc, &s_au8AaAcc[u16Need], s_u16AaAccLen);
        }
    }
}

/**
 * @brief  3C 灏存帧攒帧切帧：从攒帧缓冲连续切出完整 3C 帧并交 app_glove_proto 执行
 * @param  pu8Data  本段数据首地址
 * @param  u16Len   本段长度
 * @note   切帧后校验头尾与 CRC16；Channel 0x07 标定表下发/状态查询/擦除，标定
 *         命令处理期间由协议层自行暂停/恢复 0x02 周期上传；
 *         断帧超时自愈与 AA 攒帧同策略（每次新段到达时判定）；任务上下文调用
 */
static void AppUsart_Parse3cTask(const uint8_t *pu8Data, uint16_t u16Len)
{
    if ((pu8Data == NULL) || (u16Len == 0U)) {
        return;
    }

    /* 断帧自愈：攒帧中途断尾超过 20ms 仍无后续段 -> 清空攒帧缓存重新同步 */
    if ((s_u163cAccLen > 0U) &&
        ((xTaskGetTickCount() - s_u323cAccTick) > pdMS_TO_TICKS(USART_AA_FRAME_TIMEOUT_MS))) {
        s_u163cAccLen = 0U;
    }

    /* 段拼入攒帧缓冲（溢出防呆：整段丢弃，靠帧头重新同步） */
    if (((uint32_t)u16Len + s_u163cAccLen) > sizeof(s_au83cAcc)) {
        s_u163cAccLen = 0U;
    } else {
        memcpy(&s_au83cAcc[s_u163cAccLen], pu8Data, u16Len);
        s_u163cAccLen += u16Len;
        s_u323cAccTick = xTaskGetTickCount();
    }

    while (s_u163cAccLen > 0U) {
        uint16_t u16LenField;
        uint16_t u16Need;
        uint16_t u16CrcRecv;
        uint16_t u16CrcCalc;

        /* 丢弃前导非帧头字节（失步/垃圾字节：逐字节滑动重新对齐 3C 3C） */
        if ((s_au83cAcc[0] != 0x3CU) || (s_au83cAcc[1] != 0x3CU)) {
            memmove(&s_au83cAcc[0], &s_au83cAcc[1], s_u163cAccLen - 1U);
            s_u163cAccLen -= 1U;
            continue;
        }
        if (s_u163cAccLen < 6U) {
            break;   /* 帧头+Length 未齐，等待后续段 */
        }

        u16LenField = (uint16_t)(s_au83cAcc[4] | ((uint16_t)s_au83cAcc[5] << 8U));
        /* Length 超容量（PUT 接收缓冲 512B）= 非法帧，无边界可依：逐字节丢重新同步 */
        if (u16LenField > GLOVE_PUT_PAYLOAD_MAX) {
            memmove(&s_au83cAcc[0], &s_au83cAcc[1], s_u163cAccLen - 1U);
            s_u163cAccLen -= 1U;
            continue;
        }

        u16Need = (uint16_t)(u16LenField + 10U);
        if (s_u163cAccLen < u16Need) {
            break;   /* 半帧：攒齐后由下一次段到达触发 */
        }

        /* 校验帧尾 + CRC16(只覆盖 Payload) */
        if ((s_au83cAcc[u16Need - 2U] != 0x3EU) || (s_au83cAcc[u16Need - 1U] != 0x3EU)) {
            memmove(&s_au83cAcc[0], &s_au83cAcc[1], s_u163cAccLen - 1U);
            s_u163cAccLen -= 1U;
            continue;
        }
        u16CrcRecv = (uint16_t)(s_au83cAcc[6U + u16LenField] |
                                ((uint16_t)s_au83cAcc[7U + u16LenField] << 8U));
        u16CrcCalc = AppGloveProto_Crc16(&s_au83cAcc[6], u16LenField);
        if (u16CrcRecv != u16CrcCalc) {
            memmove(&s_au83cAcc[0], &s_au83cAcc[1], s_u163cAccLen - 1U);
            s_u163cAccLen -= 1U;
            continue;   /* CRC 错误帧由接收方丢弃（协议第 1 节） */
        }

        /* 合法帧 -> 协议层执行（应答经发送队列，在 memmove 前调用） */
        AppGloveProto_HandleCmdFrame(s_au83cAcc[2], s_au83cAcc[3],
                                     &s_au83cAcc[6], u16LenField);
        s_u163cAccLen -= u16Need;
        if (s_u163cAccLen > 0U) {
            memmove(&s_au83cAcc[0], &s_au83cAcc[u16Need], s_u163cAccLen);
        }
    }
}

/******************************************************************************
 * caliber 周期上传模式（一）：ASCII 文本命令识别
 ******************************************************************************/
/**
 * @brief  在 ASCII 滑动窗口中查找命令子串（区分大小写）
 * @param  pcszCmd  待查找的 ASCII 命令（以 '\0' 结尾）
 * @retval 1=命中；0=未命中
 */
static uint8_t Usart_AsciiFind(const char *pcszCmd)
{
    uint16_t u16CmdLen = (uint16_t)strlen(pcszCmd);
    uint16_t i;

    if ((u16CmdLen == 0U) || (u16CmdLen > s_u16AsciiAccLen)) {
        return 0U;
    }
    for (i = 0U; i <= (s_u16AsciiAccLen - u16CmdLen); ++i) {
        if (0 == memcmp(&s_au8AsciiAcc[i], pcszCmd, u16CmdLen)) {
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief  ASCII 文本命令段累积与模式切换（start / stop）
 * @param  pu8Data  本段数据首地址
 * @param  u16Len   本段长度
 * @note   RTO 分段可能把命令切断，故逐字节追加到定长滑动窗口后再匹配；窗口满时
 *         左移一字节，始终保留最近 USART_CALIBER_ASCII_BUF_LEN 字节，无需超时复位。
 *         两条命令互不为子串，故按 start→stop 顺序判定即可。
 */
static void AppUsart_ParseAsciiCmdTask(const uint8_t *pu8Data, uint16_t u16Len)
{
    uint16_t i;

    for (i = 0U; i < u16Len; ++i) {
        if (s_u16AsciiAccLen < USART_CALIBER_ASCII_BUF_LEN) {
            s_au8AsciiAcc[s_u16AsciiAccLen++] = pu8Data[i];
        } else {
            (void)memmove(&s_au8AsciiAcc[0], &s_au8AsciiAcc[1],
                          (USART_CALIBER_ASCII_BUF_LEN - 1U));
            s_au8AsciiAcc[USART_CALIBER_ASCII_BUF_LEN - 1U] = pu8Data[i];
        }
    }

    if (Usart_AsciiFind(USART_CALIBER_CMD_START) != 0U) {
        if (s_u8CaliberUploadEn == 0U) {
            s_u8CaliberUploadEn = 1U;
            /* 从干净状态开始：丢弃进入前可能残留的 AA/3C 半帧与待发帧 */
            s_u16AaAccLen = 0U;
            s_u163cAccLen = 0U;
            AppUsartTxTask_QueueReset();
            /* 启动 10ms 上传节拍并停掉 125Hz 周期上报（二者互斥）：
               节拍由 app_period_timer 的软件定时器驱动，本任务不再自行计数 */
            AppPeriodTimer_CaliberStart();
            DDL_Printf("CALIBER_MODE_START\r\n");
        }
        s_u16AsciiAccLen = 0U;
    } else if (Usart_AsciiFind(USART_CALIBER_CMD_STOP) != 0U) {
        if (s_u8CaliberUploadEn != 0U) {
            /* 先清使能位：10ms 上传回调内也会判该状态（双保险），故即使停定时器
               的命令因守护任务队列满未送达，也不会再吐 3C 数据帧 */
            s_u8CaliberUploadEn = 0U;
            /* 停掉上传节拍，并按周期上报开关恢复 125Hz 上报（见 app_period_timer） */
            AppPeriodTimer_CaliberStop();
            /* 退出收尾：清 AA/3C 攒帧缓冲，并丢弃队列中尚未发出的 3C 数据帧，
               避免退出后继续吐旧数据；官方协议/3C/Y-modem 随即恢复正常解析 */
            s_u16AaAccLen = 0U;
            s_u163cAccLen = 0U;
            AppUsartTxTask_QueueReset();
            DDL_Printf("CALIBER_MODE_STOP\r\n");
        }
        s_u16AsciiAccLen = 0U;
    }
}

/******************************************************************************
 * caliber 周期上传模式（二）：灏存格式 3C 触觉数据帧批量上传
 *
 * 帧格式（灏存《上传与标定协议》第 3 节，每部位一帧，组帧与 CRC 由
 * app_glove_proto 收口）：
 *   3C 3C | 02 | 00 | len(LE) | descriptor | format | rows | cols |
 *   data[0..N-1]（uint8，有效点按传感点编号升序连续；掩码为 0 的无效点既不占位
 *   也不发送、不补 0；N=101/130/137/130/111/141，rows/cols 仍为物理矩阵尺寸） |
 *   CRC16(LE) | 3E 3E
 *   Payload 长度 = 4 + N（随有效点数变化，非 4 + rows×cols）。
 * 数据取自采样发布快照（已减零点；已标定部位已按 LUT 换算为力值分摊值，默认 2g 单位）。
 ******************************************************************************/

/**
 * @brief  一轮周期上传：按 部位1拇指→…→部位6掌心 顺序组帧并入队
 * @param  pu8FrameBuf  组帧工作缓冲（容量须 >= USART_BSP_MAX_FRAME_LEN，由调用方提供）
 * @retval 成功入队的帧数（0~6）
 * @note   入队走 AppUsartTxTask_Send（非阻塞、线程安全），由 app_usart_tx_task
 *         发送任务统一发出；
 *         队列满沿用"失败静默丢弃、不补发"语义。
 *         带宽：一轮 6 帧 = 拇指 115 + 食指 144 + 中指 151 + 无名指 144 +
 *         小指 125 + 掌心 155 = 834 字节（帧长 = 4 + N + 10），3Mbps(8N1 →
 *         10bit/字节) 下约 2.8ms 发完，10ms 周期可行（占用约 28%）。
 */
uint16_t AppUsart_UploadCaliberFrames(uint8_t *pu8FrameBuf)
{
    uint16_t u16Len;
    uint16_t u16Sent = 0U;
    uint8_t  i;

    if (pu8FrameBuf == NULL) {
        return 0U;
    }

    for (i = 1U; i <= USART_CALIBER_FRAME_NUM; ++i) {
        u16Len = AppGloveProto_BuildUploadFrame(i, pu8FrameBuf, USART_BSP_MAX_FRAME_LEN);
        if ((u16Len > 0U) && (u16Len <= USART_BSP_MAX_FRAME_LEN)) {
            if (AppUsartTxTask_Send(pu8FrameBuf, u16Len) == LL_OK) {
                u16Sent++;
            }
            /* 投递失败（发送队列满）：丢弃本帧、不补发，继续下一部位 */
        }
    }
    return u16Sent;
}
