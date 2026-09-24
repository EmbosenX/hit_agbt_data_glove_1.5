/*******************************************************************************
 * @file        app_period_timer.c
 * @brief       周期节拍模块实现：软件定时器驱动 caliber 周期上传与 125Hz 周期上报
 * @date        2026-09-16
 * @version     V1.0
 * @note        本模块承载 caliber 周期上传(10ms)与官方协议 125Hz 周期上报两个节拍；
 *              usart 任务只保留接收调度与命令解析职责，周期产帧由本模块独立完成。
 *
 * 上下文与并发：
 *  - 回调运行于定时器守护任务(configTIMER_TASK_PRIORITY = configMAX_PRIORITIES-1)，
 *    为全系统最高任务优先级，会抢占 usart(30) 与 sample/fingers(29)。每轮回调
 *    开销 = 通知采集 + 组帧 + 整帧拷贝入队（无阻塞），具体抢占窗口见各回调注释；
 *  - 组帧缓冲必须由本模块（或按上下文拆分的独立缓冲）提供，绝不能与 usart 任务
 *    侧共用：高优先级回调若在 usart 组帧中途抢占并改写同一缓冲，会产出撕裂帧。
 *    官方协议侧已按上下文拆双缓冲（见 app_official_proto.c OFFICIAL_BUF_CTX_*），
 *    caliber 帧缓冲由本模块自持（s_au8CaliberFrame，静态分配不落守护任务栈）；
 *  - 回调内只允许非阻塞调用：xTaskNotifyGive / AppUsartTxTask_Send(0 超时入队)，
 *    禁止互斥量、禁止 vTaskDelay 等任何可能阻塞的 API。
 ******************************************************************************/
#include "app_period_timer.h"
#include "app_official_proto.h"   /* OFFICIAL_CYCLE_REPORT_PERIOD_MS / GetCycleReport / SendCycleReport */
#include "app_glove_proto.h"      /* AppGloveProto_UploadPaused：标定命令处理期间暂停 caliber 上传 */
#include "app_usart_task.h"       /* USART_CALIBER_UPLOAD_PERIOD_MS / AppUsart_IsCaliberUploadEn / AppUsart_UploadCaliberFrames */
#include "app_sample_task.h"      /* m_hSampleTask：驱动一轮采集的通知目标 */
#include "usart_bsp.h"            /* USART_BSP_MAX_FRAME_LEN */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/******************************************************************************
 * 静态资源（静态分配，无堆）
 ******************************************************************************/
static StaticTimer_t s_stCycleReportTmr;   /* 周期上报定时器控制块(8ms/125Hz) */
static StaticTimer_t s_stCaliberTmr;       /* caliber 上传定时器控制块(10ms) */
static TimerHandle_t s_hCycleReportTmr = NULL;
static TimerHandle_t s_hCaliberTmr     = NULL;

/* 最近一次节拍驱动采集的时刻：usart 任务与定时器回调跨上下文共享，
   TickType_t 为 32 位单字读写(Cortex-M4 原子)，volatile 保证可见性 */
static volatile TickType_t s_u32LastDriveTick = 0U;

/* caliber 周期上传组帧缓冲（定时器回调上下文专用，与 usart 侧互不共享）：
   一轮 6 帧复用同一块，静态分配以避开守护任务栈（最大帧为掌心约 155B，见
   app_usart_task.c 组帧注释；USART_BSP_MAX_FRAME_LEN 为 1KB） */
static uint8_t s_au8CaliberFrame[USART_BSP_MAX_FRAME_LEN];

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void AppPeriodTimer_DriveSampleOnce(void);
static void AppPeriodTimer_CycleReportCb(TimerHandle_t xTimer);
static void AppPeriodTimer_CaliberUploadCb(TimerHandle_t xTimer);

/******************************************************************************
 * 内部函数实现
 ******************************************************************************/
/**
 * @brief  驱动一轮采集并记录驱动时刻
 * @note   sample 任务只与 fingers 任务握手、从不回报通知方，故此处只 give 通知、
 *         绝不在此等待回报（回调上下文禁止阻塞）。
 *         采集为"取轮末快照"语义（允许滞后一轮），数据一致性由 TacConfig
 *         双缓冲快照保证，读侧免锁。
 */
static void AppPeriodTimer_DriveSampleOnce(void)
{
    if (m_hSampleTask != NULL) {
        (void)xTaskNotifyGive(m_hSampleTask);
    }
    s_u32LastDriveTick = xTaskGetTickCount();
}

/**
 * @brief  8ms 周期上报回调（125Hz，协议 v1.8）
 * @note   暂停条件（此处再判一次，以防定时器命令丢失）：
 *          - 0x15 关闭周期上报(OfficialProto_GetCycleReport()==0)；
 *          - caliber 周期上传中(AppUsart_IsCaliberUploadEn()!=0)：否则与 3C 上传帧
 *            争抢发送队列并污染标定数据；
 *          - 升级触发时已由 OfficialProto_SetCycleReport(0) 关闭开关并随即软复位。
 */
static void AppPeriodTimer_CycleReportCb(TimerHandle_t xTimer)
{
    (void)xTimer;

    if ((OfficialProto_GetCycleReport() == 0U) || (AppUsart_IsCaliberUploadEn() != 0U)) {
        return;   /* 开关关闭或 caliber 上传中：本轮不产帧 */
    }

    /* 主动驱动一轮采集：保证上报数据新鲜（取轮末快照，允许滞后一轮） */
    AppPeriodTimer_DriveSampleOnce();

    /* 投递一轮 get_ori_data 风格 4 分包帧（写向上位机 0x90/0x00），4 帧在同一
       循环内全部入队构成单突发，随后由 app_usart_tx_task 发送任务统一发出 */
    OfficialProto_SendCycleReport();
}

/**
 * @brief  10ms caliber 周期上传回调
 * @note   使能时驱动一轮采集并把 6 帧 3C 数据帧入队；状态在回调内再判一次，
 *         防止 stop 命令因守护任务队列满未生效而继续吐旧数据。
 *         标定命令处理中（0x07 PUT 写 Flash / GET 查询）由协议层置暂停标志，
 *         本轮直接跳过，避免上传帧与标定命令应答交错（协议第 8 节）。
 */
static void AppPeriodTimer_CaliberUploadCb(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (AppUsart_IsCaliberUploadEn() == 0U) {
        return;   /* 已退出 caliber 模式：本轮不产帧 */
    }

    if (AppGloveProto_UploadPaused() != 0U) {
        return;   /* 标定命令处理中：本轮不产帧（本轮采集也不驱动，恢复后自然补上） */
    }

    /* 主动驱动一轮采集：上传取最近一次快照，允许滞后一轮 */
    AppPeriodTimer_DriveSampleOnce();

    (void)AppUsart_UploadCaliberFrames(s_au8CaliberFrame);
}

/******************************************************************************
 * 对外接口实现
 ******************************************************************************/
/**
 * @brief  创建两个周期节拍定时器（静态内存分配，不启动）
 * @retval 1=成功，0=失败
 * @note   须在 vTaskStartScheduler() 与 APP_Usart_TaskCreate() 之前调用；
 *         详见 app_period_timer.h
 */
uint8_t APP_PeriodTimer_Init(void)
{
    /* 自动重载(pdTRUE)：周期由守护任务按 reload 值持续排期，无需在回调内重启；
       定时器命令队列为内核静态分配(prvCheckForValidListAndQueue)，故调度器启动
       前创建安全；但启动命令必须在调度器运行后发出（见 AppPeriodTimer_Start） */
    s_hCycleReportTmr = xTimerCreateStatic("TmrCycleRpt",
                                           pdMS_TO_TICKS(OFFICIAL_CYCLE_REPORT_PERIOD_MS),
                                           pdTRUE, NULL,
                                           AppPeriodTimer_CycleReportCb,
                                           &s_stCycleReportTmr);

    s_hCaliberTmr = xTimerCreateStatic("TmrCaliber",
                                       pdMS_TO_TICKS(USART_CALIBER_UPLOAD_PERIOD_MS),
                                       pdTRUE, NULL,
                                       AppPeriodTimer_CaliberUploadCb,
                                       &s_stCaliberTmr);

    if ((s_hCycleReportTmr == NULL) || (s_hCaliberTmr == NULL)) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief  按当前开关状态启动周期上报定时器（usart 任务首轮调用）
 * @note   详见 app_period_timer.h；调度器启动前发出的定时器命令不生效，故不能放在 Init 内
 */
void AppPeriodTimer_Start(void)
{
    /* 仅按当前开关启动周期上报定时器；caliber 定时器保持停止，由
       AppPeriodTimer_CaliberStart() 在进入标定模式时启动 */
    if ((s_hCycleReportTmr != NULL) && (OfficialProto_GetCycleReport() != 0U)) {
        (void)xTimerStart(s_hCycleReportTmr, 0U);
    }
}

/**
 * @brief  周期上报开关变更收口（启停 8ms 上报定时器）
 * @param  u8En  1=开启周期上报，0=关闭
 * @note   详见 app_period_timer.h；caliber 上传期间即使开启也不启动上报定时器（互斥）
 */
void AppPeriodTimer_SetCycleReportEn(uint8_t u8En)
{
    if (s_hCycleReportTmr == NULL) {
        return;
    }

    if ((u8En != 0U) && (AppUsart_IsCaliberUploadEn() == 0U)) {
        (void)xTimerStart(s_hCycleReportTmr, 0U);
    } else {
        (void)xTimerStop(s_hCycleReportTmr, 0U);
    }
}

/**
 * @brief  进入 caliber 周期上传：停周期上报 + 启动 10ms 上传定时器
 * @note   两者互斥，避免争抢发送队列并污染标定数据；详见 app_period_timer.h
 */
void AppPeriodTimer_CaliberStart(void)
{
    /* 上报与上传互斥：进入标定模式先停周期上报，再启动 caliber 上传 */
    if (s_hCycleReportTmr != NULL) {
        (void)xTimerStop(s_hCycleReportTmr, 0U);
    }
    if (s_hCaliberTmr != NULL) {
        (void)xTimerStart(s_hCaliberTmr, 0U);
    }
}

/**
 * @brief  退出 caliber 周期上传：停上传定时器，按需恢复周期上报
 * @note   是否恢复取决于周期上报开关当前值（可能已被 0x15 关闭）；详见 app_period_timer.h
 */
void AppPeriodTimer_CaliberStop(void)
{
    if (s_hCaliberTmr != NULL) {
        (void)xTimerStop(s_hCaliberTmr, 0U);
    }
    /* 退出后按需恢复周期上报（开关可能已被 0x15 关闭或升级触发停发） */
    if ((s_hCycleReportTmr != NULL) && (OfficialProto_GetCycleReport() != 0U)) {
        (void)xTimerStart(s_hCycleReportTmr, 0U);
    }
}

/**
 * @brief  查询最近一次"节拍驱动采集"的时刻
 * @retval 最近驱动时刻的 xTaskGetTickCount 值（跨上下文共享，可为 0 表示尚未驱动）
 * @note   详见 app_period_timer.h
 */
TickType_t AppPeriodTimer_GetLastDriveTick(void)
{
    return s_u32LastDriveTick;
}
