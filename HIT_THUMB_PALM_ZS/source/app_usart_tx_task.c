/*******************************************************************************
 * @file        app_usart_tx_task.c
 * @brief       串口发送任务：应用层发送队列的唯一消费者（发送引擎）
 * @author
 * @date        2026-09-16
 * @version     V1.0
 * @note        本任务是 USART 发送通道的唯一发送者，与接收解析路径并行：
 *               - 生产者：usart 任务的协议解析路径与官方协议模块(app_official_proto.c)
 *                 经 AppUsartTxTask_Send() 整帧入队（非阻塞）后立即返回；
 *               - 消费者：本任务以 xQueueReceive(portMAX_DELAY) 阻塞取帧，无帧时挂起、
 *                 零 CPU 占用；队列内多帧时一次唤醒后连续发完（单突发，帧间不被其它
 *                 发送路径插入）；
 *               - 完成同步：DMA 模式下每帧启动后阻塞等 TX_CPLT 中断投递的任务通知，
 *                 超时(100ms)作为异常兜底：复位发送通道并清残留通知后继续下一帧。
 *              请求-应答串行语义与帧序由本任务保证。
 ******************************************************************************/
#include "app_usart_tx_task.h"
#include "usart_bsp.h"
#include <string.h>

/******************************************************************************
 * 静态任务资源（仅本文件可见，静态分配）
 ******************************************************************************/
static StaticTask_t  xUsartTxTaskTcb;
static StackType_t   uxUsartTxTaskStack[USART_TX_TASK_STACK_SIZE];
TaskHandle_t  xUsartTxTaskHandle = NULL;

/******************************************************************************
 * 发送消息队列：应用层各发送出口入队，本任务发送引擎出队并逐帧发送。
 * 深度 16 的依据（整批突发容量）：
 *  - 官方 get_ori_data 单次应答 4 包，组帧后一次性全部入队，再由发送引擎在一次
 *    出队循环内连续发出（单突发、帧间不被其它发送路径插入）；
 *  - AppUsart_HandleAaTask 会先把攒帧缓冲中**所有**完整请求帧解析完才回到等待，
 *    故上位机流水线连发时，队列峰值 = 同时到达的请求数 × 单请求应答包数；
 *  - 深度 16 可吸收 4 个 get_ori_data 请求同时到达（4×4=16 帧），并留有 3C ACK
 *    等零星帧的余量，避免整批应答因队列不足被丢弃。
 * 队列满时 AppUsartTxTask_Send 返回 LL_ERR_BUSY 丢弃本帧（沿用协议侧"失败静默不补发"，
 * 缺失包由请求方自行超时重查）。
 * RAM 代价：每项 sizeof(stc_usart_tx_msg_t) = 2 + USART_BSP_MAX_FRAME_LEN(1024) 字节，
 * 深度 16 约占 16KB 静态 RAM（RW_IRAM2 余量充足）；后续若要再调深度须先复核 RAM。
 ******************************************************************************/
#define USART_TX_QUEUE_DEPTH    (16U)
typedef struct {
    uint16_t u16Len;
    uint8_t  au8Data[USART_BSP_MAX_FRAME_LEN];
} stc_usart_tx_msg_t;
static StaticQueue_t m_xTxQueueBuffer;
static uint8_t m_au8TxQueueStorage[USART_TX_QUEUE_DEPTH * sizeof(stc_usart_tx_msg_t)];
static QueueHandle_t m_xTxQueue = NULL;
static stc_usart_tx_msg_t stcTxMsg   = {0U};   /* 发送引擎出队复用缓冲(文件级，避开大栈帧) */
static stc_usart_tx_msg_t stcFlushMsg = {0U};  /* 丢弃待发帧时的出队缓冲(文件级，同理) */

/* 发送完成通知等待超时(ms)：DMA 模式下阻塞等 TX_CPLT 中断的任务通知，
 * 超时作为异常兜底：复位发送通道并丢弃残留通知后继续下一帧 */
#define USART_TX_COMPLETE_TIMEOUT_MS (100U)

/* 帧发送进行中标志：本任务启动一帧 → 该帧发送完成(或超时兜底)期间置 1，
 * 供 AppUsartTxTask_IsIdle() 跨任务查询（volatile：被 usart 任务读取） */
static volatile uint8_t s_u8TxInProgress = 0U;

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void AppTaskUsartTx(void *pvParameters);

/******************************************************************************
 * 对外接口实现
 ******************************************************************************/

/**
 * @brief  初始化串口发送任务（发送队列 + 静态任务）
 * @note   只创建队列与任务；USART 硬件与接收链初始化在 APP_Usart_TaskCreate()
 *         （USART_BSP_Init）完成，本函数不碰硬件。
 */
uint8_t APP_UsartTx_TaskCreate(void)
{
    /* 1. 创建发送队列(静态)：应用层发送出口(AppUsartTxTask_Send)入队，发送引擎出队 */
    m_xTxQueue = xQueueCreateStatic(USART_TX_QUEUE_DEPTH,
                                    sizeof(stc_usart_tx_msg_t),
                                    m_au8TxQueueStorage,   /* 数据存储区 */
                                    &m_xTxQueueBuffer);    /* 队列控制块 */
    if (m_xTxQueue == NULL) {
        return 0U;
    }

    /* 2. 创建静态任务 */
    xUsartTxTaskHandle = xTaskCreateStatic(
        AppTaskUsartTx,
        "UsartTxTask",
        USART_TX_TASK_STACK_SIZE,
        NULL,
        USART_TX_TASK_PRIORITY,
        uxUsartTxTaskStack,
        &xUsartTxTaskTcb
    );

    if (xUsartTxTaskHandle != NULL) {
        DDL_Printf("UsartTxTask success!\r\n");
        return 1U;
    }
    return 0U;
}

/**
 * @brief  应用层发送接口（线程安全，任意任务/解析路径可调用，非阻塞）
 * @param  pu8Data  待发送整帧数据指针
 * @param  u16Len   数据长度（不得超过 USART_BSP_MAX_FRAME_LEN）
 * @retval LL_OK=已拷入发送队列；LL_ERR_INVD_PARAM=参数错误；
 *         LL_ERR_BUSY=队列满或未创建（本帧丢弃，调用方沿用"失败静默不补发"语义）
 * @note   整帧拷贝入静态发送队列后立即返回；实际发送由本任务发送引擎完成
 *         （阻塞取帧→USART_BSP_Send→等发送完成通知→下一帧）。队列满返回 LL_ERR_BUSY
 *         丢弃本帧，与协议侧"失败静默不补发"语义一致。本函数不阻塞，任务自身在解析
 *         路径调用时队列仍有余量(深度 16 ≥ 单次 get_ori_data 应答 4 包)，不会自死锁。
 */
int32_t AppUsartTxTask_Send(const uint8_t *pu8Data, uint16_t u16Len)
{
    stc_usart_tx_msg_t stcMsg;

    if ((pu8Data == NULL) || (u16Len == 0U) || (u16Len > USART_BSP_MAX_FRAME_LEN)) {
        return LL_ERR_INVD_PARAM;
    }
    if (m_xTxQueue == NULL) {
        return LL_ERR_BUSY;
    }

    stcMsg.u16Len = u16Len;
    memcpy(stcMsg.au8Data, pu8Data, u16Len);

    if (xQueueSend(m_xTxQueue, &stcMsg, 0U) != pdPASS) {
        return LL_ERR_BUSY;   /* 队列满：静默丢弃本帧(不补发) */
    }
    return LL_OK;
}

/**
 * @brief  丢弃发送队列中全部待发帧（任务上下文调用）
 * @note   【为什么不用 xQueueReset】本任务平时无限阻塞在该队列的接收等待列表上，
 *         此时调用 xQueueReset 会重置队列的等待列表、把本任务从调度链表中摘掉，
 *         导致任务永久挂起（此后入队也不再唤醒它，串口彻底发不出数据）。故改由
 *         调用方以 xQueueReceive(0) 逐帧取走丢弃：取帧不会触碰阻塞中的接收任务，
 *         安全且与 xQueueReset 等效（仅丢弃待发帧，不影响正在发送中的那一帧）。
 */
void AppUsartTxTask_QueueReset(void)
{
    if (m_xTxQueue == NULL) {
        return;
    }

    while (xQueueReceive(m_xTxQueue, &stcFlushMsg, 0U) == pdPASS) {
        ;   /* 逐帧取出并丢弃 */
    }
}

/**
 * @brief  查询发送侧是否空闲（无待发帧且当前无帧在发送中）
 * @retval 1=空闲；0=有发送活动
 */
uint8_t AppUsartTxTask_IsIdle(void)
{
    uint8_t u8Idle = 1U;

    if (m_xTxQueue != NULL) {
        if (uxQueueMessagesWaiting(m_xTxQueue) > 0U) {
            u8Idle = 0U;   /* 还有待发帧 */
        }
    }
    if (s_u8TxInProgress != 0U) {
        u8Idle = 0U;       /* 某帧正在发送/等完成通知 */
    }
    return u8Idle;
}

/******************************************************************************
 * @brief  任务主体：发送引擎（USART 发送通道的唯一发送者，帧序串行不穿插）
 * @param  pvParameters  未使用
 * @note   无帧时挂起在发送队列上（零 CPU 占用），运行期不退出
 * 每轮：
 *  1) 阻塞取一帧(portMAX_DELAY)：无帧时任务挂起、零 CPU 占用；队列内多帧时一次
 *     唤醒即连续发完（单突发）；
 *  2) USART_BSP_Send 启动 DMA（DMA 模式），随后阻塞等 TX_CPLT 完成通知；
 *  3) 超时(异常)：复位发送通道并清残留通知后继续下一帧；Send 返回非 LL_OK(忙/
 *     参数错误)：本帧未发出，丢弃继续下一帧。
 ******************************************************************************/
static void AppTaskUsartTx(void *pvParameters)
{
    (void)pvParameters;

    /* 进入循环前清一次残留通知：防止任务创建前(或异常路径)遗留的通知让首帧
       误判为"已发送完成"而提前复用缓冲/启动下一帧 */
    (void)ulTaskNotifyTake(pdTRUE, 0U);

    for (;;) {
        /* 阻塞等一帧：队列本身即同步原语，无需额外唤醒机制 */
        if (xQueueReceive(m_xTxQueue, &stcTxMsg, portMAX_DELAY) != pdPASS) {
            continue;   /* 仅异常返回；正常为无限等待，不会走到这里 */
        }

        s_u8TxInProgress = 1U;
        if (USART_BSP_Send(stcTxMsg.au8Data, stcTxMsg.u16Len) == LL_OK) {
#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_DMA)
            /* 阻塞等待 TX_CPLT 发送完成任务通知；超时(异常)复位发送通道兜底，
               并清残留通知防止下一帧误判"已发送完成" */
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(USART_TX_COMPLETE_TIMEOUT_MS)) == pdFALSE) {
                USART_BSP_TxAbort();
                (void)ulTaskNotifyTake(pdTRUE, 0U);
            }
#endif
        }
        /* USART_BSP_Send 返回非 LL_OK(忙/参数错误)：本帧未发出，丢弃继续下一帧 */
        s_u8TxInProgress = 0U;
    }
}
