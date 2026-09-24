/**
 *******************************************************************************
 * @file        exit.c
 * @brief       PH0(CLK_SYNC) 外部触发同步信号驱动
 * @note        上升边沿触发外部中断，中断内通过 FreeRTOS 二值信号量通知任务
 ******************************************************************************/
#include "exit.h"

/*******************************************************************************
 * 私有变量
 ******************************************************************************/
/* 二值信号量静态缓冲区 */
static StaticSemaphore_t m_stcSyncSemBuffer;
/* 同步信号量句柄（PH0 上升沿触发时由 ISR 给出） */
SemaphoreHandle_t m_hSyncSem = NULL;

/*******************************************************************************
 * 静态函数声明
 ******************************************************************************/
static void Exit_SyncIrqCallback(void);

/*******************************************************************************
 * @brief  配置 PH0 上升沿外部中断（CLK_SYNC 同步信号）
 ******************************************************************************/
void Exit_Sync_Config(void)
{
    stc_gpio_init_t         stcGpioInit;
    stc_extint_init_t       stcExtIntInit;
    stc_irq_signin_config_t stcIrqSignConfig;

    /* 1. PH0 配置为输入 + 上拉 + 使能引脚外部中断 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_IN;         /* 输入 */
    stcGpioInit.u16PullUp = PIN_PU_ON;          /* 上拉 */
    stcGpioInit.u16ExtInt = PIN_EXTINT_ON;      /* 使能外部中断功能 */
    (void)GPIO_Init(EXIT_SYNC_PORT, EXIT_SYNC_PIN, &stcGpioInit);

    /* 2. 外部中断通道0：上升边沿触发 */
    (void)EXTINT_StructInit(&stcExtIntInit);
    stcExtIntInit.u32Edge   = EXTINT_TRIG_RISING;   /* 上升沿触发 */
    stcExtIntInit.u32Filter = EXTINT_FILTER_OFF;    /* 关闭滤波，保证响应速度 */
    (void)EXTINT_Init(EXIT_SYNC_EXTINT_CH, &stcExtIntInit);

    /* 3. 使能引脚外部中断 */
    GPIO_ExtIntCmd(EXIT_SYNC_PORT, EXIT_SYNC_PIN, ENABLE);

    /* 4. 中断源映射到 IRQ 并注册回调 */
    stcIrqSignConfig.enIntSrc    = EXIT_SYNC_INT_SRC;
    stcIrqSignConfig.enIRQn      = EXIT_SYNC_IRQn;
    stcIrqSignConfig.pfnCallback = Exit_SyncIrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(EXIT_SYNC_IRQn);
    NVIC_SetPriority(EXIT_SYNC_IRQn, EXIT_SYNC_IRQ_PRIO);
    NVIC_EnableIRQ(EXIT_SYNC_IRQn);
}

/*******************************************************************************
 * @brief  创建同步二值信号量
 * @retval EXIT_OK  成功
 * @retval EXIT_ERR 失败
 ******************************************************************************/
uint8_t Exit_Sync_CreateSem(void)
{
    if (m_hSyncSem != NULL) {
        return EXIT_OK;     /* 已创建 */
    }

    m_hSyncSem = xSemaphoreCreateBinaryStatic(&m_stcSyncSemBuffer);
    if (m_hSyncSem == NULL) {
        return EXIT_ERR;
    }
    return EXIT_OK;
}

/*******************************************************************************
 * @brief  任务侧等待同步信号
 * @param  u32TimeoutMs 超时时间(ms)
 * @retval EXIT_OK          收到同步信号
 * @retval EXIT_ERR         信号量未创建
 * @retval EXIT_ERR_TIMEOUT 等待超时
 ******************************************************************************/
uint8_t Exit_Sync_Wait(uint32_t u32TimeoutMs)
{
    if (m_hSyncSem == NULL) {
        return EXIT_ERR;
    }

    if (xSemaphoreTake(m_hSyncSem, pdMS_TO_TICKS(u32TimeoutMs)) == pdTRUE) {
        return EXIT_OK;
    }
    return EXIT_ERR_TIMEOUT;
}

/*******************************************************************************
 * @brief  PH0 外部中断回调（上升沿触发）
 * @note   先清中断标志，再给出二值信号量，最后视情况触发任务切换
 ******************************************************************************/
static void Exit_SyncIrqCallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* 必须先清中断标志，否则会重复进入中断 */
    EXTINT_ClearExtIntStatus(EXIT_SYNC_EXTINT_CH);

    /* 上升沿触发：给出同步信号量 */
    if (m_hSyncSem != NULL) {
        xSemaphoreGiveFromISR(m_hSyncSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
