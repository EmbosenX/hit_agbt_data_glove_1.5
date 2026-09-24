#include "usart_bsp.h"
#include "app_usart_task.h"      /* 需要 m_xRxSem：接收超时(RTO)中断释放接收信号量 */
#include "app_usart_tx_task.h"   /* 需要 xUsartTxTaskHandle：发送完成(TX_CPLT)中断通知发送任务 */
#include <string.h>
/******************************************************************************
 * 内部硬件宏定义（本工程使用 USART3）
 ******************************************************************************/
/* DMA接收配置：接收/发送均使用CM_DMA2（发送CH0、接收CH1），
   避免与SPI3手指采集DMA(CM_DMA1_CH0/CH1)冲突 */
#define RX_DMA_UNIT                     (CM_DMA2)
#define RX_DMA_CH                       (DMA_CH1)
#define RX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define RX_DMA_TRIG_SEL                 (AOS_DMA2_1)
#define RX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART3_RI)    /* USART3 接收事件 */
#define RX_DMA_RECONF_TRIG_SEL          (AOS_DMA_RC)
#define RX_DMA_RECONF_TRIG_EVT_SRC      (EVT_SRC_AOS_STRG)
/* DMA发送配置 */
#define TX_DMA_UNIT                     (CM_DMA2)
#define TX_DMA_CH                       (DMA_CH0)
#define TX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define TX_DMA_TRIG_SEL                 (AOS_DMA2_0)
#define TX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART3_TI)    /* USART3 发送事件 */
#define TX_DMA_TC_FLAG                  (DMA_FLAG_TC_CH0)
#define TX_DMA_TC_IRQn                  (INT001_IRQn)
#define TX_DMA_TC_INT_SRC               (INT_SRC_DMA2_TC0)

/* 超时定时器配置：USART 接收超时在芯片内固定绑定 TMR0 单元/通道
 * （USART1→TMR0_1/CH_A，USART2→TMR0_1/CH_B，USART3→TMR0_2/CH_A，USART4→TMR0_2/CH_B）。
 * 本串口为 USART3，必须为 TMR0_2/CH_A；迁移到其他 USART 时请同步修改以下三个宏。 */
#define TMR0_UNIT                       (CM_TMR0_2)
#define TMR0_CH                         (TMR0_CH_A)
#define TMR0_FCG_ENABLE()               (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_2, ENABLE))

/* USART3 中断配置 */
#define USART_TX_CPLT_IRQn              (INT002_IRQn)
#define USART_TX_CPLT_INT_SRC           (INT_SRC_USART3_TCI)   /* 发送完成中断源 */
#define USART_RX_TIMEOUT_IRQn           (INT004_IRQn)
#define USART_RX_TIMEOUT_INT_SRC        (INT_SRC_USART3_RTO)   /* 接收超时中断源 */

/******************************************************************************
 * 静态全局变量
 ******************************************************************************/
/* 接收双缓冲：两块 1024B 缓冲交替承接 DMA 接收段，最多缓存 2 个"已收齐未消费"段，
 * 消除"上一段未消费时新段覆盖旧段"导致的丢帧/混合帧 */
#define USART_RX_BUF_NUM        (2U)
static uint8_t  m_au8RxBuf[USART_RX_BUF_NUM][USART_BSP_MAX_FRAME_LEN];
static __IO uint8_t  m_u8RxPendCnt = 0U;  /* 待消费段数(0~2) */
static __IO uint8_t  m_u8RxRdIdx   = 0U;  /* 最旧待消费段所在缓冲槽(GetRxFrame 取此槽) */
static __IO uint8_t  m_u8RxWrIdx   = 0U;  /* 正在接收段所在缓冲槽(下一段写入槽) */
static __IO uint16_t m_au16RxLen[USART_RX_BUF_NUM];  /* 每段实际接收长度 */
static __IO uint8_t  m_u8RxStopped = 0U;  /* 溢出背压标志：双缓冲满已停收，待消费释放后恢复 */
static __IO uint32_t m_u32RxOverrun = 0UL;/* 溢出累计次数（调试可观察） */

#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_DMA)
/* DMA 发送忙标志：一帧 DMA 启动~TX_CPLT 完成期间置 1。
 * 发送队列已上移至应用层(app_usart_tx_task.c 静态 FreeRTOS 队列)；本 BSP 不再维护
 * 内部 TX 队列与中断自动续发：USART_BSP_Send 仅做单帧 DMA 启动，由 TX_CPLT 中断
 * 以任务通知唤醒 app_usart_tx_task 发送任务，其发送引擎串行启动下一帧。 */
static __IO uint8_t  m_u8TxBusy  = 0U;
#endif
static stc_dma_llp_descriptor_t m_stcLlpDesc;

/******************************************************************************
 * 内部静态函数声明
 ******************************************************************************/
static void DMA_Config(void);
static void TMR0_Config(uint16_t u16TimeoutBits);
static void USART_StopTimeoutTimer(CM_TMR0_TypeDef *TMR0x, uint32_t u32Ch);
static void USART_RxResumeAfterStop(void);

/******************************************************************************
 * 中断回调函数
 ******************************************************************************/

#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_DMA)
/**
 * @brief  DMA发送完成中断回调（仅 DMA 模式编译）
 */
static void TX_DMA_TC_IrqCallback(void)
{
    /* 开启发送完成中断，等待移位寄存器空 */
    USART_FuncCmd(USART_BSP_UNIT, USART_INT_TX_CPLT, ENABLE);
    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, TX_DMA_TC_FLAG);
}
#endif

/**
 * @brief  接收超时中断回调（不定长帧核心）
 */
static void USART_RxTimeout_IrqCallback(void)
{
    if (m_u8RxStopped == 0U) {
        if (m_u8RxPendCnt < USART_RX_BUF_NUM) {
            uint16_t u16RxLen;
            uint8_t  u8DoneSlot = m_u8RxWrIdx;

            u16RxLen = USART_BSP_MAX_FRAME_LEN - (uint16_t)DMA_GetTransCount(RX_DMA_UNIT, RX_DMA_CH);
            m_au16RxLen[u8DoneSlot] = u16RxLen;
            m_u8RxPendCnt++;
            m_u8RxWrIdx = (uint8_t)((m_u8RxWrIdx + 1U) % USART_RX_BUF_NUM);

            if (m_u8RxPendCnt < USART_RX_BUF_NUM) {
                /* 仍有空闲槽：切换 DMA 目标到下一块缓冲并重置计数(从缓冲头开始)，
                   绝不把下一段写回未消费段 */
                m_stcLlpDesc.DARx   = (uint32_t)&m_au8RxBuf[m_u8RxWrIdx][0];
                m_stcLlpDesc.DTCTLx = ((uint32_t)USART_BSP_MAX_FRAME_LEN << DMA_DTCTL_CNT_POS) |
                                      (1UL << DMA_DTCTL_BLKSIZE_POS);
                AOS_SW_Trigger();   /* 触发DMA重配置，为下一次接收做准备 */
            } else {
                /* 双缓冲已满(消费端严重落后)：停 DMA 做背压，等待任务取走一段后
                   由 ClearRxFlag 恢复；避免继续接收静默覆盖最旧待消费段 */
                m_u8RxStopped = 1U;
                m_u32RxOverrun++;
                (void)DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, DISABLE);
            }

            /* 调用接收完成回调 */
            USART_BSP_RxCpltCallback(u16RxLen);
        }
    } else {
        /* 背压停收期间到达的字节：DMA 已停，逐次排空 RDR 并清错误标志，
           防止溢出错误滞留影响恢复后的接收；该段数据不完整，直接丢弃 */
        (void)USART_BSP_UNIT->RDR;
        USART_ClearStatus(USART_BSP_UNIT, (USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR));
    }

    USART_StopTimeoutTimer(TMR0_UNIT, TMR0_CH);
    USART_ClearStatus(USART_BSP_UNIT, USART_FLAG_RX_TIMEOUT);
}

#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_DMA)
/**
 * @brief  发送完成中断回调（仅 DMA 模式编译：一帧发送完成收尾并通知发送任务）
 * @note   一帧 DMA 搬运完(TX_DMA_TC)使能本中断，线上发完(TX_CPLT)进入本回调：
 *         停发送/停本中断 -> 清忙标志 -> vTaskNotifyGiveFromISR 唤醒 app_usart_tx_task
 *         发送任务（其发送引擎据此串行启动下一帧；无待发帧时该任务阻塞在发送队列上）。
 *         不再在本 ISR 内自动续发——逐帧发送调度统一由 app_usart_tx_task 任务负责。
 */
static void USART_TxComplete_IrqCallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    USART_FuncCmd(USART_BSP_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    m_u8TxBusy = 0U;

    /* 发送完成任务通知：唤醒阻塞在 ulTaskNotifyTake 的 app_usart_tx_task 发送引擎 */
    if (xUsartTxTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(xUsartTxTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
#endif

/******************************************************************************
 * 内部硬件初始化
 ******************************************************************************/

/**
 * @brief  DMA配置
 */
static void DMA_Config(void)
{
    stc_dma_init_t stcDmaInit;
    stc_dma_llp_init_t stcDmaLlpInit;
#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_DMA)
    stc_irq_signin_config_t stcIrqSignConfig;
#endif

    /* 使能时钟 */
    RX_DMA_FCG_ENABLE();
    TX_DMA_FCG_ENABLE();
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /************************ 接收DMA ************************/
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn = DMA_INT_DISABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCount = USART_BSP_MAX_FRAME_LEN;
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32SrcAddr = (uint32_t)(&USART_BSP_UNIT->RDR);
    stcDmaInit.u32DestAddr = (uint32_t)&m_au8RxBuf[0][0];   /* 初始段写入第 0 块缓冲 */
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
    (void)DMA_Init(RX_DMA_UNIT, RX_DMA_CH, &stcDmaInit);

    /* 链表重配置：接收超时后自动重置DMA */
    (void)DMA_LlpStructInit(&stcDmaLlpInit);
    stcDmaLlpInit.u32State = DMA_LLP_ENABLE;
    stcDmaLlpInit.u32Mode  = DMA_LLP_WAIT;
    stcDmaLlpInit.u32Addr  = (uint32_t)&m_stcLlpDesc;
    (void)DMA_LlpInit(RX_DMA_UNIT, RX_DMA_CH, &stcDmaLlpInit);

    m_stcLlpDesc.SARx   = stcDmaInit.u32SrcAddr;
    m_stcLlpDesc.DARx   = stcDmaInit.u32DestAddr;
    m_stcLlpDesc.DTCTLx = (stcDmaInit.u32TransCount << DMA_DTCTL_CNT_POS) | (stcDmaInit.u32BlockSize << DMA_DTCTL_BLKSIZE_POS);
    m_stcLlpDesc.LLPx   = (uint32_t)&m_stcLlpDesc;
    m_stcLlpDesc.CHCTLx = stcDmaInit.u32SrcAddrInc | stcDmaInit.u32DestAddrInc | stcDmaInit.u32DataWidth |
                          stcDmaInit.u32IntEn      | stcDmaLlpInit.u32State    | stcDmaLlpInit.u32Mode;

    DMA_ReconfigLlpCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);
    /* RX已迁DMA2：关闭本模块早先在DMA1上开启的重配置，防止AOS软触发
       （DMA1/DMA2共用DMA_RC_TRGSEL）误重载SPI正在使用的DMA1_CH0 */
    DMA_ReconfigCmd(RX_DMA_UNIT, ENABLE);
    AOS_SetTriggerEventSrc(RX_DMA_RECONF_TRIG_SEL, RX_DMA_RECONF_TRIG_EVT_SRC);

    AOS_SetTriggerEventSrc(RX_DMA_TRIG_SEL, RX_DMA_TRIG_EVT_SRC);
    DMA_Cmd(RX_DMA_UNIT, ENABLE);
    (void)DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);

#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_DMA)
    /************************ 发送DMA（仅 DMA 模式初始化）************************/
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCount = 0U;
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32SrcAddr = (uint32_t)m_au8RxBuf;
    stcDmaInit.u32DestAddr = (uint32_t)(&USART_BSP_UNIT->TDR);
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    (void)DMA_Init(TX_DMA_UNIT, TX_DMA_CH, &stcDmaInit);

    /* 发送DMA中断注册 */
    stcIrqSignConfig.enIntSrc = TX_DMA_TC_INT_SRC;
    stcIrqSignConfig.enIRQn  = TX_DMA_TC_IRQn;
    stcIrqSignConfig.pfnCallback = &TX_DMA_TC_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_06);  /* 与其它外设中断（SPI-DMA/EXIT）统一为 06 */
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

    AOS_SetTriggerEventSrc(TX_DMA_TRIG_SEL, TX_DMA_TRIG_EVT_SRC);
    DMA_Cmd(TX_DMA_UNIT, ENABLE);
    DMA_TransCompleteIntCmd(TX_DMA_UNIT, TX_DMA_TC_FLAG, ENABLE);
#endif
}
/**
 * @brief  超时定时器配置（USART3 硬件固定绑定 TMR0_2/CH_A，时钟源为内部 LRC）
 * @note   USART 接收超时需要的 TMR0 单元/通道由芯片内部固定绑定，勿随意改动；
 *         若迁移串口请同步修改文件头 TMR0_UNIT/TMR0_CH/TMR0_FCG_ENABLE 三个宏。
 */
static void TMR0_Config(uint16_t u16TimeoutBits)
{
    uint16_t u16Div;
    uint16_t u16Delay;
    uint16_t u16CompareValue;
    stc_tmr0_init_t stcTmr0Init;

    /* 使能TMR0外设时钟 */
    TMR0_FCG_ENABLE();

    /* TMR0 计数时钟源：内部低速 RC(LRC)，不依赖外部晶振 */
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_LRC;
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV8;
    stcTmr0Init.u32Func     = TMR0_FUNC_CMP;

    /* 不同分频下的硬件延迟补偿值 */
    if (TMR0_CLK_DIV1 == stcTmr0Init.u32ClockDiv) {
        u16Delay = 7U;
    } else if (TMR0_CLK_DIV2 == stcTmr0Init.u32ClockDiv) {
        u16Delay = 5U;
    } else if ((TMR0_CLK_DIV4 == stcTmr0Init.u32ClockDiv) ||
               (TMR0_CLK_DIV8 == stcTmr0Init.u32ClockDiv) ||
               (TMR0_CLK_DIV16 == stcTmr0Init.u32ClockDiv)) {
        u16Delay = 3U;
    } else {
        u16Delay = 2U;
    }

    /* 计算分频系数与比较值 */
    u16Div = (uint16_t)1U << (stcTmr0Init.u32ClockDiv >> TMR0_BCONR_CKDIVA_POS);
    u16CompareValue = ((u16TimeoutBits + u16Div - 1U) / u16Div) - u16Delay;
    stcTmr0Init.u16CompareValue = u16CompareValue;
    (void)TMR0_Init(TMR0_UNIT, TMR0_CH, &stcTmr0Init);

    /* 使能硬件启动、硬件清零，配合USART接收超时 */
    TMR0_HWStartCondCmd(TMR0_UNIT, TMR0_CH, ENABLE);
    TMR0_HWClearCondCmd(TMR0_UNIT, TMR0_CH, ENABLE);
}

/**
 * @brief  停止超时定时器
 */
static void USART_StopTimeoutTimer(CM_TMR0_TypeDef *TMR0x, uint32_t u32Ch)
{
    uint32_t u32ClrMask;
    uint32_t u32SetMask;
    uint32_t u32BitOffset;

    u32BitOffset = 16UL * u32Ch;
    u32ClrMask = (TMR0_BCONR_SYNCLKA | TMR0_BCONR_SYNSA) << u32BitOffset;
    u32SetMask = TMR0_BCONR_SYNCLKA << u32BitOffset;
    MODIFY_REG32(TMR0x->BCONR, u32ClrMask, u32SetMask);

    u32ClrMask = (TMR0_BCONR_SYNCLKA | TMR0_BCONR_SYNSA | TMR0_BCONR_CSTA) << u32BitOffset;
    u32SetMask = TMR0_BCONR_SYNSA << u32BitOffset;
    MODIFY_REG32(TMR0x->BCONR, u32ClrMask, u32SetMask);
}

/******************************************************************************
 * 对外接口实现
 ******************************************************************************/

/**
 * @brief  USART BSP初始化
 */
void USART_BSP_Init(void)
{
    stc_usart_uart_init_t stcUartInit;
    stc_irq_signin_config_t stcIrqSigninConfig;

    /* 1. GPIO引脚功能配置 */
    GPIO_SetFunc(USART_BSP_RX_PORT, USART_BSP_RX_PIN, USART_BSP_RX_FUNC);
    GPIO_SetFunc(USART_BSP_TX_PORT, USART_BSP_TX_PIN, USART_BSP_TX_FUNC);

    /* 2. 外设时钟使能：USART3 */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART3, ENABLE);

    /* 3. DMA初始化 */
    DMA_Config();

    /* 4. 超时定时器初始化 */
    TMR0_Config(USART_BSP_TIMEOUT_BITS);

    /* 5. USART串口初始化 */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_ENABLE;
    stcUartInit.u32Baudrate = USART_BSP_BAUDRATE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    (void)USART_UART_Init(USART_BSP_UNIT, &stcUartInit, NULL);

    /* 6. 中断注册 */
#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_DMA)
    /* 发送完成中断（仅 DMA 模式需要：TX_CPLT 通知任务发送完成） */
    stcIrqSigninConfig.enIRQn = USART_TX_CPLT_IRQn;
    stcIrqSigninConfig.enIntSrc = USART_TX_CPLT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_TxComplete_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_06);  /* 与其它外设中断（SPI-DMA/EXIT）统一为 06 */
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);
#endif

    /* 接收超时中断 */
    stcIrqSigninConfig.enIRQn = USART_RX_TIMEOUT_IRQn;
    stcIrqSigninConfig.enIntSrc = USART_RX_TIMEOUT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxTimeout_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_06);  /* 与其它外设中断（SPI-DMA/EXIT）统一为 06 */
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* 7. 开启接收 */
    USART_FuncCmd(USART_BSP_UNIT,
                  (USART_RX | USART_INT_RX | USART_RX_TIMEOUT | USART_INT_RX_TIMEOUT),
                  ENABLE);

    /* 复位接收段状态机：无待消费段，接收 DMA 指向第 0 块缓冲(已在 DMA_Config 配置) */
    m_u8RxPendCnt = 0U;
    m_u8RxRdIdx   = 0U;
    m_u8RxWrIdx   = 0U;
    m_u8RxStopped = 0U;
    for (uint8_t u8Idx = 0U; u8Idx < USART_RX_BUF_NUM; u8Idx++) {
        m_au16RxLen[u8Idx] = 0U;
    }
    m_u32RxOverrun = 0UL;

#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_DMA)
    m_u8TxBusy  = 0U;
#endif
}

#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_POLL)
/**
 * @brief  POLL 模式寄存器轮询直发核心（任务/中断上下文均可调用）
 * @param  pu8Data 发送数据指针
 * @param  u16Len  数据长度（不得超过 USART_BSP_MAX_FRAME_LEN）
 * @retval LL_OK=本帧已完整发送完毕；LL_ERR_INVD_PARAM=参数错误；
 *         LL_ERR_TIMEOUT=发送卡死（内部已复位发送通道）
 * @note   POLL 直发不依赖发送 DMA/完成中断/任务通知；逐字节等 TX_EMPTY、
 *         帧末等 TX_CPLT 忙等，期间不释放 CPU。超时判定用系统 tick 计数差值
 *         （单核下中断上下文读取 tick 计数与任务一致，可安全使用）。
 */
static int32_t Usart_SendPollCore(const uint8_t *pu8Data, uint16_t u16Len)
{
    uint16_t u16Idx;
    uint32_t u32TimeoutMs;
    TickType_t xStartTick;

    if ((pu8Data == NULL) || (u16Len == 0U) || (u16Len > USART_BSP_MAX_FRAME_LEN)) {
        return LL_ERR_INVD_PARAM;
    }

    /* 使能发送，清除残留发送完成标志 */
    USART_FuncCmd(USART_BSP_UNIT, USART_TX, ENABLE);
    USART_ClearStatus(USART_BSP_UNIT, USART_FLAG_TX_CPLT);

    /* 超时预算：按 3Mbps(8N1)=300k 字节/s 折算，本式约有 3.6 倍实时耗时裕量
     * （公式与速率解耦，后续升降速同样安全），2ms 收尾裕量另计 */
    u32TimeoutMs = ((uint32_t)u16Len * 10UL) / 1000UL + 2UL;
    xStartTick = xTaskGetTickCount();

    for (u16Idx = 0U; u16Idx < u16Len; u16Idx++) {
        /* 等发送数据寄存器空（TDR 可写入下一字节） */
        while (SET != USART_GetStatus(USART_BSP_UNIT, USART_FLAG_TX_EMPTY)) {
            if ((uint32_t)(xTaskGetTickCount() - xStartTick) >= pdMS_TO_TICKS(u32TimeoutMs)) {
                USART_BSP_TxAbort();
                return LL_ERR_TIMEOUT;
            }
        }
        USART_WriteData(USART_BSP_UNIT, pu8Data[u16Idx]);
    }

    /* 帧末等发送完成（全部字节已移出移位寄存器、线上发送完毕）后再返回 */
    while (SET != USART_GetStatus(USART_BSP_UNIT, USART_FLAG_TX_CPLT)) {
        if ((uint32_t)(xTaskGetTickCount() - xStartTick) >= pdMS_TO_TICKS(u32TimeoutMs)) {
            USART_BSP_TxAbort();
            return LL_ERR_TIMEOUT;
        }
    }
    return LL_OK;
}
#endif /* USART_BSP_TX_MODE_POLL */

/**
 * @brief  发送一帧数据（底层单帧发送入口，任务上下文调用；仅 app_usart_tx_task 发送引擎使用）
 * @param  pu8Data 发送数据指针
 * @param  u16Len  数据长度（不得超过 USART_BSP_MAX_FRAME_LEN）
 * @retval LL_OK=本帧 DMA 已启动（或 POLL 模式已线上发送完毕）；
 *         LL_ERR_INVD_PARAM=参数错误/长度超限；LL_ERR_BUSY=发送忙（前一帧未发完）
 * @note   DMA 模式（当前默认）：忙检测(m_u8TxBusy + DMA 硬件状态) -> 关发送 DMA/
 *         清残留 TC/错误标志 -> 重配源地址与长度 -> 启动 DMA，返回 LL_OK 即本帧开始
 *         发送。本接口**不等**线上发送完毕：调用方应在启动后阻塞等待 TX_CPLT 中断
 *         发出的"发送完成"任务通知（vTaskNotifyGiveFromISR），并在源缓冲被改写前
 *         确保该帧已发完（逐帧串行，队列/调度由 app_usart_tx_task 发送任务负责）。
 *         返回 LL_ERR_BUSY 时本帧未启动，调用方可直接丢弃本帧继续下一帧。
 *         POLL 模式：降级为寄存器轮询直发 Usart_SendPollCore（返回即发送完毕）。
 */
int32_t USART_BSP_Send(const uint8_t *pu8Data, uint16_t u16Len)
{
#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_DMA)
    uint32_t u32Timeout = 0x1000UL;

    if ((pu8Data == NULL) || (u16Len == 0U) || (u16Len > USART_BSP_MAX_FRAME_LEN)) {
        return LL_ERR_INVD_PARAM;
    }

    /* 忙检测：软件忙标志 + DMA 通道硬件发送状态（防止标志被异常清除后重入） */
    if ((m_u8TxBusy != 0U) ||
        (SET == DMA_GetTransStatus(TX_DMA_UNIT, DMA_STAT_TRANS_CH0))) {
        return LL_ERR_BUSY;
    }

    /* 关闭发送 DMA 通道并等待硬件停止，清残留 TC/错误标志与本帧前的 TX_CPLT 标志，
       防止本次启动被误判为上一帧完成而提前触发 TX_DMA_TC/TX_CPLT 中断 */
    DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    while ((SET == DMA_GetTransStatus(TX_DMA_UNIT, DMA_STAT_TRANS_CH0)) && (u32Timeout-- > 0U)) {
        ;
    }
    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, DMA_FLAG_TC_CH0 | DMA_FLAG_BTC_CH0);
    DMA_ClearErrStatus(TX_DMA_UNIT, DMA_FLAG_TRANS_ERR_CH0 | DMA_FLAG_REQ_ERR_CH0);
    USART_ClearStatus(USART_BSP_UNIT, USART_FLAG_TX_CPLT);

    /* 置忙标志后再重配并启动 DMA，防止 TX_DMA_TC 中断提前触发 */
    m_u8TxBusy = 1U;
    DMA_SetSrcAddr(TX_DMA_UNIT, TX_DMA_CH, (uint32_t)pu8Data);
    DMA_SetTransCount(TX_DMA_UNIT, TX_DMA_CH, u16Len);
    DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, ENABLE);
    USART_FuncCmd(USART_BSP_UNIT, USART_TX, ENABLE);

    return LL_OK;
#else
    /* POLL 模式：寄存器轮询直发核心（返回即线上发送完毕，仅调试用） */
    return Usart_SendPollCore(pu8Data, u16Len);
#endif
}

/**
 * @brief  [兼容保留] 异步发送一帧数据（等价调用 USART_BSP_Send）
 * @note   原内部 TX 队列/自动续发机制已移除，发送队列上移至应用层
 *         app_usart_tx_task.c（静态 FreeRTOS 队列）。本接口保留仅作兼容：
 *         直接转调 USART_BSP_Send，语义（DMA 启动/忙/参数）与其一致。
 *         **新代码请统一走应用层 AppUsartTxTask_Send()（见 app_usart_tx_task.h）**，
 *         由 app_usart_tx_task 发送引擎逐帧调用 USART_BSP_Send 并等待发送完成通知，
 *         保证帧序与源缓冲生命周期受控。
 */
int32_t USART_BSP_SendAsync(const uint8_t *pu8Data, uint16_t u16Len)
{
    return USART_BSP_Send(pu8Data, u16Len);
}

/**
 * @brief  中断上下文轮询直发一帧数据（ISR 版发送接口）
 * @param  pu8Data 发送数据指针
 * @param  u16Len  数据长度（不得超过 USART_BSP_MAX_FRAME_LEN）
 * @retval LL_OK=本帧已完整发送完毕；LL_ERR_INVD_PARAM=参数错误（含非 POLL 模式调用）；
 *         LL_ERR_TIMEOUT=发送卡死（内部已复位发送通道，可能截断本帧）
 * @note   POLL 模式（当前编译宏）：寄存器轮询直发忙等，不依赖 FreeRTOS 任务通知，
 *         可在中断上下文（官方协议解析所在的接收超时 RTO 中断）直接调用；返回时
 *         本帧全部字节已移出发送移位寄存器，源缓冲即可复用。短应答帧约 0.1ms 级，
 *         大数据多包应答会持续占用中断（接收侧由 RX DMA 双缓冲兜底）。
 * @note   DMA 模式：ISR 内无法阻塞等待"发送完成"任务通知，本接口直接返回
 *         LL_ERR_INVD_PARAM；如确需 DMA 模式中断直发，应采用异步 DMA 发送方案。
 */
int32_t USART_BSP_SendFromISR(const uint8_t *pu8Data, uint16_t u16Len)
{
#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_POLL)
    return Usart_SendPollCore(pu8Data, u16Len);
#else
    (void)pu8Data;
    (void)u16Len;
    return LL_ERR_INVD_PARAM;
#endif
}

/**
 * @brief  强制复位发送通道（任务上下文调用；实现按 USART_BSP_TX_MODE 编译）
 * @note   用于发送卡死/启动残留忙等异常收尾，复位后调用方必须丢弃当前帧并
 *         停止引用源缓冲，避免改写正在发送的数据而产生"混合帧"；PC 端可能
 *         收到一帧被截断/损坏的帧，本机不自动补发/重发，由接收方按其自身
 *         机制处理（如自行重新发起查询）。DMA 模式：停发送 DMA + 清 TC/错误
 *         标志与 TX_CPLT 中断、复位忙标志；POLL 模式：无 DMA/中断，仅停发送
 *         并清发送完成标志。
 */
void USART_BSP_TxAbort(void)
{
#if (USART_BSP_TX_MODE == USART_BSP_TX_MODE_DMA)
    uint32_t u32Timeout = 0x1000UL;

    /* 1. 先屏蔽发送相关的两个中断，防止收尾期间挂起的中断回调再次使能 TX_CPLT */
    NVIC_DisableIRQ(TX_DMA_TC_IRQn);
    NVIC_DisableIRQ(USART_TX_CPLT_IRQn);

    /* 2. 关闭发送DMA通道并等待硬件停止 */
    DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    while ((SET == DMA_GetTransStatus(TX_DMA_UNIT, DMA_STAT_TRANS_CH0)) && (u32Timeout-- > 0U)) {
        ;
    }

    /* 3. 清除所有残留发送标志 */
    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, DMA_FLAG_TC_CH0 | DMA_FLAG_BTC_CH0);
    DMA_ClearErrStatus(TX_DMA_UNIT, DMA_FLAG_TRANS_ERR_CH0 | DMA_FLAG_REQ_ERR_CH0);
    USART_ClearStatus(USART_BSP_UNIT, USART_FLAG_TX_CPLT);

    /* 4. 关闭发送使能与发送完成中断，复位忙标志 */
    USART_FuncCmd(USART_BSP_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    m_u8TxBusy = 0U;

    /* 5. 丢弃可能已挂起的中断，再恢复中断（复位后不再自动续发：
          待发帧由 app_usart_tx_task 发送引擎在下一轮统一调度，防止任务通知计数错乱） */
    NVIC_ClearPendingIRQ(TX_DMA_TC_IRQn);
    NVIC_ClearPendingIRQ(USART_TX_CPLT_IRQn);
    NVIC_EnableIRQ(USART_TX_CPLT_IRQn);
    NVIC_EnableIRQ(TX_DMA_TC_IRQn);
#else
    /* POLL 模式：无发送 DMA/完成中断，停发送使能并清发送完成标志即可；
       残留于 TDR/移位寄存器的半帧字节随发送停止被丢弃 */
    USART_FuncCmd(USART_BSP_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    USART_ClearStatus(USART_BSP_UNIT, USART_FLAG_TX_CPLT);
#endif
}
/**
 * @brief  查询是否有已收齐且未被取走的接收段
 * @retval 1=有待消费段，0=无
 */
uint8_t USART_BSP_IsRxFrameDone(void)
{
    return (m_u8RxPendCnt > 0U) ? 1U : 0U;
}

/**
 * @brief  获取最旧的待消费接收段（不改变待处理状态，取走后须调用 ClearRxFlag 释放）
 * @param  pu8Buf     输出缓冲区
 * @param  u16BufLen  缓冲区大小
 * @retval 实际拷贝长度；0=无待消费段或参数错误
 */
uint16_t USART_BSP_GetRxFrame(uint8_t *pu8Buf, uint16_t u16BufLen)
{
    uint16_t u16CopyLen;

    if ((pu8Buf == NULL) || (m_u8RxPendCnt == 0U)) {
        return 0U;
    }

    u16CopyLen = (m_au16RxLen[m_u8RxRdIdx] < u16BufLen) ? m_au16RxLen[m_u8RxRdIdx] : u16BufLen;
    memcpy(pu8Buf, m_au8RxBuf[m_u8RxRdIdx], u16CopyLen);

    return u16CopyLen;
}

/**
 * @brief  溢出停收后的恢复接收（公共恢复序列）
 * @note   由任务版 ClearRxFlag（内部带 NVIC 屏蔽）与中断版 ClearRxFlagIsr
 *         （接收超时中断上下文，不重入）共用；调用方须保证与 RTO ISR 互斥。
 * @note   恢复条件已由调用方判定：已释放出空槽（m_u8RxPendCnt < 缓冲数）。
 *         此时 m_u8RxWrIdx 恰好指向被释放的缓冲。
 */
static void USART_RxResumeAfterStop(void)
{
    (void)USART_BSP_UNIT->RDR;   /* 排空停收期间滞留的字节 */
    USART_ClearStatus(USART_BSP_UNIT, (USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR | USART_FLAG_RX_TIMEOUT));

    /* 描述符与通道寄存器同步更新到被释放的缓冲：寄存器写入保证通道未完成/未进入
       WAIT 时也能从新目标开始；描述符保持最新保证后续 AOS 软触发重载一致 */
    m_stcLlpDesc.DARx   = (uint32_t)&m_au8RxBuf[m_u8RxWrIdx][0];
    m_stcLlpDesc.DTCTLx = ((uint32_t)USART_BSP_MAX_FRAME_LEN << DMA_DTCTL_CNT_POS) |
                          (1UL << DMA_DTCTL_BLKSIZE_POS);
    (void)DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, DISABLE);
    (void)DMA_SetDestAddr(RX_DMA_UNIT, RX_DMA_CH, (uint32_t)&m_au8RxBuf[m_u8RxWrIdx][0]);
    (void)DMA_SetTransCount(RX_DMA_UNIT, RX_DMA_CH, (uint16_t)USART_BSP_MAX_FRAME_LEN);
    (void)DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);
    AOS_SW_Trigger();   /* 若通道因整段打满而停在 LLP WAIT，此触发完成重配置恢复 */
    m_u8RxStopped = 0U;
}

/**
 * @brief  释放最旧的待消费接收段（消费完成调用一次，每段须对应一次调用）
 * @note   若此前因双缓冲满而溢出停收，则本函数在释放出空槽后自动恢复接收：
 *         排空残留字节、清除溢出错误标志、重配 DMA 目标到刚释放的缓冲并重新使能。
 * @note   任务上下文调用；内部临时屏蔽接收超时中断，保证状态机切换与 RTO ISR 互斥。
 */
void USART_BSP_ClearRxFlag(void)
{
    NVIC_DisableIRQ(USART_RX_TIMEOUT_IRQn);

    if (m_u8RxPendCnt > 0U) {
        m_u8RxPendCnt--;
        m_u8RxRdIdx = (uint8_t)((m_u8RxRdIdx + 1U) % USART_RX_BUF_NUM);
    }

    if ((m_u8RxStopped != 0U) && (m_u8RxPendCnt < USART_RX_BUF_NUM)) {
        USART_RxResumeAfterStop();
    }

    NVIC_EnableIRQ(USART_RX_TIMEOUT_IRQn);
}

/**
 * @brief  获取最旧的待消费接收段指针（不拷贝数据，中断上下文专用）
 * @param  ppu8Data  输出：段数据指针（指向接收缓冲内部，释放前有效）
 * @param  pu16Len   输出：该段实际接收长度
 * @retval 1=成功取得一段；0=无待消费段或参数错误
 * @note   仅在 USART_BSP_RxCpltCallback（接收超时中断上下文）内调用：回调返回前
 *         接收状态机不会变化（同级 RTO 中断不重入），可安全直接引用完成段，
 *         避免把最长 1KB 段 memcpy 到 ISR 栈上；解析完须调用 ClearRxFlagIsr 释放。
 */
uint8_t USART_BSP_GetRxFramePtr(const uint8_t **ppu8Data, uint16_t *pu16Len)
{
    if ((ppu8Data == NULL) || (pu16Len == NULL) || (m_u8RxPendCnt == 0U)) {
        return 0U;
    }

    *ppu8Data = &m_au8RxBuf[m_u8RxRdIdx][0];
    *pu16Len  = m_au16RxLen[m_u8RxRdIdx];
    return 1U;
}

/**
 * @brief  释放最旧的待消费接收段（中断上下文专用）
 * @note   与 ClearRxFlag 语义一致，但面向"中断自消费"模型：接收状态机仅由接收
 *         超时中断自身读写（同级不重入），故省略 NVIC 屏蔽，仅在中断上下文调用。
 *         若此前曾因双缓冲满溢出停收，释放出空槽后会自动恢复接收。
 */
void USART_BSP_ClearRxFlagIsr(void)
{
    if (m_u8RxPendCnt > 0U) {
        m_u8RxPendCnt--;
        m_u8RxRdIdx = (uint8_t)((m_u8RxRdIdx + 1U) % USART_RX_BUF_NUM);
    }

    if ((m_u8RxStopped != 0U) && (m_u8RxPendCnt < USART_RX_BUF_NUM)) {
        USART_RxResumeAfterStop();
    }
}

/******************************************************************************
 * 重写 BSP 接收完成回调：运行于 USART 接收超时(RTO)中断上下文。
 * 只释放接收信号量唤醒 usart 任务；不清段、不解析、不取段指针。攒帧/3C 校验/
 * 升级识别/官方协议解析全部上移至 usart 任务上下文执行（对齐 HIT_QT 的
 * RxCpltCallback 仅给信号量的做法），中断占用显著下降。
 ******************************************************************************/
void USART_BSP_RxCpltCallback(uint16_t u16Len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    (void)u16Len;

    if (m_xRxSem != NULL) {
        (void)xSemaphoreGiveFromISR(m_xRxSem, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



