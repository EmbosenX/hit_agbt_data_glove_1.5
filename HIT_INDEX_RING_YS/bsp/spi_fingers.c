/**
 *******************************************************************************
 * @file  spi_fingers.c
 * @brief This midware file provides firmware functions to fingers group spi.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2023-12-15       CDT             Add null pointer check
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "spi_fingers.h"
#include <stdint.h>
#include "hc32_ll_interrupts.h"
#include "app_finger_task.h"


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* SPI外设定义 */
#define SPI_SLAVE_UNIT        CM_SPI1
#define SPI_SLAVE_CLK         FCG1_PERIPH_SPI1

/* SPI总线引脚 */
#define SPI_SCK_PORT          GPIO_PORT_A
#define SPI_SCK_PIN           GPIO_PIN_10
#define SPI_SCK_FUNC          GPIO_FUNC_43

#define SPI_MOSI_PORT         GPIO_PORT_A
#define SPI_MOSI_PIN          GPIO_PIN_11    
#define SPI_MOSI_FUNC         GPIO_FUNC_40

#define SPI_MISO_PORT         GPIO_PORT_A
#define SPI_MISO_PIN          GPIO_PIN_12   
#define SPI_MISO_FUNC         GPIO_FUNC_41

/* ====================== CS片选输入 PA9 外部中断定义 ====================== */
#define SPI_CS_PORT           GPIO_PORT_A
#define SPI_CS_PIN            GPIO_PIN_09
#define CS_EXTINT_CH          EXTINT_CH09    /* PA9对应外部中断通道9 */
#define CS_INT_SRC            INT_SRC_PORT_EIRQ9  /* 外部中断9对应的中断源 */
#define CS_IRQn               INT008_IRQn    /* 映射到IRQ8 */
/* ====================================================================== */
/* DMA定义 */
#define DMA_UNIT              CM_DMA1
#define DMA_CLK               (FCG0_PERIPH_DMA1 | FCG0_PERIPH_AOS)
#define DMA_TX_CH             DMA_CH0
#define DMA_TX_TRIG_CH        AOS_DMA1_0
#define SPI_TX_EVT_SRC        EVT_SRC_SPI1_SPTI

#define DMA_RX_CH             DMA_CH1
#define DMA_RX_INT_CH         DMA_INT_TC_CH1
#define DMA_RX_TRIG_CH        AOS_DMA1_1
#define SPI_RX_EVT_SRC        EVT_SRC_SPI1_SPRI
#define DMA_RX_INT_SRC        INT_SRC_DMA1_TC1
#define DMA_RX_IRQ_NUM        INT007_IRQn


/* 收发缓冲区：双缓冲乒乓，CS中断与任务分别操作互不重叠的缓冲片 */
#define SPI_BUF_NUM             2U
static uint8_t m_u8TxBuf[SPI_BUF_NUM][SPI_MAX_FRAME_LEN];
static uint8_t m_u8RxBuf[SPI_BUF_NUM][SPI_MAX_FRAME_LEN];

/* 标志位与信号量 */
static __IO uint8_t m_u8RxFinishFlag = 0;

/* 乒乓状态（仅在CS中断[优先级5]与SpiCommTask之间共享，必须__IO保证访存顺序） */
static __IO uint8_t  m_u8TxActiveIdx   = 0U;   /* 本帧DMA读取的TX缓冲片 */
static __IO uint8_t  m_u8TxReadyFlag   = 0U;   /* 备用TX缓冲片已整帧就绪 */
static __IO uint8_t  m_u8RxActiveIdx   = 0U;   /* 本帧DMA写入的RX缓冲片（每帧交替） */
static __IO uint8_t  m_u8RxFilledIdx   = 0U;   /* 已填充、待任务取走的RX缓冲片 */
static __IO uint16_t m_u16RxLen        = 0U;   /* 上升沿快照的本帧实际接收字节数 */
static __IO uint8_t  m_u8RxOverrunFlag = 0U;   /* 发生过丢帧（旧帧未取走即来新帧），仅作状态感知 */

/**
 * @brief  DMA接收完成中断回调
 */
static void SPI_DmaRxCpltCallback(void)
{

    DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_RX_INT_CH);

}

/**
 * @brief  CS引脚外部中断回调（双边沿触发）
 */
static void CS_ExtIntCallback(void)
{
    /* 必须先清中断标志，否则会重复进入中断 */
    EXTINT_ClearExtIntStatus(CS_EXTINT_CH);

    /* 读取CS引脚电平：低电平=下降沿（主机选中），高电平=上升沿（主机释放） */
    if (PIN_RESET == GPIO_ReadInputPins(SPI_CS_PORT, SPI_CS_PIN))
    {
        /* CS下降沿：新一帧开始，切换乒乓缓冲并重装DMA */
        /* 1. 上一帧数据尚未被任务取走：该旧帧视为被顶替丢失，置丢帧标记供任务感知 */
        if (m_u8RxFinishFlag != 0U)
        {
            m_u8RxOverrunFlag = 1U;
        }
        /* 2. RX切到另一片缓冲，保证不会覆盖任务可能仍在读取的那一片 */
        m_u8RxActiveIdx = (uint8_t)(m_u8RxActiveIdx ^ 1U);
        /* 3. TX仅在备用片整帧就绪时才切换，绝不发送半成品 */
        if (m_u8TxReadyFlag != 0U)
        {
            m_u8TxActiveIdx = (uint8_t)(m_u8TxActiveIdx ^ 1U);
            m_u8TxReadyFlag = 0U;
        }

        DMA_ChCmd(DMA_UNIT, DMA_TX_CH, DISABLE);
        DMA_ChCmd(DMA_UNIT, DMA_RX_CH, DISABLE);
        DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_RX_INT_CH);

        /* 重装DMA地址与最大长度（源/目的指向本帧选定的乒乓缓冲片） */
        DMA_SetSrcAddr(DMA_UNIT, DMA_TX_CH, (uint32_t)m_u8TxBuf[m_u8TxActiveIdx]);
        DMA_SetTransCount(DMA_UNIT, DMA_TX_CH, SPI_MAX_FRAME_LEN);
        DMA_SetDestAddr(DMA_UNIT, DMA_RX_CH, (uint32_t)m_u8RxBuf[m_u8RxActiveIdx]);
        DMA_SetTransCount(DMA_UNIT, DMA_RX_CH, SPI_MAX_FRAME_LEN);

        DMA_ChCmd(DMA_UNIT, DMA_TX_CH, ENABLE);
        DMA_ChCmd(DMA_UNIT, DMA_RX_CH, ENABLE);
        /* 必须先清除SPI错误标志，再使能SPI。
           主机若在上一帧传输中途提前释放CS（异常帧/通信被打断），SPI硬件会置位MD_FAULT。
           上升沿分支只做DISABLE（该断言在DISABLE时不生效），标志会一直挂着；
           下一帧再执行SPI_Cmd(ENABLE)时，hc32_ll_spi.c 的
           DDL_ASSERT(IS_SPI_CMD_ALLOWED(enNewState)) 会因MD_FAULT仍置位而失败，
           进入DDL_AssertHandler死循环。这里连同OVERRUN/PARITY/UNDERRUN一并清除，
           使从机在异常帧后能自行恢复。 */
        SPI_ClearStatus(SPI_SLAVE_UNIT, SPI_FLAG_CLR_ALL);
        SPI_Cmd(SPI_SLAVE_UNIT, ENABLE);
    }
    else
    {
        /* CS上升沿：主机释放，先快照本帧实际接收长度，再停机 */
        uint16_t u16Remain = DMA_GetTransCount(DMA_UNIT, DMA_RX_CH);
        m_u16RxLen      = (uint16_t)(SPI_MAX_FRAME_LEN - u16Remain);
        m_u8RxFilledIdx = m_u8RxActiveIdx;

        SPI_Cmd(SPI_SLAVE_UNIT, DISABLE);
        DMA_ChCmd(DMA_UNIT, DMA_TX_CH, DISABLE);
        DMA_ChCmd(DMA_UNIT, DMA_RX_CH, DISABLE);
        m_u8RxFinishFlag = 1u;
        /* 用带判空的包装函数：初始化窗口内 m_xRxSem 可能仍为 NULL，直接给裸句柄会命中
           configASSERT 死循环导致从机永久僵死（加固点 R1） */
        AppFingerTask_NotifyRxFromISR();
    }
}

/**
 * @brief  从机SPI+DMA初始化
 */
void Slave_SPI_Init(void)
{
    stc_spi_init_t stcSpiInit;
    stc_dma_init_t stcDmaInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    stc_gpio_init_t stcGpioInit;
    stc_extint_init_t stcExtIntInit;

    /* 乒乓状态复位：保证上电首帧读到确定状态 */
    m_u8RxFinishFlag  = 0u;
    m_u8TxActiveIdx   = 0U;
    m_u8TxReadyFlag   = 0U;
    m_u8RxActiveIdx   = 0U;
    m_u8RxFilledIdx   = 0U;
    m_u16RxLen        = 0U;
    m_u8RxOverrunFlag = 0U;

    /* ========== 1. CS引脚GPIO配置 ========== */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_IN;
    stcGpioInit.u16PullUp = PIN_PU_ON;          /* 开启上拉 */
    stcGpioInit.u16ExtInt = PIN_EXTINT_ON;      /* 开启引脚外部中断功能 */
    (void)GPIO_Init(SPI_CS_PORT, SPI_CS_PIN, &stcGpioInit);

    /* ========== 2. 外部中断通道配置 ========== */
    (void)EXTINT_StructInit(&stcExtIntInit);
    stcExtIntInit.u32Edge   = EXTINT_TRIG_BOTH; /* 双边沿触发 */
    stcExtIntInit.u32Filter = EXTINT_FILTER_OFF;/* 关闭滤波，保证响应速度 */
    EXTINT_Init(CS_EXTINT_CH, &stcExtIntInit);

    /* 使能引脚外部中断 */
    GPIO_ExtIntCmd(SPI_CS_PORT, SPI_CS_PIN, ENABLE);

    /* 注册CS中断回调 */
    stcIrqSignConfig.enIntSrc    = CS_INT_SRC;
    stcIrqSignConfig.enIRQn      = CS_IRQn;
    stcIrqSignConfig.pfnCallback = CS_ExtIntCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(CS_IRQn);
    /* CS 片选取 FreeRTOS 系统调用边界内最高抢占级 DDL_IRQ_PRIO_05：
       FreeRTOSConfig.h 中 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5，
       优先级 0~4 的 ISR 一旦调用 ...FromISR 会命中 vPortValidateInterruptPriority
       -> configASSERT 死循环（从机永久僵死）；取 5 既拿到可安全调用 FromISR 的最高
       抢占级，又保证本 ISR 内 AppFingerTask_NotifyRxFromISR() 可用 */
    NVIC_SetPriority(CS_IRQn, DDL_IRQ_PRIO_05);
    NVIC_EnableIRQ(CS_IRQn);

    /* ========== 3. SPI总线引脚配置 ========== */
    GPIO_SetFunc(SPI_SCK_PORT,  SPI_SCK_PIN,  SPI_SCK_FUNC);
    GPIO_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC);
    GPIO_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_FUNC);

    /* ========== 4. SPI外设配置 ========== */
    FCG_Fcg1PeriphClockCmd(SPI_SLAVE_CLK, ENABLE);
    (void)SPI_StructInit(&stcSpiInit);
    stcSpiInit.u32WireMode          = SPI_4_WIRE;
    stcSpiInit.u32TransMode         = SPI_FULL_DUPLEX;
    stcSpiInit.u32MasterSlave       = SPI_SLAVE;       /* 从机模式 */
    stcSpiInit.u32Parity            = SPI_PARITY_INVD;
    stcSpiInit.u32SpiMode           = SPI_MD_3;        /* Mode3，与主机严格一致 */
    stcSpiInit.u32DataBits          = SPI_DATA_SIZE_8BIT;
    stcSpiInit.u32FirstBit          = SPI_FIRST_MSB;
    stcSpiInit.u32FrameLevel        = SPI_1_FRAME;
    (void)SPI_Init(SPI_SLAVE_UNIT, &stcSpiInit);

    /* 初始关闭SPI，由CS下降沿唤醒 */
    SPI_Cmd(SPI_SLAVE_UNIT, DISABLE);

    /* ========== 5. DMA配置 ========== */
    FCG_Fcg0PeriphClockCmd(DMA_CLK, ENABLE);
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32BlockSize  = 1UL;
    stcDmaInit.u32TransCount = SPI_MAX_FRAME_LEN;
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;

    /* TX DMA：内存 -> SPI_DR */
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    stcDmaInit.u32SrcAddr     = (uint32_t)m_u8TxBuf[0];
    stcDmaInit.u32DestAddr    = (uint32_t)(&SPI_SLAVE_UNIT->DR);
    (void)DMA_Init(DMA_UNIT, DMA_TX_CH, &stcDmaInit);
    AOS_SetTriggerEventSrc(DMA_TX_TRIG_CH, SPI_TX_EVT_SRC);

    /* RX DMA：SPI_DR -> 内存 */
    stcDmaInit.u32IntEn       = DMA_INT_ENABLE;
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
    stcDmaInit.u32SrcAddr     = (uint32_t)(&SPI_SLAVE_UNIT->DR);
    stcDmaInit.u32DestAddr    = (uint32_t)m_u8RxBuf[0];
    (void)DMA_Init(DMA_UNIT, DMA_RX_CH, &stcDmaInit);
    AOS_SetTriggerEventSrc(DMA_RX_TRIG_CH, SPI_RX_EVT_SRC);

    /* DMA接收中断注册 */
    stcIrqSignConfig.enIntSrc    = DMA_RX_INT_SRC;
    stcIrqSignConfig.enIRQn      = DMA_RX_IRQ_NUM;
    stcIrqSignConfig.pfnCallback = SPI_DmaRxCpltCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(DMA_RX_IRQ_NUM);
    /* DMA接收完成回调仅清传输完成标志，不调用任何 FreeRTOS API，故不受系统调用
       边界(5)限制，可安全提到 DDL_IRQ_PRIO_01，保证 SPI 通道不被低优先级中断拖慢 */
    NVIC_SetPriority(DMA_RX_IRQ_NUM, DDL_IRQ_PRIO_01);
    NVIC_EnableIRQ(DMA_RX_IRQ_NUM);

    DMA_Cmd(DMA_UNIT, ENABLE);

    /* 初始化收发缓冲区默认填充0x00（两片乒乓缓冲全部清零） */
    memset(m_u8TxBuf, 0x00, sizeof(m_u8TxBuf));
    memset(m_u8RxBuf, 0x00, sizeof(m_u8RxBuf));
}

/**
 * @brief  预加载应答数据到TX缓冲区（下一次主机读取时自动发出）
 */
int32_t Slave_SPI_LoadTxData(const uint8_t *pu8Data, uint16_t u16Len)
{
    uint8_t u8PrepareIdx;

    if ((pu8Data == NULL) || (u16Len > SPI_MAX_FRAME_LEN))
    {
        return -1;
    }

    /* 只写"非活动"的备用缓冲片，DMA正在读取的那一片绝不触碰，故无需关TX DMA */
    u8PrepareIdx = (uint8_t)(m_u8TxActiveIdx ^ 1U);
    memcpy(m_u8TxBuf[u8PrepareIdx], pu8Data, u16Len);
    /* 剩余位置补0x00 */
    if (u16Len < SPI_MAX_FRAME_LEN)
    {
        memset(&m_u8TxBuf[u8PrepareIdx][u16Len], 0x00, SPI_MAX_FRAME_LEN - u16Len);
    }

    /* 整帧写入完成后才置就绪标志：CS下降沿中断只会切到完整帧，绝不发半成品 */
    m_u8TxReadyFlag = 1U;

    return 0;
}

/**
 * @brief  获取本次接收到的主机命令数据
 */
int32_t Slave_SPI_GetRxData(uint8_t *pu8RxBuf, uint16_t *pu16Len)
{
    uint8_t  u8FilledIdx;
    uint16_t u16Len;

    if ((pu8RxBuf == NULL) || (pu16Len == NULL))
    {
        return -1;
    }

    if (m_u8RxFinishFlag == 0u)
    {
        return -1;
    }

    /* 先取索引与长度快照：此后即使新帧到达也只写另一片缓冲，本次拷贝不会被改写 */
    u8FilledIdx = m_u8RxFilledIdx;
    u16Len      = m_u16RxLen;
    if (u16Len > SPI_MAX_FRAME_LEN)
    {
        u16Len = SPI_MAX_FRAME_LEN;
    }

    /* 取走本次数据：被顶替的是更旧的一帧，这一帧位于独立缓冲片内且已完整，
       故照常返回最新数据；丢帧标记一并清除 */
    memcpy(pu8RxBuf, m_u8RxBuf[u8FilledIdx], u16Len);
    *pu16Len = u16Len;

    m_u8RxFinishFlag  = 0u;
    m_u8RxOverrunFlag = 0u;
    return 0;
}

/**
 * @brief  查询是否有新接收数据
 */
uint8_t Slave_SPI_IsRxComplete(void)
{
    return m_u8RxFinishFlag;
}

/**
 * @brief  清除接收完成标志
 */
void Slave_SPI_ClearRxFlag(void)
{
    m_u8RxFinishFlag  = 0u;
    m_u8RxOverrunFlag = 0u;
}
/**
 * @brief  CRC16-CCITT 算法实现
 */
uint16_t CRC16_CCITT(const uint8_t *pData, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    const uint16_t poly = 0x1021;
    uint16_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= ((uint16_t)pData[i] << 8);
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x8000U)
            {
                crc = (crc << 1) ^ poly;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }
    return crc;
}
/**
 * @brief  处理主机SPI命令，生成应答Payload
 * @param  pstcReq  主机请求帧指针
 * @param  pu8Data  输出：应答Payload缓冲区
 * @param  pu16Len  输出：Payload有效字节长度
 * @retval None
 * @note   仅生成Payload，应答头+CRC由外层统一组装
 */
void App_ProcessSpiCmd(spi_req_t *pstcReq, uint8_t *pu8Data, uint16_t *pu16Len)
{
    /* 1. 参数合法性校验 */
    if ((pstcReq == NULL) || (pu8Data == NULL) || (pu16Len == NULL))
    {
        *pu16Len = 0U;
        return;
    }
    /* 默认长度为0 */
    *pu16Len = 0U;
    /* 2. 按命令码分支处理 */
    switch (pstcReq->cmd)
    {
        /************************ 心跳探测 ************************/
        case SPI_CMD_NULL:break;
        /************************ 读取ADC数据【核心业务】 ************************/
        case SPI_CMD_READ_TOUCH_DATA:
        {
            static uint16_t u16MaxRead;
            static uint16_t u16Format;
            uint16_t u16ActualLen;

            /* 取出主机要求的最大读取长度，强制限制在阈值以内 */
            u16MaxRead = pstcReq->rd_max_len;
            if (u16MaxRead > SPI_MAX_PAYLOAD_LEN)
            {
                u16MaxRead = SPI_MAX_PAYLOAD_LEN;
            }
						/* 取出数据格式选择 */
						u16Format = pstcReq->cmd_param & ADC_FMT_MASK;
						if(!u16Format)
						{
							DDL_Printf("u16Format==0\r\n");
						}
						/* 实时采集一整帧数据并输出 */
						u16ActualLen = Adc_ReadFrame(pu8Data);

						*pu16Len = u16ActualLen;
        }
        break;
        /************************ 写入标定参数 ************************/
        case SPI_CMD_CALIB_WRITE:break;
        /************************ 读取标定参数 ************************/
        case SPI_CMD_CALIB_READ:break;
        /************************ 保存参数到Flash ************************/
        case SPI_CMD_SAVE_PARAM:break;
        /************************ 未知命令 ************************/
        default:break;
    }
}
/**
 * @brief  freertos task spi+dma data process
 */
void spi_dma_rx_data_process(void)
{
    uint16_t u16RxLen;
    uint16_t u16TxLen = 0U;
    uint16_t u16HeadCrc;
    spi_req_t *pstcReq;
    static uint8_t u8RxData[SPI_MAX_FRAME_LEN];
    static uint8_t u8TxData[SPI_MAX_FRAME_LEN];

    if (Slave_SPI_GetRxData(u8RxData, &u16RxLen) != 0)
    {
        return;
    }

    if (u16RxLen >= sizeof(spi_req_t))
    {
        pstcReq = (spi_req_t *)u8RxData;
        pstcReq->head_crc = ((pstcReq->head_crc >> 8) | (pstcReq->head_crc << 8)) & 0xFFFFU;

        if ((pstcReq->header == SPI_REQ_HEADER) && (CRC16_CCITT(u8RxData, 6U) == pstcReq->head_crc))
        {
            /* 1. 组应答头（10B）：帧头 + 状态 + 载荷长度 + 头部CRC + 尾部 */
            memset(u8TxData, 0x00, sizeof(u8TxData));
            u8TxData[0] = SPI_RESP_HEADER;
            u8TxData[1] = SPI_STAT_OK;

            /* 2. 按命令执行业务，载荷直接写入应答帧 u8TxData[10..] */
            App_ProcessSpiCmd(pstcReq, &u8TxData[SPI_RESP_HEAD_LEN], &u16TxLen);

            u8TxData[4] = (uint8_t)((u16TxLen >> 8U) & 0xFFU);
            u8TxData[5] = (uint8_t)(u16TxLen & 0xFFU);
            u16HeadCrc  = CRC16_CCITT(u8TxData, 6U);
            u8TxData[6] = (uint8_t)((u16HeadCrc >> 8U) & 0xFFU);
            u8TxData[7] = (uint8_t)(u16HeadCrc & 0xFFU);
            u8TxData[8] = 0x0DU;
            u8TxData[9] = 0x0DU;

            /* 3. 预装到 TX 备用缓冲，由主控下一拍事务取走 */
            (void)Slave_SPI_LoadTxData(u8TxData, (uint16_t)(SPI_RESP_HEAD_LEN + u16TxLen));
        }
    }
}


/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
