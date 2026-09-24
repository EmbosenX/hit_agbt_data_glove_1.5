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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/*******************************************************************************
 * 硬件引脚与外设定义（完全匹配指定引脚）
 ******************************************************************************/
/* SPI3 外设 */
#define SPI_UNIT                (CM_SPI3)
#define SPI_CLK                 (FCG1_PERIPH_SPI3)
#define SPI_TX_EVT_SRC          (EVT_SRC_SPI3_SPTI)
#define SPI_RX_EVT_SRC          (EVT_SRC_SPI3_SPRI)

/* DMA 外设（沿用DMA1通道0/1） */
#define DMA_UNIT                (CM_DMA1)
#define DMA_CLK                 (FCG0_PERIPH_DMA1 | FCG0_PERIPH_AOS)
#define DMA_TX_CH               (DMA_CH0)
#define DMA_TX_TRIG_CH          (AOS_DMA1_0)
#define DMA_RX_CH               (DMA_CH1)
#define DMA_RX_INT_CH           (DMA_INT_TC_CH1)
#define DMA_RX_TRIG_CH          (AOS_DMA1_1)
#define DMA_RX_INT_SRC          (INT_SRC_DMA1_TC1)
#define DMA_RX_IRQ_NUM          (INT007_IRQn)

/* 2路片选 CS 引脚：1=食指无名指 2=中指小指 */
#define SPI_CS1_PORT            GPIO_PORT_A
#define SPI_CS1_PIN             GPIO_PIN_02      /* (SPI1-NSS) */

#define SPI_CS2_PORT            GPIO_PORT_A
#define SPI_CS2_PIN             GPIO_PIN_03      /* (SPI2-NSS) */

/* SPI3 总线引脚与复用功能号（HC32F460 Func_Grp2 统一编码）
 * SPI_SCK  = GPIO_FUNC_43（Func_Spi3_Sck）
 * SPI_MOSI = GPIO_FUNC_40（Func_Spi3_Mosi）
 * SPI_MISO = GPIO_FUNC_41（Func_Spi3_Miso）
 * 引脚已对照原理图确认：PB14=SCK，PD11=MOSI，PD10=MISO */
#define SPI_SCK_PORT            GPIO_PORT_B
#define SPI_SCK_PIN             GPIO_PIN_14
#define SPI_SCK_FUNC            GPIO_FUNC_43 //SPI3

#define SPI_MOSI_PORT           GPIO_PORT_D
#define SPI_MOSI_PIN            GPIO_PIN_11
#define SPI_MOSI_FUNC           GPIO_FUNC_40 //SPI3

#define SPI_MISO_PORT           GPIO_PORT_D
#define SPI_MISO_PIN            GPIO_PIN_10
#define SPI_MISO_FUNC           GPIO_FUNC_41 //SPI3

/* 传输超时（约5ms@200MHz主频） */
#define SPI_TRANS_TIMEOUT       500000UL
/*******************************************************************************
 * 局部变量
 ******************************************************************************/
static SemaphoreHandle_t g_spi_mutex = NULL;
static __IO en_flag_status_t enRxCompleteFlag = RESET;

static uint8_t m_u8TxBuf[SPI_MAX_FRAME_LEN];
static uint8_t m_u8RxBuf[SPI_MAX_FRAME_LEN];
/*******************************************************************************
 * 局部函数声明
 ******************************************************************************/
static uint16_t CRC16_CCITT(const uint8_t *pData, uint16_t len);

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
 /**
  * @brief  CRC16-CCITT 校验（poly 0x1021、初值 0xFFFF、MSB first）
  * @param  pData  待校验数据首地址
  * @param  len    数据长度
  * @retval 16 位 CRC 值（收发双方算法必须一致）
  */
 static uint16_t CRC16_CCITT(const uint8_t *pData, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    const uint16_t poly = 0x1021U;
    uint16_t i, j;

    for (i = 0U; i < len; i++)
    {
        crc ^= ((uint16_t)pData[i] << 8U);
        for (j = 0U; j < 8U; j++)
        {
            if (crc & 0x8000U) {
                crc = (crc << 1U) ^ poly;
            } else {
                crc = crc << 1U;
            }
        }
    }
    return crc;
}
/**
 * @brief  DMA接收完成中断回调
 */
static void DMA_TransCompleteCallback(void)
{
    enRxCompleteFlag = SET;
    DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_RX_INT_CH);
}

/**
 * @brief  SPI主机硬件初始化
 */
void Fingers_SPI_Config(void)
{
    stc_spi_init_t stcSpiInit;
    stc_dma_init_t stcDmaInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    stc_gpio_init_t stcGpioInit;

    /* 创建总线互斥量，多任务安全 */
    g_spi_mutex = xSemaphoreCreateMutex();
    enRxCompleteFlag = RESET;

    /* GPIO 默认配置 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
    stcGpioInit.u16PinDir = PIN_DIR_OUT;

    /**************** 1. 初始化 2 路 CS 引脚，默认全部拉高 ****************/
    (void)GPIO_Init(SPI_CS1_PORT, SPI_CS1_PIN, &stcGpioInit);
    GPIO_SetPins(SPI_CS1_PORT, SPI_CS1_PIN);
    (void)GPIO_Init(SPI_CS2_PORT, SPI_CS2_PIN, &stcGpioInit);
    GPIO_SetPins(SPI_CS2_PORT, SPI_CS2_PIN);

    /**************** 2. 初始化 SPI 总线引脚复用 ****************/
    GPIO_SetFunc(SPI_SCK_PORT,  SPI_SCK_PIN,  SPI_SCK_FUNC);
    GPIO_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC);
    GPIO_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_FUNC);

    /**************** 3. SPI 外设配置：Mode3、主机、8bit、MSB ****************/
    FCG_Fcg1PeriphClockCmd(SPI_CLK, ENABLE);
    (void)SPI_StructInit(&stcSpiInit);
    stcSpiInit.u32WireMode          = SPI_4_WIRE;
    stcSpiInit.u32TransMode         = SPI_FULL_DUPLEX;
    stcSpiInit.u32MasterSlave       = SPI_MASTER;
    stcSpiInit.u32Parity            = SPI_PARITY_INVD;
    stcSpiInit.u32SpiMode           = SPI_MD_3;        /* 协议约定 Mode3，与从机严格一致 */
    stcSpiInit.u32BaudRatePrescaler = SPI_BR_CLK_DIV8; /* 约25MHz@200MHz，可按需调整 */
    stcSpiInit.u32DataBits          = SPI_DATA_SIZE_8BIT;
    stcSpiInit.u32FirstBit          = SPI_FIRST_MSB;
    stcSpiInit.u32FrameLevel        = SPI_1_FRAME;
    (void)SPI_Init(SPI_UNIT, &stcSpiInit);

    /**************** 4. DMA 配置 ****************/
    FCG_Fcg0PeriphClockCmd(DMA_CLK, ENABLE);
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32BlockSize  = 1UL;
    stcDmaInit.u32TransCount = SPI_MAX_FRAME_LEN;
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;

    /* TX DMA：内存 → SPI_DR */
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    stcDmaInit.u32SrcAddr     = (uint32_t)m_u8TxBuf;
    stcDmaInit.u32DestAddr    = (uint32_t)(&SPI_UNIT->DR);
    (void)DMA_Init(DMA_UNIT, DMA_TX_CH, &stcDmaInit);
    AOS_SetTriggerEventSrc(DMA_TX_TRIG_CH, SPI_TX_EVT_SRC);

    /* RX DMA：SPI_DR → 内存 */
    stcDmaInit.u32IntEn       = DMA_INT_ENABLE;
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
    stcDmaInit.u32SrcAddr     = (uint32_t)(&SPI_UNIT->DR);
    stcDmaInit.u32DestAddr    = (uint32_t)m_u8RxBuf;
    (void)DMA_Init(DMA_UNIT, DMA_RX_CH, &stcDmaInit);
    AOS_SetTriggerEventSrc(DMA_RX_TRIG_CH, SPI_RX_EVT_SRC);

    /* DMA 接收中断注册 */
    stcIrqSignConfig.enIntSrc    = DMA_RX_INT_SRC;
    stcIrqSignConfig.enIRQn      = DMA_RX_IRQ_NUM;
    stcIrqSignConfig.pfnCallback = DMA_TransCompleteCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_06);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

    DMA_Cmd(DMA_UNIT, ENABLE);
    DMA_ChCmd(DMA_UNIT, DMA_TX_CH, DISABLE);
    DMA_ChCmd(DMA_UNIT, DMA_RX_CH, DISABLE);

    memset(m_u8TxBuf, 0x00, sizeof(m_u8TxBuf));
}
/**
 * @brief  底层 DMA 全双工传输（内部调用，调用方须已持有 g_spi_mutex）
 * @param  u8SlaveIdx  从机编号 1/2（决定片选引脚）
 * @param  pu8TxBuf    发送缓冲（容量 >= SPI_MAX_FRAME_LEN）
 * @param  pu8RxBuf    接收缓冲（容量 >= SPI_MAX_FRAME_LEN）
 * @param  u32Len      传输字节数
 * @retval SPI_OK=完成；SPI_ERR_TIMEOUT=DMA 完成轮询超时
 * @note   帧末等总线空闲后再释放片选，避免从机状态机错位；
 *         传输窗口不进临界区（不屏蔽中断、不禁止任务切换），总线互斥由调用方
 *         持有的 g_spi_mutex 保证，DMA 由硬件推进，任务被抢占不影响从机状态机
 */
static int32_t SPI_DoDmaTransmit(uint8_t u8SlaveIdx, const uint8_t *pu8TxBuf,
                                 uint8_t *pu8RxBuf, uint32_t u32Len)
{
    uint32_t u32Timeout;
    uint8_t  u8DmaDone = 0U;
    int32_t  s32Ret;

    if ((u8SlaveIdx < 1U) || (u8SlaveIdx > 2U)) {
        return SPI_ERR_PARAM;
    }

    /* 1. 拉低对应片选，建立时间 ≥1μs（从机1/从机2 共用同一加固路径） */
    switch (u8SlaveIdx)
    {
        case 1U: GPIO_ResetPins(SPI_CS1_PORT, SPI_CS1_PIN); break;
        case 2U: GPIO_ResetPins(SPI_CS2_PORT, SPI_CS2_PIN); break;
        default: return SPI_ERR_PARAM;
    }
		DDL_DelayUS(1);
    /* 2. 重置 DMA，重载地址与长度 */
    DMA_ChCmd(DMA_UNIT, DMA_TX_CH, DISABLE);
    DMA_ChCmd(DMA_UNIT, DMA_RX_CH, DISABLE);
    DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_RX_INT_CH);
    enRxCompleteFlag = RESET;

    DMA_SetSrcAddr(DMA_UNIT, DMA_TX_CH, (uint32_t)pu8TxBuf);
    DMA_SetTransCount(DMA_UNIT, DMA_TX_CH, u32Len);
    DMA_SetDestAddr(DMA_UNIT, DMA_RX_CH, (uint32_t)pu8RxBuf);
    DMA_SetTransCount(DMA_UNIT, DMA_RX_CH, u32Len);

    DMA_ChCmd(DMA_UNIT, DMA_TX_CH, ENABLE);
    DMA_ChCmd(DMA_UNIT, DMA_RX_CH, ENABLE);

    /* 3. 清SPI残留数据，启动传输 */
    if (SET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_RX_BUF_FULL))
    {
        (void)SPI_ReadData(SPI_UNIT);
    }
    SPI_Cmd(SPI_UNIT, ENABLE);
    /* 4. 等待传输完成：不再进入临界区，DMA 完成中断（INT007，优先级 6）可正常响应。
          该 ISR 会置位 enRxCompleteFlag 并清除 TC 状态位，故完成判据取双条件——
          以 ISR 置位的 enRxCompleteFlag 为主，DMA TC 状态位作为 ISR 尚未执行时的兜底；
          轮询次数即超时保护 */
    u32Timeout = SPI_TRANS_TIMEOUT;
    while ((enRxCompleteFlag != SET) &&
           (RESET == DMA_GetTransCompleteStatus(DMA_UNIT, DMA_RX_INT_CH)))
    {
        if (u32Timeout-- == 0U) {
            break;
        }
    }
    if ((enRxCompleteFlag == SET) ||
        (SET == DMA_GetTransCompleteStatus(DMA_UNIT, DMA_RX_INT_CH))) {
        u8DmaDone = 1U;
    }

    /* 5. 统一收尾（成功/超时共用）：先有界等待 SPI 总线空闲，再关断 SPI 并拉高双片选，
          避免在帧未发送完毕时硬截断传输，导致从机状态机错位 */
    u32Timeout = SPI_TRANS_TIMEOUT;
    while ((RESET == SPI_GetStatus(SPI_UNIT, SPI_FLAG_IDLE)) && (u32Timeout-- > 0U))
    {
    }
    SPI_Cmd(SPI_UNIT, DISABLE);
    DMA_ChCmd(DMA_UNIT, DMA_TX_CH, DISABLE);
    DMA_ChCmd(DMA_UNIT, DMA_RX_CH, DISABLE);
    GPIO_SetPins(SPI_CS1_PORT, SPI_CS1_PIN);
    GPIO_SetPins(SPI_CS2_PORT, SPI_CS2_PIN);
    DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_RX_INT_CH);

    /* 6. 片选释放后的总线稳定延时，隔离相邻帧 */
    DDL_DelayUS(1);

    s32Ret = (u8DmaDone != 0U) ? SPI_OK : SPI_ERR_TIMEOUT;
    return s32Ret;
}

/**
 * @brief  SPI 标准事务接口（协议层封装：组请求头+CRC -> 全双工收发 -> 应答校验）
 * @param  u8SlaveIdx       从机编号 1/2
 * @param  u8Cmd            命令码（如 SPI_CMD_READ_DATA）
 * @param  u16CmdParam      命令参数
 * @param  u16RdMaxLen      期望读取的最大数据长度
 * @param  pu8RespPayload   输出：应答数据段首地址（指向内部接收缓冲）
 * @param  pu16RespLen      输出：应答数据段长度
 * @param  pu8Status        输出：从机状态码
 * @retval SPI_OK=成功；SPI_ERR_TIMEOUT=传输超时；其余为帧/CRC 校验类错误码
 * @note   内部以 g_spi_mutex 串行化总线访问（portMAX_DELAY 等待）；
 *         调用上下文：fingers 任务（单实例）
 */
int32_t Fingers_SPI_Transact(uint8_t u8SlaveIdx, uint8_t u8Cmd, uint16_t u16CmdParam,
                             uint16_t u16RdMaxLen, uint8_t *pu8RespPayload,
                             uint16_t *pu16RespLen, uint8_t *pu8Status)
{
    int32_t s32Ret;
    uint16_t u16HeadCrc;
    uint16_t u16DataLen;
    uint16_t u16RespCrc;

    /* 1. 参数校验：pu8RespPayload/pu16RespLen/pu8Status 允许为 NULL（只发不收的命令） */
    if ((u8SlaveIdx < 1U) || (u8SlaveIdx > 2U)) {
        return SPI_ERR_PARAM;
    }
    if (u16RdMaxLen > SPI_MAX_PAYLOAD_LEN) {
        u16RdMaxLen = SPI_MAX_PAYLOAD_LEN;
    }

    /* 2. 组装主机请求帧（10字节，大端序） */
    memset(m_u8TxBuf, 0x00, sizeof(m_u8TxBuf));
    m_u8TxBuf[0] = SPI_REQ_HEADER;
    m_u8TxBuf[1] = u8Cmd;
    m_u8TxBuf[2] = (uint8_t)((u16CmdParam >> 8U) & 0xFFU);
    m_u8TxBuf[3] = (uint8_t)(u16CmdParam & 0xFFU);
    m_u8TxBuf[4] = (uint8_t)((u16RdMaxLen >> 8U) & 0xFFU);
    m_u8TxBuf[5] = (uint8_t)(u16RdMaxLen & 0xFFU);

    u16HeadCrc = CRC16_CCITT(m_u8TxBuf, 6U);
    m_u8TxBuf[6] = (uint8_t)((u16HeadCrc >> 8U) & 0xFFU);
    m_u8TxBuf[7] = (uint8_t)(u16HeadCrc & 0xFFU);
		m_u8TxBuf[8] = (uint8_t)(0X0D & 0xFFU);
		m_u8TxBuf[9] = (uint8_t)(0X0D & 0xFFU);
    /* 3. 总线加锁，执行 DMA 传输 */
    (void)xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
    s32Ret = SPI_DoDmaTransmit(u8SlaveIdx, m_u8TxBuf, m_u8RxBuf, SPI_MAX_FRAME_LEN);
    (void)xSemaphoreGive(g_spi_mutex);

    if (s32Ret != SPI_OK) {
        return s32Ret;
    }

    /* 4. 解析应答帧，校验合法性 */
    if (m_u8RxBuf[0] != SPI_RESP_HEADER) {
        return SPI_ERR_FRAME;
    }

    u16HeadCrc = CRC16_CCITT(m_u8RxBuf, 6U);
    u16RespCrc = ((uint16_t)m_u8RxBuf[6] << 8U) | (uint16_t)m_u8RxBuf[7];
    if (u16HeadCrc != u16RespCrc) {
        return SPI_ERR_FRAME;
    }

    if (pu8Status != NULL) {
        *pu8Status = m_u8RxBuf[1];
    }
    u16DataLen = ((uint16_t)m_u8RxBuf[4] << 8U) | (uint16_t)m_u8RxBuf[5];

    if (u16DataLen > SPI_MAX_PAYLOAD_LEN) {
        return SPI_ERR_FRAME;
    }

    /* 5. 拷贝载荷数据（pu8RespPayload 为 NULL 时只回报长度，供心跳/复位类命令使用） */
    if ((u16DataLen > 0U) && (pu8RespPayload != NULL)) {
        memcpy(pu8RespPayload, &m_u8RxBuf[SPI_RESP_HEAD_LEN], u16DataLen);
    }
    if (pu16RespLen != NULL) {
        *pu16RespLen = u16DataLen;
    }

    return SPI_OK;
}
/*******************************************************************************
 * 测试接口实现
 ******************************************************************************/


/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
