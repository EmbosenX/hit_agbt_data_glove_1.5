/**
 *******************************************************************************
 * @file  iap/iap_boot/source/com.c
 * @brief This file provides firmware functions to manage the Communication
 *        Device driver.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
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
#include "com.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* RX error flags that must be cleared continuously while polling. If they are
 * left set (e.g. an overrun caused by the host bursting at 2 Mbps), the
 * receiver may never see a clean RXNE and silently stalls. */
#define COM_RX_ERR_FLAGS                (USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR | USART_FLAG_PARITY_ERR)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
/* Number of UART RX error events (overrun / framing / parity) observed since
 * reset. Exposed through COM_GetRxErrCnt() so a failed transfer can be told
 * apart from a protocol problem. */
static uint32_t m_u32RxErrCnt = 0UL;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @brief  Clear pending USART RX error flags (overrun/framing/parity) and
 *         account for them.
 * @param  None
 * @retval None
 */
static void COM_RxClearError(void)
{
    /* IMPORTANT: USART_GetStatus() answers SET/RESET (1/0) for "any of these
     * flags is pending", it is NOT a bit mask of the requested flags. Reading
     * the status register directly (same bit layout as the USART_FLAG_xxx
     * macros) is the only way to know *which* flag is pending, and the whole
     * error mask can then be cleared in one call. */
    uint32_t u32Flag = READ_REG32(MODEM_USART_UNIT->SR) & (uint32_t)COM_RX_ERR_FLAGS;

    if (0UL != u32Flag) {
        m_u32RxErrCnt++;
        USART_ClearStatus(MODEM_USART_UNIT, u32Flag);
    }
}

/**
 * @brief  Wait for one received byte with a real millisecond timeout.
 * @param  [out] pu8Data                 Pointer to the received byte
 * @param  [in]  u32StartTick            SysTick value captured when the whole
 *                                       receive operation started
 * @param  [in]  u32TimeoutMs            Total allowed elapsed time(ms)
 * @retval int32_t:
 *           - LL_OK: Byte received
 *           - LL_ERR_TIMEOUT: No byte within the deadline
 * @note   HC32F460 USART has no RX FIFO, so the register is polled byte by
 *         byte while clearing overrun/framing errors on the fly. The status is
 *         read once per iteration for both decisions (byte available and error
 *         pending) to keep this per-byte hot path as short as possible, which
 *         is what leaves headroom at higher baud rates.
 */
static int32_t COM_WaitByte(uint8_t *pu8Data, uint32_t u32StartTick, uint32_t u32TimeoutMs)
{
    uint32_t u32Sr;

    do {
        /* One status-register read per iteration serves both decisions. It has
         * to be a raw register read: USART_GetStatus() only reports SET/RESET
         * for "any of the requested flags", so it cannot tell RXNE from the
         * error flags. Using it here made the error branch fire on every byte
         * (1 & COM_RX_ERR_FLAGS != 0) and the byte branch never fire
         * (1 & USART_FLAG_RX_FULL == 0), i.e. no byte could ever be returned. */
        u32Sr = READ_REG32(MODEM_USART_UNIT->SR);
        if (0UL != (u32Sr & (uint32_t)COM_RX_ERR_FLAGS)) {
            m_u32RxErrCnt++;
            USART_ClearStatus(MODEM_USART_UNIT, COM_RX_ERR_FLAGS);
        }
        if (0UL != (u32Sr & (uint32_t)USART_FLAG_RX_FULL)) {
            *pu8Data = (uint8_t)(USART_ReadData(MODEM_USART_UNIT) & 0xFFU);
            return LL_OK;
        }
    } while ((SysTick_GetTick() - u32StartTick) < u32TimeoutMs);

    return LL_ERR_TIMEOUT;
}

/**
 * @brief  COM De-Initialize.
 * @param  None
 * @retval None
 */
void COM_DeInit(void)
{
    USART_DeInit(MODEM_USART_UNIT);
    /* Disable USART clock */
    MODEM_USART_CLK_CTRL(DISABLE);
    /* Configure USART RX/TX pin */
    GPIO_SetFunc(MODEM_USART_RX_PORT, MODEM_USART_RX_PIN, GPIO_FUNC_0);
    GPIO_SetFunc(MODEM_USART_TX_PORT, MODEM_USART_TX_PIN, GPIO_FUNC_0);
}


/**
 * @brief  COM Initialize.
 * @param  None
 * @retval None
 */
void COM_Init(void)
{
    stc_usart_uart_init_t stcUartInit;

    /* Configure USART RX/TX pin */
    GPIO_SetFunc(MODEM_USART_RX_PORT, MODEM_USART_RX_PIN, MODEM_USART_RX_FUNC);
    GPIO_SetFunc(MODEM_USART_TX_PORT, MODEM_USART_TX_PIN, MODEM_USART_TX_FUNC);
    /* Enable USART Clock. */
    MODEM_USART_CLK_CTRL(ENABLE);
    /* Initialize UART */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv      = USART_CLK_DIV1;
    stcUartInit.u32CKOutput      = USART_CK_OUTPUT_ENABLE;
    stcUartInit.u32Baudrate      = MODEM_USART_BAUD_RATE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    (void)USART_UART_Init(MODEM_USART_UNIT, &stcUartInit, NULL);
    /* Enable RX/TX function */
    USART_FuncCmd(MODEM_USART_UNIT, (USART_RX | USART_TX), ENABLE);
}

/**
 * @brief  COM send data.
 * @param  [in] pu8Buff                 Pointer to the buffer to be sent
 * @param  [in] u16Len                  Send buffer length
 * @retval None
 */
void COM_SendData(uint8_t *pu8Buff, uint16_t u16Len)
{
    USART_UART_Trans(MODEM_USART_UNIT, pu8Buff, u16Len, USART_MAX_TIMEOUT);
}

/**
 * @brief  COM receive data.
 * @param  [out] pu8Buff                Pointer to the buffer to be sent
 * @param  [in]  u16Len                 Receive data length
 * @param  [in]  u32Timeout             Total receive timeout(ms), SysTick based
 * @retval int32_t:
 *           - LL_OK: Receive data finished
 *           - LL_ERR: Receive error
 *           - LL_ERR_INVD_PARAM: u32Len value is 0 or the pointer pvBuf value is NULL.
 * @note   The timeout covers the whole buffer and is measured in real
 *         milliseconds with SysTick. Previously it was forwarded as
 *         "u32Timeout * 1000" to USART_UART_Receive(), whose 4th argument is a
 *         *poll-loop iteration count* (see USART_WaitStatus()), not a time
 *         value. On a 200 MHz core that made the effective timeout about 16x
 *         shorter than intended, so the Y-modem handshake re-sent 'C' every
 *         few tens of milliseconds instead of the standard ~1 s and the
 *         per-block wait window was far too small.
 */
int32_t COM_RecvData(uint8_t *pu8Buff, uint16_t u16Len, uint32_t u32Timeout)
{
    uint32_t u32StartTick;
    uint16_t i;

    if ((NULL == pu8Buff) || (0U == u16Len)) {
        return LL_ERR_INVD_PARAM;
    }

    /* Clear any pending RX error before the transfer starts */
    COM_RxClearError();

    u32StartTick = SysTick_GetTick();
    for (i = 0U; i < u16Len; i++) {
        if (LL_OK != COM_WaitByte(&pu8Buff[i], u32StartTick, u32Timeout)) {
            return LL_ERR;
        }
    }

    return LL_OK;
}

/**
 * @brief  Discard every byte already pending in the USART RX register.
 * @param  None
 * @retval None
 * @note   Used to resynchronise before starting a Y-modem session so that
 *         leftovers from a previous aborted attempt cannot desync the host.
 */
void COM_FlushRx(void)
{
    uint32_t u32Guard = 0UL;

    COM_RxClearError();
    while ((SET == USART_GetStatus(MODEM_USART_UNIT, USART_FLAG_RX_FULL)) && (u32Guard < 4096UL)) {
        (void)USART_ReadData(MODEM_USART_UNIT);
        u32Guard++;
    }
}

/**
 * @brief  Get the accumulated count of UART RX error events.
 * @param  None
 * @retval uint32_t: Number of overrun/framing/parity events since reset
 */
uint32_t COM_GetRxErrCnt(void)
{
    return m_u32RxErrCnt;
}

/**
 * @brief  Get the baud rate currently configured for the OTA UART.
 * @param  None
 * @retval uint32_t: Baud rate in bit/s
 * @note   Reads the compile-time setting, so a banner printed at run time can
 *         never disagree with the value programmed into the USART.
 */
uint32_t COM_GetBaudRate(void)
{
    return (uint32_t)MODEM_USART_BAUD_RATE;
}

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
