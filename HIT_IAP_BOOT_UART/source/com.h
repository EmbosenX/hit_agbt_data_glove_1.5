/**
 *******************************************************************************
 * @file  iap/iap_boot/source/com.h
 * @brief This file contains all the functions prototypes of the communication
 *        device driver.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2026-09-02       Custom          UART switch to USART3 (PE3=RX/PE4=TX), baud 2000000
   2026-09-11       Custom          OTA baud lowered to 115200 for a reliable baseline
   2026-09-11       Custom          OTA baud unified to 2000000 to match the App;
                                    fixed USART_GetStatus() misuse in the RX path
                                    (it returns SET/RESET, not a flag bit mask)
   2026-09-12       Custom          App run-time rate raised to 3 Mbps; OTA baud
                                    kept independent at 921600 (host switches)
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
#ifndef __COM_H__
#define __COM_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll_fcg.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_usart.h"
#include "hc32_ll_utility.h"

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define MODEM_USART_UNIT                (CM_USART3)
#define MODEM_USART_CLK_CTRL(state)     FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART3, state)

/* OTA baud rate. During a Y-modem transfer this must match the PC terminal
 * (e.g. Tera Term). It is INDEPENDENT of the App run-time rate: the App keeps
 * USART_BSP_BAUDRATE (3 Mbps, bsp/usart_bsp.h) for its data channel, while this
 * boot stays on its own 921600 baseline. The host therefore has to switch rates
 * after the hand-over (the web_calib_host upgrade wizard does that
 * automatically via its "Boot baud" field).
 *
 * Both sides still share one register setup (USART_CLK_DIV1 + 8x oversampling
 * on PCLK1 = 100 MHz) and the LL driver computes USART_BRR with its fractional
 * divider (CR1.FBME), so the rate only carries the usual divider rounding.
 *
 * NOTE: HC32F460 USART has no RX FIFO and this boot polls the register, so at
 * 921600 a byte lasts ~10.9 us. Y-modem is strictly lock-step (one frame, then
 * wait for the ACK), so the longest possible burst is a single 1K frame
 * (~11.4 ms) and the poll loop has to survive only that; see COM_WaitByte().
 * If a link ever proves marginal, check COM_GetRxErrCnt() (a non-zero value
 * after a failed transfer means overruns) and fall back to 115200 on both
 * sides before considering DMA. */
#define MODEM_USART_BAUD_RATE           (921600UL)

#define MODEM_USART_RX_PORT             (GPIO_PORT_E)
#define MODEM_USART_RX_PIN              (GPIO_PIN_03)
#define MODEM_USART_RX_FUNC             (GPIO_FUNC_33)

#define MODEM_USART_TX_PORT             (GPIO_PORT_E)
#define MODEM_USART_TX_PIN              (GPIO_PIN_04)
#define MODEM_USART_TX_FUNC             (GPIO_FUNC_32)

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
void COM_DeInit(void);
void COM_Init(void);
void COM_SendData(uint8_t *pu8Buff, uint16_t u16Len);
/* @param u32Timeout  total receive timeout in milliseconds (SysTick based),
 *                    counted from the first poll of the whole buffer, NOT a
 *                    per-byte value. 0 means "non-blocking". */
int32_t COM_RecvData(uint8_t *pu8Buff, uint16_t u16Len, uint32_t u32Timeout);
/* Discard every byte already pending in the RX register (resync helper). */
void COM_FlushRx(void);
/* Total number of UART RX error events (overrun / framing / parity) observed
 * since reset. A non-zero value after a failed transfer points at a link that
 * is too fast for this polling receiver, or at a host burst with no flow
 * control, rather than at a protocol problem. */
uint32_t COM_GetRxErrCnt(void);
/* Baud rate currently configured for the OTA UART (used by the startup banner
 * so the printed value can never drift from the compiled-in setting). */
uint32_t COM_GetBaudRate(void);

#ifdef __cplusplus
}
#endif

#endif /* __COM_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
