/**
 *******************************************************************************
 * @file  iap/iap_boot/source/ymodem.h
 * @brief This file contains all the functions prototypes of the Y-modem protocol.
 *******************************************************************************
 * Copyright (C) 2026, All rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#ifndef __YMODEM_H__
#define __YMODEM_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll_def.h"

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Y-modem control characters */
#define YMODEM_SOH                       (0x01U)
#define YMODEM_STX                       (0x02U)
#define YMODEM_EOT                       (0x04U)
#define YMODEM_ACK                       (0x06U)
#define YMODEM_NAK                       (0x15U)
#define YMODEM_CAN                       (0x18U)
#define YMODEM_CRC_REQUEST               (0x43U)  /* 'C' */
#define YMODEM_FILL_BYTE                 (0x1AU)

/* Y-modem packet definitions */
#define YMODEM_BLOCK0_DATA_SIZE          (128U)
#define YMODEM_SOH_DATA_SIZE             (128U)
#define YMODEM_STX_DATA_SIZE             (1024U)
#define YMODEM_MAX_DATA_SIZE             (YMODEM_STX_DATA_SIZE)

/* Y-modem retry budget. Applies to the *data* phase only: there it takes a
 * genuine link/host failure to exhaust it, and the standard CAN abort is the
 * correct answer. The block0 handshake deliberately has NO retry budget: it is
 * the receiver inviting the sender and must never send CAN (see ymodem.c). */
#define YMODEM_MAX_RETRY                 (10U)

/* Timeout for the first byte of a new frame (waiting for SOH/STX/EOT/CAN).
 * It is deliberately longer than the inter-byte timeout because the sender may
 * need a few hundred milliseconds between two blocks (host-side CRC
 * computation, file/log access) and a too short window would provoke spurious
 * NAKs, which in turn slows the whole transfer down. */
#define YMODEM_HEADER_TIMEOUT_MS         (3000U)

/* Inter-byte timeout used *inside* a frame: once the header byte (SOH/STX) has
 * been received, every following byte (block number, data, CRC) must arrive
 * within this window. A frame cut short by a lost byte is therefore detected in
 * ~100 ms instead of after the whole header timeout, which makes the NAK/resend
 * recovery loop an order of magnitude faster.
 * Must stay far above one byte time (87 us at 115200 baud, 5 us at 2 Mbps). */
#define YMODEM_INTER_BYTE_TIMEOUT_MS     (100U)

/* Interval used between two 'C' (CRC request) characters while waiting for
 * block0. The Y-modem spec recommends about one 'C' per second, which is also
 * what Tera Term expects: sending 'C' much faster makes the host see extra
 * state bytes while it is waiting for an ACK, causing spurious retransmissions
 * of block0 and, eventually, an aborted transfer. Pass a 1000 ms (or larger)
 * value as u32TimeoutMs of YModem_Receive() to honour this. */
#define YMODEM_CRC_REQUEST_INTERVAL_MS   (1000U)

/* Y-modem file name length */
#define YMODEM_MAX_FILE_NAME             (64U)

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/
/**
 * @brief Y-modem I/O hooks: bind to the underlying UART driver and Flash driver.
 */
typedef struct {
    void (*SendData)(uint8_t *pu8Buff, uint16_t u16Len);                /*!< Send data through UART */
    int32_t (*RecvData)(uint8_t *pu8Buff, uint16_t u16Len, uint32_t u32Timeout);  /*!< Receive data, timeout in real milliseconds */
    void (*FlushRx)(void);                                              /*!< Optional: discard pending RX bytes, may be NULL */
    int32_t (*EraseSector)(uint32_t u32Addr, uint32_t u32Size);         /*!< Erase flash sectors */
    int32_t (*WriteData)(uint32_t u32Addr, uint8_t *pu8Buff, uint32_t u32Len);   /*!< Write data to flash */
} stc_ymodem_io_t;

/**
 * @brief Y-modem transfer information.
 */
typedef struct {
    uint32_t u32FileAddr;        /*!< [in]  Flash write start address (e.g. APP start address) */
    uint32_t u32FileSize;        /*!< [out] File size parsed from block0 */
    uint32_t u32WrittenSize;     /*!< [out] Actually written size */
    uint8_t  au8FileName[YMODEM_MAX_FILE_NAME];  /*!< [out] File name parsed from block0 */
} stc_ymodem_info_t;

/**
 * @brief Why a Y-modem session ended. Reported through @ref stc_ymodem_diag_t
 *        so that the caller can print an actionable message instead of a
 *        generic "transfer failed".
 */
typedef enum {
    YMODEM_RESULT_NONE = 0,      /*!< Session still running / not filled */
    YMODEM_RESULT_OK,            /*!< EOT sequence received, transfer complete */
    YMODEM_RESULT_TIMEOUT,       /*!< Header or intra-frame timeout, retries exhausted */
    YMODEM_RESULT_CRC_ERROR,     /*!< CRC16 mismatch, retries exhausted */
    YMODEM_RESULT_BLOCK_ERROR,   /*!< Block number / complement invalid, retries exhausted */
    YMODEM_RESULT_CANCELLED,     /*!< Host sent CAN (user aborted) */
    YMODEM_RESULT_WRITE_ERROR,   /*!< Flash erase or write failed */
    YMODEM_RESULT_PARAM_ERROR,   /*!< Invalid parameters passed to YModem_Receive() */
    YMODEM_RESULT_HEADER_ERROR   /*!< Header is neither SOH/STX/EOT/CAN: the host
                                      is not sending Y-modem (wrong tool, wrong
                                      protocol, plain text). Kept for reports;
                                      the block0 handshake no longer aborts on
                                      this condition, it only counts hdrerr */
} en_ymodem_result_t;

/**
 * @brief Y-modem session diagnostics. Filled by YModem_Receive(), read-only for
 *        the caller. All counters are cheap increments that never take part in
 *        the receive decision, so they do not disturb the byte timing.
 */
typedef struct {
    en_ymodem_result_t enResult;      /*!< Outcome of the session */
    uint32_t u32BlockOk;              /*!< Blocks successfully written and ACKed */
    uint32_t u32TimeoutCnt;           /*!< Timed out waiting for a header or a frame body */
    uint32_t u32CrcErrCnt;            /*!< CRC16 mismatch count */
    uint32_t u32BlockErrCnt;          /*!< Bad block number / complement count */
    uint32_t u32RxErrCnt;             /*!< UART overrun / framing / parity events */
    uint32_t u32HdrErrCnt;            /*!< Bytes received where a frame header
                                           (SOH/STX/EOT/CAN) was expected. Non-zero
                                           means the host did talk on the wire, but
                                           not in Y-modem. Noise deliberately does
                                           not clear the retry budget, so the wire
                                           timing of a well-behaved host is
                                           unchanged; this counter is observation
                                           only and never drives a decision. */
} stc_ymodem_diag_t;

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
uint16_t YModem_Crc16(const uint8_t *pu8Data, uint32_t u32Len);

/**
 * @brief  Receive a file through the standard Y-modem protocol.
 * @param  [in]  pstcIo                  Pointer to a @ref stc_ymodem_io_t structure
 * @param  [in]  pInfo                   Pointer to a @ref stc_ymodem_info_t structure
 *                                       (u32FileAddr must be initialized by the caller)
 * @param  [in]  u32TimeoutMs            Byte-blocking timeout in real milliseconds
 *                                       used for the block0 handshake and the
 *                                       'C' request interval
 * @param  [out] pstcDiag                Optional (may be NULL) session diagnostics
 * @retval int32_t:
 *           - LL_OK: File received successfully (see pstcDiag->enResult)
 *           - LL_ERR: Communication cancelled or write error
 *           - LL_ERR_TIMEOUT: Retry times exhausted
 *           - LL_ERR_INVD_PARAM: Invalid parameters
 */
int32_t YModem_Receive(stc_ymodem_io_t *pstcIo, stc_ymodem_info_t *pInfo,
                       uint32_t u32TimeoutMs, stc_ymodem_diag_t *pstcDiag);

#ifdef __cplusplus
}
#endif

#endif /* __YMODEM_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
