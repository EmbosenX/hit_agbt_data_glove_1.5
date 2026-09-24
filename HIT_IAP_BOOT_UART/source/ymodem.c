/**
 *******************************************************************************
 * @file  iap/iap_boot/source/ymodem.c
 * @brief This file provides firmware functions to manage the Y-modem protocol.
 *
 *        Standard Y-modem receiver:
 *          - Send 'C' to request block0 (file name packet) in CRC mode.
 *          - Parse file name and file size from block0, then erase flash.
 *          - Receive data blocks (SOH/STX), verify CRC16-XMODEM and write to flash.
 *          - Respond ACK/NAK/EOT/CAN and retransmit on timeout.
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

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "ymodem.h"

#include <string.h>

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Frame layout indexes of a data block after the header byte:
 *   u8Data[0]     : block number
 *   u8Data[1]     : complement of block number
 *   u8Data[2]     : data start
 *   u8Data[2+len] : CRC high byte
 *   u8Data[3+len] : CRC low byte
 */
#define YMODEM_BLOCK_NUM_INDEX          (0x00U)
#define YMODEM_BLOCK_XORNUM_INDEX       (0x01U)
#define YMODEM_DATA_INDEX               (0x02U)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t m_au8FrameData[YMODEM_MAX_DATA_SIZE + 4U];

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @brief  Calculate CRC16-XMODEM (poly 0x1021, init 0x0000, MSB first).
 * @param  [in] pu8Data                  Pointer to the data buffer
 * @param  [in] u32Len                   Data length
 * @retval uint16_t: CRC16 result
 */
uint16_t YModem_Crc16(const uint8_t *pu8Data, uint32_t u32Len)
{
    uint16_t u16Crc = 0U;

    if ((NULL != pu8Data) && (0U != u32Len)) {
        while (u32Len-- > 0U) {
            u16Crc ^= (uint16_t)((uint16_t)(*pu8Data++) << 8U);
            for (uint8_t u8Bit = 0U; u8Bit < 8U; u8Bit++) {
                if (0U != (u16Crc & 0x8000U)) {
                    u16Crc = (uint16_t)((u16Crc << 1U) ^ 0x1021U);
                } else {
                    u16Crc <<= 1U;
                }
            }
        }
    }

    return u16Crc;
}

/**
 * @brief  Send a single byte through the UART.
 * @param  [in] pstcIo                   Pointer to a @ref stc_ymodem_io_t structure
 * @param  [in] u8Data                   Byte to be sent
 * @retval None
 */
static void YModem_SendByte(stc_ymodem_io_t *pstcIo, uint8_t u8Data)
{
    pstcIo->SendData(&u8Data, 1U);
}

/**
 * @brief  Abort the current session with the standard Y-modem cancel sequence.
 * @param  [in] pstcIo                   Pointer to a @ref stc_ymodem_io_t structure
 * @retval None
 * @note   Two consecutive CAN bytes are what Y-modem senders (including Tera
 *         Term) require to stop retransmitting, so the host does not stay stuck
 *         in its own retry loop after this receiver has given up.
 */
static void YModem_Abort(stc_ymodem_io_t *pstcIo)
{
    YModem_SendByte(pstcIo, YMODEM_CAN);
    YModem_SendByte(pstcIo, YMODEM_CAN);
}

/**
 * @brief  Receive the body of a data frame (block number .. CRC) byte by byte.
 * @param  [in]  pstcIo                  Pointer to a @ref stc_ymodem_io_t structure
 * @param  [out] pu8Data                 Destination buffer (u16Len bytes)
 * @param  [in]  u16Len                  Body length = block length + 4
 * @retval int32_t:
 *           - LL_OK: Whole body received
 *           - LL_ERR: A byte did not arrive within @ref YMODEM_INTER_BYTE_TIMEOUT_MS
 * @note   Each byte is requested with an individual deadline instead of passing
 *         one long deadline for the whole buffer. This turns a truncated frame
 *         (a byte lost to an overrun) into a failure detected within ~100 ms
 *         rather than after the full header timeout, keeping the NAK/resend
 *         recovery loop fast. A byte only lasts 5 us at the OTA rate of
 *         2 Mbps, so the 100 ms inter-byte deadline is four orders of magnitude
 *         above the wire time and only trips on a genuinely dead link.
 */
static int32_t YModem_RecvFrameBody(stc_ymodem_io_t *pstcIo, uint8_t *pu8Data, uint16_t u16Len)
{
    uint16_t i;

    for (i = 0U; i < u16Len; i++) {
        if (LL_OK != pstcIo->RecvData(&pu8Data[i], 1U, YMODEM_INTER_BYTE_TIMEOUT_MS)) {
            return LL_ERR;
        }
    }

    return LL_OK;
}

/**
 * @brief  Parse the file name and file size from the block0 data area.
 * @param  [in]  pu8Data                 Pointer to the 128-byte block0 data
 * @param  [in]  u32DataSize             Data size (128 bytes)
 * @param  [out] pInfo                   Pointer to a @ref stc_ymodem_info_t structure
 * @retval None
 */
static void YModem_ParseBlock0(const uint8_t *pu8Data, uint32_t u32DataSize, stc_ymodem_info_t *pInfo)
{
    uint32_t u32Idx = 0U;
    uint32_t u32NameLen = 0U;
    uint32_t u32FileSize = 0U;

    /* Parse the file name (terminated by NUL) */
    while ((u32Idx < u32DataSize) && (0U != pu8Data[u32Idx])) {
        if (u32NameLen < (YMODEM_MAX_FILE_NAME - 1U)) {
            pInfo->au8FileName[u32NameLen] = pu8Data[u32Idx];
        }
        u32NameLen++;
        u32Idx++;
    }
    pInfo->au8FileName[YMODEM_MAX_FILE_NAME - 1U] = 0U;

    /* Skip the NUL terminator of the file name */
    if (u32Idx < u32DataSize) {
        u32Idx++;
    }

    /* Parse the file size (ASCII decimal, terminated by NUL) */
    while ((u32Idx < u32DataSize) && (0U != pu8Data[u32Idx])) {
        if ((pu8Data[u32Idx] >= '0') && (pu8Data[u32Idx] <= '9')) {
            u32FileSize = (u32FileSize * 10U) + (uint32_t)(pu8Data[u32Idx] - '0');
        }
        u32Idx++;
    }
    pInfo->u32FileSize = u32FileSize;
}

/**
 * @brief  Receive a file through the standard Y-modem protocol.
 * @param  [in]  pstcIo                  Pointer to a @ref stc_ymodem_io_t structure
 * @param  [in]  pInfo                   Pointer to a @ref stc_ymodem_info_t structure
 *                                        (u32FileAddr must be initialized by the caller)
 * @param  [in]  u32TimeoutMs            'C' request interval while waiting for
 *                                        block0, in real milliseconds (SysTick
 *                                        based); the data phase uses
 *                                        @ref YMODEM_HEADER_TIMEOUT_MS instead
 * @param  [out] pstcDiag                Optional (may be NULL) session diagnostics
 * @retval int32_t:
 *           - LL_OK: File received successfully (u32WrittenSize > 0), or
 *                    a null block0 received (u32FileSize == 0, transfer finished)
 *           - LL_ERR: Cancelled by the host, flash erase or write error
 *           - LL_ERR_TIMEOUT: Data phase retries exhausted
 *           - LL_ERR_INVD_PARAM: Invalid parameters
 * @note   While waiting for block0 this function does not return on its own: it
 *         keeps requesting with 'C' until the host answers or cancels, because
 *         aborting there makes every standard PC send dialog close at once.
 */
int32_t YModem_Receive(stc_ymodem_io_t *pstcIo, stc_ymodem_info_t *pInfo,
                       uint32_t u32TimeoutMs, stc_ymodem_diag_t *pstcDiag)
{
    stc_ymodem_diag_t stcDiagDummy;
    uint8_t  u8Byte;
    uint8_t  u8Retry = 0U;
    uint8_t  u8CanCnt = 0U;
    uint8_t  u8ExpectBlock;
    uint32_t u32CIntervalTicks;
    uint32_t u32CIdleTicks;
    uint32_t u32BlockLen;
    uint32_t u32WriteAddr;
    uint32_t u32Remain;
    uint32_t u32WriteLen;
    uint16_t u16CrcCal;
    uint16_t u16CrcRx;

    /* Diagnostics are optional: redirect to a scratch instance so the rest of
     * this function never has to test for NULL. */
    if (NULL == pstcDiag) {
        pstcDiag = &stcDiagDummy;
    }
    (void)memset(pstcDiag, 0, sizeof(stc_ymodem_diag_t));
    pstcDiag->enResult = YMODEM_RESULT_NONE;

    if ((NULL == pstcIo) || (NULL == pInfo) ||
        (NULL == pstcIo->SendData) || (NULL == pstcIo->RecvData) ||
        (NULL == pstcIo->EraseSector) || (NULL == pstcIo->WriteData) ||
        (0U == pInfo->u32FileAddr)) {
        pstcDiag->enResult = YMODEM_RESULT_PARAM_ERROR;
        return LL_ERR_INVD_PARAM;
    }

    pInfo->u32FileSize = 0U;
    pInfo->u32WrittenSize = 0U;
    pInfo->au8FileName[0] = 0U;

    /* Drop leftovers from a previous aborted attempt so the host and this
     * receiver start from a known-idle state. */
    if (NULL != pstcIo->FlushRx) {
        pstcIo->FlushRx();
    }

    /* ==================== Receive block0 (file name packet) ==================== */
    /* This phase is the receiver *inviting* the sender, so it must never abort:
     * two consecutive CAN bytes mean "receiver cancelled" to every standard
     * sender (Tera Term included), which then closes its transfer dialog at
     * once. A retry budget ending in YModem_Abort() is therefore forbidden
     * here - that is what made the PC send dialog close within a second while
     * the board was merely idling.
     *
     * The loop below is unbounded and keeps sending 'C' at the Y-modem
     * recommended interval (~1 s, derived from u32TimeoutMs) until a valid
     * block0 arrives; the only other exit is the host cancelling with CAN.
     * Received bytes are polled with the short inter-byte timeout, and any
     * received byte restarts the interval, so a chatty (non Y-modem) host is
     * never flooded with one 'C' per byte. */
    u32CIntervalTicks = (u32TimeoutMs + YMODEM_INTER_BYTE_TIMEOUT_MS - 1U) /
                        YMODEM_INTER_BYTE_TIMEOUT_MS;
    if (0U == u32CIntervalTicks) {
        u32CIntervalTicks = 1U;
    }
    u32CIdleTicks = 0U;
    YModem_SendByte(pstcIo, YMODEM_CRC_REQUEST);  /* First 'C' right away */

    for (;;) {
        if (LL_OK != pstcIo->RecvData(&u8Byte, 1U, YMODEM_INTER_BYTE_TIMEOUT_MS)) {
            /* Deliberately NOT counted in u32TimeoutCnt: that counter is meant
             * to quantify data-phase retries, while this branch fires every
             * 100 ms for as long as nobody has connected yet. */
            /* Silence: emit the next 'C' once a whole request interval has
             * elapsed, then keep waiting. Never returns, never sends CAN. */
            u32CIdleTicks++;
            if (u32CIdleTicks >= u32CIntervalTicks) {
                u32CIdleTicks = 0U;
                YModem_SendByte(pstcIo, YMODEM_CRC_REQUEST);
            }
            continue;
        }
        u32CIdleTicks = 0U;  /* Activity: restart the 'C' interval */
        if (YMODEM_CAN == u8Byte) {
            if (++u8CanCnt >= 2U) {
                pstcDiag->enResult = YMODEM_RESULT_CANCELLED;
                return LL_ERR;                        /* Sender cancelled */
            }
            continue;
        }
        u8CanCnt = 0U;
        /* The file information block may be sent either with SOH (128 bytes) or,
         * when the host selected 1K packets, with STX (1024 bytes). Both are
         * valid block0 headers and the body length follows the header, exactly
         * like the data block path below. Accepting only SOH here used to drop
         * a 1K block0 silently and fail the whole handshake in a few ms. */
        if ((YMODEM_SOH != u8Byte) && (YMODEM_STX != u8Byte)) {
            /* Not a frame header: the host is talking, but not Y-modem. Counted
             * so the session is still reported instead of ending in silence. */
            pstcDiag->u32HdrErrCnt++;
            continue;
        }
        u32BlockLen = (YMODEM_SOH == u8Byte) ? YMODEM_SOH_DATA_SIZE : YMODEM_STX_DATA_SIZE;
        /* Read block number, complement, data and CRC. A frame cut short is
         * NAKed and the handshake simply continues (never aborts). */
        if (LL_OK != YModem_RecvFrameBody(pstcIo, &m_au8FrameData[0], (uint16_t)(u32BlockLen + 4U))) {
            if (NULL != pstcIo->FlushRx) {
                pstcIo->FlushRx();
            }
            pstcDiag->u32TimeoutCnt++;
            YModem_SendByte(pstcIo, YMODEM_NAK);
            continue;
        }
        /* Block0 must be: 0x00 0xFF data crc */
        if ((0x00U != m_au8FrameData[YMODEM_BLOCK_NUM_INDEX]) ||
            (0xFFU != m_au8FrameData[YMODEM_BLOCK_XORNUM_INDEX])) {
            pstcDiag->u32BlockErrCnt++;
            YModem_SendByte(pstcIo, YMODEM_NAK);
            continue;
        }
        u16CrcCal = YModem_Crc16(&m_au8FrameData[YMODEM_DATA_INDEX], u32BlockLen);
        u16CrcRx = (uint16_t)(((uint16_t)m_au8FrameData[YMODEM_DATA_INDEX + u32BlockLen] << 8U) |
                              m_au8FrameData[YMODEM_DATA_INDEX + u32BlockLen + 1U]);
        if (u16CrcCal != u16CrcRx) {
            pstcDiag->u32CrcErrCnt++;
            YModem_SendByte(pstcIo, YMODEM_NAK);
            continue;
        }

        /* A null file name means the transfer is finished */
        if (0x00U == m_au8FrameData[YMODEM_DATA_INDEX]) {
            YModem_SendByte(pstcIo, YMODEM_ACK);
            pstcDiag->enResult = YMODEM_RESULT_OK;
            return LL_OK;
        }

        YModem_ParseBlock0(&m_au8FrameData[YMODEM_DATA_INDEX], u32BlockLen, pInfo);

        /* Erase the APP area according to the file size */
        if (LL_OK != pstcIo->EraseSector(pInfo->u32FileAddr, pInfo->u32FileSize)) {
            pstcDiag->enResult = YMODEM_RESULT_WRITE_ERROR;
            YModem_Abort(pstcIo);
            return LL_ERR;
        }

        YModem_SendByte(pstcIo, YMODEM_ACK);
        YModem_SendByte(pstcIo, YMODEM_CRC_REQUEST);  /* Request the first data block */
        break;
    }

    /* ==================== Receive data blocks ==================== */
    u32WriteAddr = pInfo->u32FileAddr;
    u8ExpectBlock = 1U;
    u8CanCnt = 0U;
    u8Retry = 0U;

    for (;;) {
        /* ---- Wait for the header byte of the next frame ---- */
        if (LL_OK != pstcIo->RecvData(&u8Byte, 1U, YMODEM_HEADER_TIMEOUT_MS)) {
            pstcDiag->u32TimeoutCnt++;
            if (++u8Retry >= YMODEM_MAX_RETRY) {
                pstcDiag->enResult = YMODEM_RESULT_TIMEOUT;
                YModem_Abort(pstcIo);
                return LL_ERR_TIMEOUT;
            }
            YModem_SendByte(pstcIo, YMODEM_NAK);
            continue;
        }

        if (YMODEM_CAN == u8Byte) {
            if (++u8CanCnt >= 2U) {
                pstcDiag->enResult = YMODEM_RESULT_CANCELLED;
                return LL_ERR;                        /* Sender cancelled */
            }
            continue;
        }
        u8CanCnt = 0U;

        if (YMODEM_EOT == u8Byte) {
            /* First EOT: NAK, then second EOT: ACK */
            YModem_SendByte(pstcIo, YMODEM_NAK);
            if (LL_OK != pstcIo->RecvData(&u8Byte, 1U, YMODEM_HEADER_TIMEOUT_MS)) {
                pstcDiag->enResult = YMODEM_RESULT_TIMEOUT;
                YModem_Abort(pstcIo);
                return LL_ERR_TIMEOUT;
            }
            if (YMODEM_EOT != u8Byte) {
                pstcDiag->enResult = YMODEM_RESULT_TIMEOUT;
                YModem_Abort(pstcIo);
                return LL_ERR;
            }
            YModem_SendByte(pstcIo, YMODEM_ACK);
            /* Accept the end-of-transfer null block0 if the sender sends one */
            YModem_SendByte(pstcIo, YMODEM_CRC_REQUEST);
            if ((LL_OK == pstcIo->RecvData(&u8Byte, 1U, YMODEM_HEADER_TIMEOUT_MS)) &&
                ((YMODEM_SOH == u8Byte) || (YMODEM_STX == u8Byte))) {
                u32BlockLen = (YMODEM_SOH == u8Byte) ? YMODEM_SOH_DATA_SIZE : YMODEM_STX_DATA_SIZE;
                if (LL_OK == pstcIo->RecvData(&m_au8FrameData[0], (uint16_t)(u32BlockLen + 4U), u32TimeoutMs)) {
                    if ((0x00U == m_au8FrameData[YMODEM_BLOCK_NUM_INDEX]) &&
                        (0xFFU == m_au8FrameData[YMODEM_BLOCK_XORNUM_INDEX]) &&
                        (0x00U == m_au8FrameData[YMODEM_DATA_INDEX])) {
                        YModem_SendByte(pstcIo, YMODEM_ACK);
                    }
                }
            }
            pstcDiag->enResult = YMODEM_RESULT_OK;
            return LL_OK;
        }

        if ((YMODEM_SOH == u8Byte) || (YMODEM_STX == u8Byte)) {
            u32BlockLen = (YMODEM_SOH == u8Byte) ? YMODEM_SOH_DATA_SIZE : YMODEM_STX_DATA_SIZE;

            /* Read block number, complement, data and CRC. Every byte has its
             * own short deadline, so a frame cut short by an overrun is
             * detected in ~100 ms instead of after the header timeout. */
            if (LL_OK != YModem_RecvFrameBody(pstcIo, &m_au8FrameData[0], (uint16_t)(u32BlockLen + 4U))) {
                /* The frame was cut short: drop the pending tail byte so it
                 * cannot be mistaken for a header of the next frame. */
                if (NULL != pstcIo->FlushRx) {
                    pstcIo->FlushRx();
                }
                pstcDiag->u32TimeoutCnt++;
                if (++u8Retry >= YMODEM_MAX_RETRY) {
                    pstcDiag->enResult = YMODEM_RESULT_TIMEOUT;
                    YModem_Abort(pstcIo);
                    return LL_ERR_TIMEOUT;
                }
                YModem_SendByte(pstcIo, YMODEM_NAK);
                continue;
            }
            /* Block number verification */
            if ((uint8_t)(m_au8FrameData[YMODEM_BLOCK_NUM_INDEX] ^ 0xFFU) != m_au8FrameData[YMODEM_BLOCK_XORNUM_INDEX]) {
                pstcDiag->u32BlockErrCnt++;
                if (++u8Retry >= YMODEM_MAX_RETRY) {
                    pstcDiag->enResult = YMODEM_RESULT_BLOCK_ERROR;
                    YModem_Abort(pstcIo);
                    return LL_ERR;
                }
                YModem_SendByte(pstcIo, YMODEM_NAK);
                continue;
            }
            /* CRC verification */
            u16CrcCal = YModem_Crc16(&m_au8FrameData[YMODEM_DATA_INDEX], u32BlockLen);
            u16CrcRx = (uint16_t)(((uint16_t)m_au8FrameData[YMODEM_DATA_INDEX + u32BlockLen] << 8U) |
                                  m_au8FrameData[YMODEM_DATA_INDEX + u32BlockLen + 1U]);
            if (u16CrcCal != u16CrcRx) {
                pstcDiag->u32CrcErrCnt++;
                if (++u8Retry >= YMODEM_MAX_RETRY) {
                    pstcDiag->enResult = YMODEM_RESULT_CRC_ERROR;
                    YModem_Abort(pstcIo);
                    return LL_ERR;
                }
                YModem_SendByte(pstcIo, YMODEM_NAK);
                continue;
            }
            /* Duplicated block: ACK without writing. The host clearly made
             * progress, so the retry budget is restored. */
            if (m_au8FrameData[YMODEM_BLOCK_NUM_INDEX] == (uint8_t)(u8ExpectBlock - 1U)) {
                u8Retry = 0U;
                YModem_SendByte(pstcIo, YMODEM_ACK);
                continue;
            }
            /* Unexpected block number */
            if (m_au8FrameData[YMODEM_BLOCK_NUM_INDEX] != u8ExpectBlock) {
                pstcDiag->u32BlockErrCnt++;
                if (++u8Retry >= YMODEM_MAX_RETRY) {
                    pstcDiag->enResult = YMODEM_RESULT_BLOCK_ERROR;
                    YModem_Abort(pstcIo);
                    return LL_ERR;
                }
                YModem_SendByte(pstcIo, YMODEM_NAK);
                continue;
            }
            /* Calculate the actual write length of this block.
             * If the file size is unknown (0), write the whole block. */
            if (0U == pInfo->u32FileSize) {
                u32WriteLen = u32BlockLen;
            } else {
                u32Remain = pInfo->u32FileSize - pInfo->u32WrittenSize;
                u32WriteLen = (u32Remain < u32BlockLen) ? u32Remain : u32BlockLen;
            }
            if (0U == u32WriteLen) {
                u8Retry = 0U;
                YModem_SendByte(pstcIo, YMODEM_ACK);
                continue;
            }
            if (LL_OK != pstcIo->WriteData(u32WriteAddr, &m_au8FrameData[YMODEM_DATA_INDEX], u32WriteLen)) {
                pstcDiag->enResult = YMODEM_RESULT_WRITE_ERROR;
                YModem_Abort(pstcIo);
                return LL_ERR;
            }
            u32WriteAddr += u32WriteLen;
            pInfo->u32WrittenSize += u32WriteLen;
            u8ExpectBlock++;
            pstcDiag->u32BlockOk++;
            /* Real progress: the retry budget is restored only here (and for a
             * duplicated block above), never on a mere byte arrival. */
            u8Retry = 0U;
            YModem_SendByte(pstcIo, YMODEM_ACK);
            continue;
        }

        /* Any other byte is noise and must NOT clear u8Retry. Clearing it on
         * every received byte was the original defect: the header byte of a
         * retransmitted frame always arrives, so u8Retry stayed pinned at 0,
         * this receiver NAKed forever and the host eventually gave up at 0 %.
         * It is only counted, so a non-Y-modem sender still shows up in the
         * session report instead of ending in silence. */
        pstcDiag->u32HdrErrCnt++;
    }
}

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
