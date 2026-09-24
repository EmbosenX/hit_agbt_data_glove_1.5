/**
 *******************************************************************************
 * @file  iap/iap_boot_uart/source/main.c
 * @brief Main program of UART IAP Boot.
 *
 *        Bootloader based on the standard Y-modem protocol:
 *          - If a valid APP exists, jump to it directly.
 *          - Otherwise, send 'C' periodically to wait for a Y-modem transfer,
 *            receive the file, write it to flash and jump to the APP.
 *
 *        The waiting state is intentionally unbounded and never emits CAN: to
 *        every standard sender a CAN pair means "the receiver cancelled", which
 *        makes it close its transfer dialog at once. See YModem_Receive().
 *******************************************************************************
 */

#include "main.h"
#include "com.h"
#include "ymodem.h"

#include <string.h>

/* unlock/lock peripheral */
#define EXAMPLE_PERIPH_WE               (LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | \
                                         LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM)
#define EXAMPLE_PERIPH_WP               (LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_SRAM)

#define IAP_APP_ADDR                    (FLASH_BASE + IAP_BOOT_SIZE)
#define IAP_JUMP_DELAY_MS               (500UL)
/* Real milliseconds (SysTick based). Interval between two 'C' requests during
 * the block0 handshake, so it must not be smaller than
 * YMODEM_CRC_REQUEST_INTERVAL_MS. The data phase has its own header timeout
 * (@ref YMODEM_HEADER_TIMEOUT_MS) and is not affected by this value. */
#define IAP_YMODEM_TIMEOUT_MS           (YMODEM_CRC_REQUEST_INTERVAL_MS)

typedef void (*func_ptr_t)(void);

/* Longest line sent by the banner/diagnostic helpers. Every message is built
 * here first so a session report costs a single UART burst instead of one
 * burst per field. Static storage keeps the boot stack untouched. */
#define IAP_MSG_BUF_SIZE                (192U)

static uint32_t JumpAddr;
static func_ptr_t JumpToApp;
static stc_ymodem_io_t m_stcYmodemIo;
static stc_ymodem_info_t m_stcYmodemInfo;
static stc_ymodem_diag_t m_stcYmodemDiag;
static char m_szMsgBuf[IAP_MSG_BUF_SIZE];

static void IAP_CLK_DeInit(void);
static void IAP_PeriphInit(void);
static void IAP_PeriphDeinit(void);
static int32_t IAP_JumpToApp(uint32_t u32Addr);
static void IAP_CheckApp(void);
static int32_t IAP_FinalizeUpdate(void);
static int32_t IAP_PrepareAppArea(uint32_t u32FileSize);
static int32_t IAP_YmodemErase(uint32_t u32Addr, uint32_t u32Size);
static char *IAP_StrCat(char *pszDst, const char *pszSrc);
static char *IAP_AppendU32(char *pszDst, uint32_t u32Value);
static const char *IAP_ResultToStr(en_ymodem_result_t enResult);
static void IAP_SendBanner(void);
static void IAP_ReportSession(const stc_ymodem_diag_t *pstcDiag);


void SysTick_Handler(void)
{
    SysTick_IncTick();
    __DSB();
}

static void SysTick_DeInit(void)
{
    SysTick->CTRL = 0UL;
    SysTick->LOAD = 0UL;
    SysTick->VAL = 0UL;
}

static int32_t IAP_FinalizeUpdate(void)
{
    uint32_t u32AppAddr = IAP_APP_ADDR;
    uint32_t u32Flag = APP_EXIST_FLAG;

    if (LL_OK != FLASH_EraseSector(APP_UPGRADE_FLAG_ADDR, 0U)) {
        return LL_ERR;
    }
    if (LL_OK != FLASH_WriteData(APP_RUN_ADDR, (uint8_t *)&u32AppAddr, 4U)) {
        return LL_ERR;
    }
    if (LL_OK != FLASH_WriteData(APP_EXIST_FLAG_ADDR, (uint8_t *)&u32Flag, 4U)) {
        return LL_ERR;
    }

    return LL_OK;
}

static int32_t IAP_PrepareAppArea(uint32_t u32FileSize)
{
    uint32_t u32StartAddr;
    uint32_t u32EndAddr;
    uint32_t u32EraseAddr;

    u32StartAddr = IAP_APP_ADDR - (IAP_APP_ADDR % FLASH_SECTOR_SIZE);
    if (0U == u32FileSize) {
        /* File size unknown: erase the remaining flash sectors */
        u32EndAddr = FLASH_BASE + FLASH_SIZE - 1UL;
    } else {
        u32EndAddr = IAP_APP_ADDR + u32FileSize - 1UL;
    }
    u32EndAddr = u32EndAddr - (u32EndAddr % FLASH_SECTOR_SIZE);

    for (u32EraseAddr = u32StartAddr; u32EraseAddr <= u32EndAddr; u32EraseAddr += FLASH_SECTOR_SIZE) {
        if (LL_OK != FLASH_EraseSector(u32EraseAddr, 0U)) {
            return LL_ERR;
        }
    }

    return LL_OK;
}

static int32_t IAP_YmodemErase(uint32_t u32Addr, uint32_t u32Size)
{
    (void)u32Addr;

    /* Clear the pending upgrade flag first */
    if (LL_OK != FLASH_EraseSector(APP_UPGRADE_FLAG_ADDR, 0U)) {
        return LL_ERR;
    }
    return IAP_PrepareAppArea(u32Size);
}

static void IAP_CLK_DeInit(void)
{
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_MRC);
    (void)PWC_HighPerformanceToHighSpeed();
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV1 | CLK_PCLK0_DIV1 |
                                      CLK_PCLK1_DIV1 | CLK_PCLK2_DIV1 | CLK_PCLK3_DIV1 |
                                      CLK_PCLK4_DIV1));
    (void)CLK_HrcCmd(ENABLE);
}

static void IAP_PeriphInit(void)
{
    LL_PERIPH_WE(EXAMPLE_PERIPH_WE);
    BSP_CLK_Init();
    SysTick_Init(1000U);

    (void)memset(&m_stcYmodemIo, 0, sizeof(m_stcYmodemIo));
    (void)memset(&m_stcYmodemInfo, 0, sizeof(m_stcYmodemInfo));

    COM_Init();
}

static void IAP_PeriphDeinit(void)
{
    COM_DeInit();
    SysTick_DeInit();
    IAP_CLK_DeInit();
    LL_PERIPH_WP(EXAMPLE_PERIPH_WP);
}

static int32_t IAP_JumpToApp(uint32_t u32Addr)
{
    uint32_t u32StackTop = *((__IO uint32_t *)u32Addr);
    uint32_t i;

    if ((u32StackTop > SRAM_BASE) && (u32StackTop <= (SRAM_BASE + SRAM_SIZE))) {
        __disable_irq();
        for (i = 0UL; i < 8UL; ++i) {
            NVIC->ICER[i] = 0xFFFFFFFFUL;
            NVIC->ICPR[i] = 0xFFFFFFFFUL;
        }
        IAP_PeriphDeinit();
        SCB->VTOR = u32Addr;
        __DSB();
        __ISB();
        JumpAddr = *(__IO uint32_t *)(u32Addr + 4U);
        JumpToApp = (func_ptr_t)JumpAddr;
        __set_MSP(u32StackTop);
        __enable_irq();
        JumpToApp();
    }

    return LL_ERR;
}

static void IAP_CheckApp(void)
{
    uint32_t u32UpgradeFlag;
    uint32_t u32ExistFlag;
    uint32_t u32RunAddr;

    if (LL_OK != FLASH_ReadData(APP_UPGRADE_FLAG_ADDR, (uint8_t *)&u32UpgradeFlag, 4U)) {
        return;
    }
    if (APP_UPGRADE_FLAG == u32UpgradeFlag) {
        return;
    }

    if ((LL_OK == FLASH_ReadData(APP_EXIST_FLAG_ADDR, (uint8_t *)&u32ExistFlag, 4U)) &&
        (LL_OK == FLASH_ReadData(APP_RUN_ADDR, (uint8_t *)&u32RunAddr, 4U)) &&
        (APP_EXIST_FLAG == u32ExistFlag) &&
        (u32RunAddr >= IAP_APP_ADDR) &&
        (u32RunAddr < (FLASH_BASE + FLASH_SIZE))) {
        (void)IAP_JumpToApp(u32RunAddr);
    }
}

/**
 * @brief  Append a NUL terminated string to a message buffer.
 * @param  [in,out] pszDst              Current write position (return value of
 *                                      a previous append, or the buffer start)
 * @param  [in]     pszSrc              String to append
 * @retval char*: New write position
 * @note   Deliberately avoids printf/snprintf: the bootloader is size limited
 *         and pulling in stdio would cost several kilobytes of flash.
 */
static char *IAP_StrCat(char *pszDst, const char *pszSrc)
{
    while ('\0' != *pszSrc) {
        *pszDst = *pszSrc;
        pszDst++;
        pszSrc++;
    }
    *pszDst = '\0';

    return pszDst;
}

/**
 * @brief  Append an unsigned decimal number to a message buffer.
 * @param  [in,out] pszDst              Current write position
 * @param  [in]     u32Value            Value to append
 * @retval char*: New write position
 */
static char *IAP_AppendU32(char *pszDst, uint32_t u32Value)
{
    char szDigits[10];
    uint32_t u32Count = 0U;

    if (0U == u32Value) {
        *pszDst = '0';
        pszDst++;
        *pszDst = '\0';
        return pszDst;
    }

    while ((0U != u32Value) && (u32Count < sizeof(szDigits))) {
        szDigits[u32Count] = (char)('0' + (u32Value % 10U));
        u32Value /= 10U;
        u32Count++;
    }
    while (u32Count > 0U) {
        u32Count--;
        *pszDst = szDigits[u32Count];
        pszDst++;
    }
    *pszDst = '\0';

    return pszDst;
}

/**
 * @brief  Human readable name of a session result.
 */
static const char *IAP_ResultToStr(en_ymodem_result_t enResult)
{
    switch (enResult) {
        case YMODEM_RESULT_OK:          return "OK";
        case YMODEM_RESULT_TIMEOUT:     return "TIMEOUT";
        case YMODEM_RESULT_CRC_ERROR:   return "CRC_ERROR";
        case YMODEM_RESULT_BLOCK_ERROR: return "BLOCK_ERROR";
        case YMODEM_RESULT_CANCELLED:   return "CANCELLED";
        case YMODEM_RESULT_WRITE_ERROR: return "WRITE_ERROR";
        case YMODEM_RESULT_PARAM_ERROR: return "PARAM_ERROR";
        case YMODEM_RESULT_HEADER_ERROR: return "HEADER_ERROR";
        default:                        return "NONE";
    }
}

/**
 * @brief  Print the waiting banner, including the baud rate actually compiled
 *         in, so the host setting can be checked at a glance.
 * @param  None
 * @retval None
 * @note   Printed before the first handshake byte, so it does not disturb a
 *         Y-modem sender that is already waiting for 'C'.
 */
static void IAP_SendBanner(void)
{
    char *pszMsg = m_szMsgBuf;

    pszMsg = IAP_StrCat(pszMsg, "\r\n[IAP] No valid APP, waiting for Y-modem (baud ");
    pszMsg = IAP_AppendU32(pszMsg, COM_GetBaudRate());
    pszMsg = IAP_StrCat(pszMsg, ", send the .bin with 1K/CRC)...\r\n");

    COM_SendData((uint8_t *)m_szMsgBuf, (uint16_t)(pszMsg - m_szMsgBuf));
}

/**
 * @brief  Print one line describing why the previous session ended.
 * @param  [in] pstcDiag                Diagnostics filled by YModem_Receive()
 * @retval None
 * @note   Counters tell timeout, CRC and block number errors apart, which is
 *         what distinguishes "link too fast / byte lost" (rxerr/crc) from
 *         "protocol or host side problem" (blk) at 0 % progress.
 *         "hdr" counts bytes received where a Y-modem frame header was
 *         expected: hdr > 0 means the host did transmit, so a failure is a
 *         protocol/tool mismatch (wrong sender, wrong file, non-Y-modem tool),
 *         while tout > 0 with hdr == 0 means nothing arrived at all (cable,
 *         port or baud rate).
 */
static void IAP_ReportSession(const stc_ymodem_diag_t *pstcDiag)
{
    char *pszMsg = m_szMsgBuf;

    pszMsg = IAP_StrCat(pszMsg, "\r\n[IAP] Session end: ");
    pszMsg = IAP_StrCat(pszMsg, IAP_ResultToStr(pstcDiag->enResult));
    pszMsg = IAP_StrCat(pszMsg, ", file=");
    pszMsg = IAP_AppendU32(pszMsg, m_stcYmodemInfo.u32FileSize);
    pszMsg = IAP_StrCat(pszMsg, ", written=");
    pszMsg = IAP_AppendU32(pszMsg, m_stcYmodemInfo.u32WrittenSize);
    pszMsg = IAP_StrCat(pszMsg, ", blk_ok=");
    pszMsg = IAP_AppendU32(pszMsg, pstcDiag->u32BlockOk);
    pszMsg = IAP_StrCat(pszMsg, ", tout=");
    pszMsg = IAP_AppendU32(pszMsg, pstcDiag->u32TimeoutCnt);
    pszMsg = IAP_StrCat(pszMsg, ", crc=");
    pszMsg = IAP_AppendU32(pszMsg, pstcDiag->u32CrcErrCnt);
    pszMsg = IAP_StrCat(pszMsg, ", blk=");
    pszMsg = IAP_AppendU32(pszMsg, pstcDiag->u32BlockErrCnt);
    pszMsg = IAP_StrCat(pszMsg, ", hdr=");
    pszMsg = IAP_AppendU32(pszMsg, pstcDiag->u32HdrErrCnt);
    pszMsg = IAP_StrCat(pszMsg, ", rxerr=");
    pszMsg = IAP_AppendU32(pszMsg, COM_GetRxErrCnt());
    pszMsg = IAP_StrCat(pszMsg, "\r\n");

    COM_SendData((uint8_t *)m_szMsgBuf, (uint16_t)(pszMsg - m_szMsgBuf));
}

int main(void)
{
    int32_t i32Ret;
    uint8_t bHostInvolved;

    IAP_PeriphInit();
    IAP_CheckApp();

    m_stcYmodemIo.SendData = COM_SendData;
    m_stcYmodemIo.RecvData = COM_RecvData;
    m_stcYmodemIo.FlushRx = COM_FlushRx;
    m_stcYmodemIo.EraseSector = IAP_YmodemErase;
    m_stcYmodemIo.WriteData = FLASH_WriteData;
    m_stcYmodemInfo.u32FileAddr = IAP_APP_ADDR;

    IAP_SendBanner();

    for (;;) {
        i32Ret = YModem_Receive(&m_stcYmodemIo, &m_stcYmodemInfo,
                                IAP_YMODEM_TIMEOUT_MS, &m_stcYmodemDiag);

        /* Report only sessions the host actually took part in. Idling cannot
         * reach this point any more (the block0 handshake never gives up), but
         * any non-zero counter - including a plain timeout while waiting for a
         * data header - still proves the host was present and is worth a
         * diagnostic line. */
        bHostInvolved = ((YMODEM_RESULT_TIMEOUT != m_stcYmodemDiag.enResult) ||
                         (m_stcYmodemDiag.u32BlockOk > 0U) ||
                         (m_stcYmodemDiag.u32TimeoutCnt > 0U) ||
                         (m_stcYmodemDiag.u32CrcErrCnt > 0U) ||
                         (m_stcYmodemDiag.u32BlockErrCnt > 0U) ||
                         (m_stcYmodemDiag.u32HdrErrCnt > 0U) ||
                         (m_stcYmodemInfo.u32WrittenSize > 0U)) ? 1U : 0U;

        if (0U != bHostInvolved) {
            IAP_ReportSession(&m_stcYmodemDiag);
        }

        if ((LL_OK == i32Ret) && (m_stcYmodemInfo.u32WrittenSize > 0U)) {
            if (LL_OK == IAP_FinalizeUpdate()) {
                DDL_DelayMS(IAP_JUMP_DELAY_MS);
                (void)IAP_JumpToApp(IAP_APP_ADDR);
            }
        }

        /* Nothing else to do: stay in the waiting state so a new attempt can be
         * started without a power cycle. Re-print the banner only after a
         * reported session, as a hint that the receiver is ready again. */
        if ((YMODEM_RESULT_OK != m_stcYmodemDiag.enResult) && (0U != bHostInvolved)) {
            IAP_SendBanner();
        }
    }
}
