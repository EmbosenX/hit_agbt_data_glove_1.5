/**
 * @file  iap_flash.c
 * @brief IAP Flash 驱动实现（主/从机共用）
 *        编程用 EFM_SequenceProgram(__RAM_FUNC 运行于 RAM)；
 *        擦除用 EFM_SectorErase，HC32F460 支持跨扇区执行，本工程分区保证
 *        被擦扇区与执行代码扇区不重叠（App 只擦缓存区，Bootloader 只擦 App 区）。
 */
#include "iap_flash.h"
#include <string.h>

static uint32_t s_au32CrcTable[256];
static uint8_t  s_u8CrcInited = 0U;

static void IapFlash_CrcTableInit(void)
{
    uint32_t c, i, j;
    for (i = 0UL; i < 256UL; i++) {
        c = i;
        for (j = 0UL; j < 8UL; j++) {
            c = (c & 1UL) ? ((c >> 1UL) ^ 0xEDB88320UL) : (c >> 1UL);
        }
        s_au32CrcTable[i] = c;
    }
    s_u8CrcInited = 1U;
}

uint32_t IapFlash_CRC32_Init(void)
{
    if (0U == s_u8CrcInited) { IapFlash_CrcTableInit(); }
    return 0xFFFFFFFFUL;
}

uint32_t IapFlash_CRC32_Update(uint32_t u32Crc, const uint8_t *p, uint32_t n)
{
    uint32_t i;
    if (0U == s_u8CrcInited) { IapFlash_CrcTableInit(); }
    for (i = 0UL; i < n; i++) {
        u32Crc = (u32Crc >> 8UL) ^ s_au32CrcTable[(u32Crc ^ p[i]) & 0xFFUL];
    }
    return u32Crc;
}

uint32_t IapFlash_CRC32_Final(uint32_t u32Crc)
{
    return u32Crc ^ 0xFFFFFFFFUL;
}

uint32_t IapFlash_CRC32(const uint8_t *p, uint32_t n)
{
    return IapFlash_CRC32_Final(IapFlash_CRC32_Update(IapFlash_CRC32_Init(), p, n));
}

uint32_t IapFlash_CRC32Range(uint32_t u32Addr, uint32_t u32Len)
{
    uint8_t  au8Buf[128];
    uint32_t u32Crc = IapFlash_CRC32_Init();
    uint32_t u32Tmp, u32N;

    u32Tmp = READ_REG32_BIT(CM_EFM->FRMC, EFM_CACHE_ALL);
    CLR_REG32_BIT(CM_EFM->FRMC, EFM_CACHE_ALL);
    while (u32Len > 0UL) {
        u32N = (u32Len > sizeof(au8Buf)) ? (uint32_t)sizeof(au8Buf) : u32Len;
        memcpy(au8Buf, (const void *)u32Addr, u32N);
        u32Crc = IapFlash_CRC32_Update(u32Crc, au8Buf, u32N);
        u32Addr += u32N;
        u32Len  -= u32N;
    }
    MODIFY_REG32(CM_EFM->FRMC, EFM_CACHE_ALL, u32Tmp);
    return IapFlash_CRC32_Final(u32Crc);
}

void IapFlash_Init(void)
{
    EFM_FWMC_Cmd(ENABLE);
    EFM_REG_Unlock();
    EFM_SetBusStatus(EFM_BUS_RELEASE);
    (void)EFM_SetWaitCycle(EFM_WAIT_CYCLE1);
    (void)EFM_SetOperateMode(EFM_MD_READONLY);
    EFM_REG_Lock();
}

int32_t IapFlash_EraseRegion(uint32_t u32Addr, uint32_t u32Size)
{
    int32_t  r = LL_OK;
    uint32_t end;

    if ((0UL == u32Size) ||
        (0UL != (u32Addr & (EFM_SECTOR_SIZE - 1UL))) ||
        (0UL != (u32Size & (EFM_SECTOR_SIZE - 1UL)))) {
        return LL_ERR_INVD_PARAM;
    }
    end = u32Addr + u32Size;
    if (end > EFM_END_ADDR) {
        return LL_ERR_INVD_PARAM;
    }

    EFM_REG_Unlock();
    while (u32Addr < end) {
        r = EFM_SectorErase(u32Addr);
        if (LL_OK != r) { break; }
        u32Addr += EFM_SECTOR_SIZE;
    }
    EFM_REG_Lock();
    return r;
}

int32_t IapFlash_Write(uint32_t u32Addr, const uint8_t *p, uint32_t u32Len)
{
    int32_t  r = LL_OK;
    uint32_t aligned;
    uint8_t  tail[4];

    if (0UL == u32Len) { return LL_OK; }
    if (0UL != (u32Addr & 0x3UL)) { return LL_ERR_INVD_PARAM; }

    EFM_REG_Unlock();

    aligned = u32Len & ~3UL;
    if (aligned > 0UL) {
        r = EFM_SequenceProgram(u32Addr, p, aligned);
        if (LL_OK != r) { EFM_REG_Lock(); return r; }
        u32Addr += aligned;
        p       += aligned;
        u32Len  -= aligned;
    }
    if (u32Len > 0UL) {
        tail[0] = p[0];
        tail[1] = (u32Len > 1UL) ? p[1] : 0xFFU;
        tail[2] = (u32Len > 2UL) ? p[2] : 0xFFU;
        tail[3] = 0xFFU;
        r = EFM_SequenceProgram(u32Addr, tail, 4UL);
    }

    EFM_REG_Lock();
    return r;
}

int32_t IapFlash_Read(uint32_t u32Addr, uint8_t *p, uint32_t u32Len)
{
    uint32_t u32Tmp;
    if ((u32Addr + u32Len) > EFM_END_ADDR) {
        return LL_ERR_INVD_PARAM;
    }
    u32Tmp = READ_REG32_BIT(CM_EFM->FRMC, EFM_CACHE_ALL);
    CLR_REG32_BIT(CM_EFM->FRMC, EFM_CACHE_ALL);
    memcpy(p, (const void *)u32Addr, u32Len);
    MODIFY_REG32(CM_EFM->FRMC, EFM_CACHE_ALL, u32Tmp);
    return LL_OK;
}

int32_t IapFlash_WriteBootFlag(uint32_t u32FwSize, uint32_t u32FwCrc)
{
    iap_boot_flag_t stFlag;
    int32_t r;

    stFlag.u32Magic    = IAP_FLAG_MAGIC;
    stFlag.u32FwSize   = u32FwSize;
    stFlag.u32FwCrc    = u32FwCrc;
    stFlag.u32Reserved = 0UL;

    r = IapFlash_EraseRegion(IAP_FLAG_ADDR, IAP_FLAG_SIZE);
    if (LL_OK != r) { return r; }
    return IapFlash_Write(IAP_FLAG_ADDR, (const uint8_t *)&stFlag, sizeof(stFlag));
}

int32_t IapFlash_ReadBootFlag(iap_boot_flag_t *pstFlag)
{
    return IapFlash_Read(IAP_FLAG_ADDR, (uint8_t *)pstFlag, sizeof(*pstFlag));
}

int32_t IapFlash_ClearBootFlag(void)
{
    return IapFlash_EraseRegion(IAP_FLAG_ADDR, IAP_FLAG_SIZE);
}
