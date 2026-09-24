/**
 * @file  iap_boot.c
 * @brief IAP Bootloader 主逻辑（主/从机共用）
 *
 *        Bootloader 独立程序（0x00000000, 16KB）调用 IapBoot_Check()：
 *        1. 读升级标志区，若 magic 有效且 CRC 校验通过，则把缓存区固件搬运到 App 区；
 *        2. 校验 App 区有效后跳转执行。
 *
 *        App 端在完成固件接收（写入缓存区）并写好升级标志后，调用
 *        Iap_JumpToBoot() 复位，重启进入 Bootloader 完成搬运。
 */
#include "iap_boot.h"
#include "iap_flash.h"

/* HC32F460 SRAM 范围（本工程 RAM: 0x1FFF8000, 188KB） */
#define IAP_SRAM_BASE   (0x1FFF8000UL)
#define IAP_SRAM_END    (0x20027000UL)

/* 搬运缓存（Bootloader 阶段使用，栈上大数组避免与 App 冲突） */
#define IAP_COPY_BUF_SIZE   (512UL)

void Iap_JumpToApp(uint32_t u32Addr)
{
    uint32_t u32Sp;
    uint32_t u32Pc;

    u32Sp = *(volatile uint32_t *)u32Addr;
    u32Pc = *(volatile uint32_t *)(u32Addr + 4UL);

    __disable_irq();
    SCB->VTOR = u32Addr;          /* 重映射向量表 */
    __set_MSP(u32Sp);             /* 重设主栈指针 */
    ((void (*)(void))u32Pc)();    /* 跳转复位向量 */
    while (1) { }
}

uint32_t IapBoot_IsAppValid(void)
{
    uint32_t u32Sp = *(volatile uint32_t *)IAP_APP_ADDR;
    uint32_t u32Pc = *(volatile uint32_t *)(IAP_APP_ADDR + 4UL);

    /* SP 落在 SRAM 区间，复位向量落在 App 区且为 Thumb 指令 */
    if ((u32Sp < IAP_SRAM_BASE) || (u32Sp >= IAP_SRAM_END)) {
        return 0UL;
    }
    if ((u32Pc < IAP_APP_ADDR) || (u32Pc >= (IAP_APP_ADDR + IAP_APP_SIZE))) {
        return 0UL;
    }
    if (0UL != (u32Pc & 1UL)) {
        return 0UL;
    }
    return 1UL;
}

static int32_t IapBoot_CopyCacheToApp(uint32_t u32FwSize)
{
    uint8_t  au8Buf[IAP_COPY_BUF_SIZE];
    uint32_t u32Off = 0UL;
    uint32_t u32N;
    int32_t  r;

    for (u32Off = 0UL; u32Off < u32FwSize; u32Off += u32N) {
        u32N = (u32FwSize - u32Off) > IAP_COPY_BUF_SIZE ? IAP_COPY_BUF_SIZE : (u32FwSize - u32Off);
        r = IapFlash_Read(IAP_CACHE_ADDR + u32Off, au8Buf, u32N);
        if (LL_OK != r) { return r; }
        r = IapFlash_Write(IAP_APP_ADDR + u32Off, au8Buf, u32N);
        if (LL_OK != r) { return r; }
    }
    return LL_OK;
}

void IapBoot_Check(void)
{
    iap_boot_flag_t stFlag;

    IapFlash_Init();

    /* 1. 检查升级标志 */
    if (LL_OK == IapFlash_ReadBootFlag(&stFlag) && (IAP_FLAG_MAGIC == stFlag.u32Magic)) {
        if ((stFlag.u32FwSize > 0UL) && (stFlag.u32FwSize <= IAP_FW_MAX_SIZE)) {
            /* 校验缓存区固件 CRC */
            if (IapFlash_CRC32Range(IAP_CACHE_ADDR, stFlag.u32FwSize) == stFlag.u32FwCrc) {
                /* 擦除 App 区并搬运 */
                if (LL_OK == IapFlash_EraseRegion(IAP_APP_ADDR, IAP_APP_SIZE)) {
                    if (LL_OK == IapBoot_CopyCacheToApp(stFlag.u32FwSize)) {
                        /* 校验写入后的 App 区 */
                        if (IapFlash_CRC32Range(IAP_APP_ADDR, stFlag.u32FwSize) == stFlag.u32FwCrc) {
                            (void)IapFlash_ClearBootFlag();
                        }
                    }
                }
            }
        } else {
            (void)IapFlash_ClearBootFlag();
        }
    }

    /* 2. 跳转 App */
    if (IapBoot_IsAppValid()) {
        Iap_JumpToApp(IAP_APP_ADDR);
    }

    /* 3. App 无效：停留等待重新升级（可在此点亮指示灯/喂狗） */
    while (1) { }
}

void Iap_JumpToBoot(void)
{
    __disable_irq();
    NVIC_SystemReset();
}
