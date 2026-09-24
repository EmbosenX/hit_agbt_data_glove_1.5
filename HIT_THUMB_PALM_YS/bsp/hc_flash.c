/**
 *******************************************************************************
 * @file    hc_flash.c
 * @brief   片内 EFM Flash 驱动实现（通用擦写读 + 标定表掉电存储）
 *
 *          实现要点（地址布局与各条约束的完整说明见 hc_flash.h 文件头）：
 *          - 编程用 EFM_SequenceProgram（__RAM_FUNC 运行于 RAM），擦除用
 *            EFM_SectorErase，读前关闭 Flash Cache 以保证回读到最新数据；
 *          - 禁区 0x00000000~0x00007FFF（boot 代码 + IAP 标志扇区）一律拒绝
 *            擦写并返回 LL_ERR_INVD_PARAM，读取不受限制；
 *          - 擦除前提：目标扇区不得与代码所在扇区重叠，故禁止擦除 APP 自身扇区；
 *          - 擦写期间总线必须保持 EFM_BUS_HOLD，否则 CPU 取指取到无效指令即跑飞；
 *          - 擦写内核期用 __disable_irq 全屏蔽中断，退出时按 PRIMASK 原值恢复，
 *            不依赖 FreeRTOS 内核状态，调度器启动前亦可调用；
 *          - 本模块不修改 Flash 等待周期（200MHz 下 FLWT 违规即跑飞）。
 *******************************************************************************
 */
#include "hc_flash.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/*******************************************************************************
 * 内部辅助：EFM 加解锁
 ******************************************************************************/
/**
 * @brief  解除 EFM 寄存器保护（FWMC 等受 FAPRT 保护，写前必须解锁）
 * @note   仅本文件内部的临界段包裹使用，配对调用 HcFlash_Lock()
 */
static void HcFlash_Unlock(void)
{
    EFM_REG_Unlock();
}

/**
 * @brief  恢复 EFM 寄存器保护
 * @note   与 HcFlash_Unlock() 配对，擦写内核期结束后立即调用
 */
static void HcFlash_Lock(void)
{
    EFM_REG_Lock();
}

/*******************************************************************************
 * 全局函数实现
 ******************************************************************************/
/**
 * @brief  初始化 EFM：使能 FWMC、设置擦写期间保持总线、置只读工作模式
 * @note   上电后调用一次即可；无临界段，可在调度器启动前调用（详见 hc_flash.h）
 */
void HcFlash_Init(void)
{
    /* 顺序必须与 app_iap.c 的可用流程一致：FWMC（含 PEMODE）受 FAPRT 保护，
     * 必须先 EFM_REG_Unlock() 再写 FWMC，否则写入无效，
     * 后续 EFM_SetBusStatus / EFM_SetOperateMode 的 IS_EFM_FWMC_UNLOCK() 断言会失败 */
    EFM_REG_Unlock();
    EFM_FWMC_Cmd(ENABLE);
    /* 擦写期间必须保持总线（HOLD）：EFM_SectorErase 在 Flash 内运行，
     * 若 RELEASE 则擦除期间 CPU 取指会取到无效指令而跑飞（见文件头说明）。
     * 此处与 app_iap.c 的可用流程一致（其保持复位默认 HOLD，未调用本接口）。 */
    EFM_SetBusStatus(EFM_BUS_HOLD);
    /* 等待周期由 BSP_RC_CLK_Init 按主频设置（200MHz 为 5 周期），此处不重复配置 */
    (void)EFM_SetOperateMode(EFM_MD_READONLY);
    EFM_REG_Lock();
}

/**
 * @brief  擦除单个扇区（8KB 对齐；约 20~40ms，期间屏蔽中断）
 * @param  u32Addr  扇区地址
 * @retval LL_OK / LL_ERR_INVD_PARAM / LL_ERR
 * @note   接口说明见 hc_flash.h；地址落在禁区（<= 0x7FFF）直接拒绝
 */
int32_t HcFlash_EraseSector(uint32_t u32Addr)
{
    int32_t  i32Ret;
    uint32_t u32Primask;

    /* 地址须落在 Flash 内且 8KB 扇区对齐；禁区（boot + IAP 标志扇区）拒绝擦除。
     * u32Addr 为无符号数恒 >= HC_FLASH_BASE(0x0)，且禁区判定已隐含下界，故不比较下界 */
    if ((u32Addr > HC_FLASH_END) ||
        (0UL != (u32Addr & (HC_FLASH_SECTOR_SIZE - 1UL))) ||
        (u32Addr <= HC_FLASH_PROTECT_END)) {
        return LL_ERR_INVD_PARAM;
    }

    while (SET != EFM_GetStatus(EFM_FLAG_RDY)) {
        /* 等待 EFM 空闲 */
    }
    /* 擦除内核期进临界段：全屏蔽中断，避免任务切换/中断对 EFM 保护序列插队
     * （见文件头说明）。时长约 20~40ms，仅在标定表下发/擦除命令时发生。
     * 退出时按保存的 PRIMASK 原值恢复，不误开上层已关闭的中断。 */
    u32Primask = __get_PRIMASK();
    __disable_irq();
    HcFlash_Unlock();
    i32Ret = EFM_SectorErase(u32Addr);
    HcFlash_Lock();
    if (0UL == (u32Primask & 1UL)) {
        __enable_irq();
    }

    return i32Ret;
}

/**
 * @brief  擦除地址区间（地址与长度均须 8KB 扇区对齐，且不得落入禁区）
 * @param  u32Addr  起始地址
 * @param  u32Size  长度（须为扇区大小整数倍且非 0）
 * @retval LL_OK / LL_ERR_INVD_PARAM / LL_ERR（首个失败扇区的返回值）
 * @note   接口说明见 hc_flash.h
 */
int32_t HcFlash_EraseRegion(uint32_t u32Addr, uint32_t u32Size)
{
    int32_t  i32Ret = LL_OK;
    uint32_t u32End;

    if ((0UL == u32Size) ||
        (0UL != (u32Addr & (HC_FLASH_SECTOR_SIZE - 1UL))) ||
        (0UL != (u32Size & (HC_FLASH_SECTOR_SIZE - 1UL))) ||
        (u32Addr <= HC_FLASH_PROTECT_END)) {
        return LL_ERR_INVD_PARAM;
    }
    u32End = u32Addr + u32Size;
    if (u32End > (HC_FLASH_END + 1UL)) {
        return LL_ERR_INVD_PARAM;
    }

    while (u32Addr < u32End) {
        i32Ret = HcFlash_EraseSector(u32Addr);
        if (LL_OK != i32Ret) {
            break;
        }
        u32Addr += HC_FLASH_SECTOR_SIZE;
    }
    return i32Ret;
}

/**
 * @brief  写 Flash（目标区须已擦除；4 字节对齐；禁区拒绝）
 * @param  u32Addr   目标地址（须 4 字节对齐）
 * @param  pu8Data   源数据
 * @param  u32Len    数据长度（尾部不足 4 字节自动补 0xFF 后编程）
 * @retval LL_OK / LL_ERR_INVD_PARAM / EFM 编程错误码
 * @note   接口说明见 hc_flash.h；内核期屏蔽中断，按 PRIMASK 原值恢复
 */
int32_t HcFlash_Write(uint32_t u32Addr, const uint8_t *pu8Data, uint32_t u32Len)
{
    /* EFM_SequenceProgram 内部把源指针按 uint32_t* 解引用，要求源地址 4 字节对齐，
       故先用对齐的栈上搬运缓冲分块拷贝（尾块不足 4 字节补 0xFF）再编程 */
    uint32_t au32Bounce[HC_FLASH_BOUNCE_WORDS];
    uint8_t *pu8Bounce = (uint8_t *)(void *)au32Bounce;
    int32_t  i32Ret = LL_OK;
    uint32_t u32Off = 0UL;
    uint32_t u32Chunk;
    uint32_t u32Copy;
    uint32_t u32Pad;
    uint32_t u32Primask;

    if (0UL == u32Len) {
        return LL_OK;
    }
    if ((pu8Data == NULL) || ((u32Addr + u32Len) > (HC_FLASH_END + 1UL))) {
        return LL_ERR_INVD_PARAM;
    }
    if (0UL != (u32Addr & 0x3UL)) {
        return LL_ERR_INVD_PARAM;   /* 地址须 4 字节对齐 */
    }
    if (u32Addr <= HC_FLASH_PROTECT_END) {
        return LL_ERR_INVD_PARAM;   /* 禁区：boot 代码 + IAP 标志扇区 */
    }

    while (SET != EFM_GetStatus(EFM_FLAG_RDY)) {
        /* 等待 EFM 空闲 */
    }
    /* 编程内核期进临界段：循环内 memcpy 走栈上 RAM 搬运缓冲(可安全执行)，
     * EFM_SequenceProgram 为 __RAM_FUNC 亦在 RAM 运行，故整段不会自取指阻塞；
     * 进临界段只为隔离任务切换/中断对 EFM 保护序列的插队（见文件头说明）。
     * 退出时按保存的 PRIMASK 原值恢复，不误开上层已关闭的中断。 */
    u32Primask = __get_PRIMASK();
    __disable_irq();
    HcFlash_Unlock();

    while ((u32Off < u32Len) && (LL_OK == i32Ret)) {
        u32Chunk = u32Len - u32Off;
        if (u32Chunk > HC_FLASH_BOUNCE_BYTES) {
            u32Chunk = HC_FLASH_BOUNCE_BYTES;
        }
        u32Copy = (u32Len - u32Off < u32Chunk) ? (u32Len - u32Off) : u32Chunk;
        u32Pad  = (((u32Copy + 3UL) & ~3UL) > u32Copy) ? (((u32Copy + 3UL) & ~3UL) - u32Copy) : 0UL;
        if (u32Copy > 0UL) {
            (void)memcpy(pu8Bounce, &pu8Data[u32Off], (size_t)u32Copy);
        }
        if (u32Pad > 0UL) {
            (void)memset(&pu8Bounce[u32Copy], 0xFF, (size_t)u32Pad);
        }

        /* 本块按 4 字节整数倍长度编程 */
        i32Ret = EFM_SequenceProgram(u32Addr + u32Off, pu8Bounce, u32Copy + u32Pad);
        u32Off += u32Copy + u32Pad;
    }

    HcFlash_Lock();
    if (0UL == (u32Primask & 1UL)) {
        __enable_irq();
    }
    return i32Ret;
}

/**
 * @brief  读 Flash（内部临时关闭 Cache，保证读到刚刚写入的内容）
 * @param  u32Addr  源地址
 * @param  pu8Data  输出缓冲
 * @param  u32Len   读取长度
 * @retval LL_OK / LL_ERR_INVD_PARAM
 * @note   接口说明见 hc_flash.h；读取不受禁区限制
 */
int32_t HcFlash_Read(uint32_t u32Addr, uint8_t *pu8Data, uint32_t u32Len)
{
    uint32_t u32CacheBak;

    if ((pu8Data == NULL) || (0UL == u32Len) ||
        ((u32Addr + u32Len) > (HC_FLASH_END + 1UL))) {
        return LL_ERR_INVD_PARAM;
    }

    /* 关闭 Flash Cache 后按字节直接读，保证读到刚刚写入的内容 */
    u32CacheBak = READ_REG32_BIT(CM_EFM->FRMC, EFM_CACHE_ALL);
    CLR_REG32_BIT(CM_EFM->FRMC, EFM_CACHE_ALL);
    (void)memcpy(pu8Data, (const void *)u32Addr, u32Len);
    MODIFY_REG32(CM_EFM->FRMC, EFM_CACHE_ALL, u32CacheBak);

    return LL_OK;
}

/**
 * @brief  回读并与给定数据逐字节比对（分块比对，不在栈上开大缓冲）
 * @param  u32Addr   比对起始地址
 * @param  pu8Data   期望数据
 * @param  u32Len    比对长度
 * @retval LL_OK 一致 / LL_ERR 不一致 / LL_ERR_INVD_PARAM 参数非法
 * @note   接口说明见 hc_flash.h
 */
int32_t HcFlash_Verify(uint32_t u32Addr, const uint8_t *pu8Data, uint32_t u32Len)
{
    uint32_t au32Bounce[HC_FLASH_BOUNCE_WORDS];
    uint8_t *pu8Bounce = (uint8_t *)(void *)au32Bounce;
    int32_t  i32Ret = LL_OK;
    uint32_t u32Off = 0UL;
    uint32_t u32Chunk;

    if (0UL == u32Len) {
        return LL_OK;
    }
    if ((pu8Data == NULL) || ((u32Addr + u32Len) > (HC_FLASH_END + 1UL))) {
        return LL_ERR_INVD_PARAM;
    }

    /* 分块回读比对，避免为校验而在栈上开大缓冲 */
    while ((u32Off < u32Len) && (LL_OK == i32Ret)) {
        u32Chunk = u32Len - u32Off;
        if (u32Chunk > HC_FLASH_BOUNCE_BYTES) {
            u32Chunk = HC_FLASH_BOUNCE_BYTES;
        }
        i32Ret = HcFlash_Read(u32Addr + u32Off, pu8Bounce, u32Chunk);
        if (LL_OK != i32Ret) {
            break;
        }
        if (0 != memcmp(pu8Bounce, &pu8Data[u32Off], (size_t)u32Chunk)) {
            i32Ret = LL_ERR;
            break;
        }
        u32Off += u32Chunk;
    }

    return i32Ret;
}

/*******************************************************************************
 * 标定区封装：参数为标定区内偏移，上层无需感知物理地址
 ******************************************************************************/
/**
 * @brief  擦除整个标定区（8KB 单扇区，0x0007E000）
 * @retval LL_OK / LL_ERR
 * @note   接口说明见 hc_flash.h；擦标定区不影响 IAP 升级标志
 */
int32_t HcFlash_CalibErase(void)
{
    return HcFlash_EraseSector(HC_FLASH_CALIB_ADDR);
}

/**
 * @brief  写标定区（参数为标定区内偏移，越界直接拒绝）
 * @param  u32Offset  标定区内偏移 0 ~ HC_FLASH_CALIB_SIZE
 * @param  pu8Data    源数据
 * @param  u32Len     数据长度
 * @retval LL_OK / LL_ERR_INVD_PARAM / EFM 编程错误码
 * @note   接口说明见 hc_flash.h；目标区域须已擦除
 */
int32_t HcFlash_CalibWrite(uint32_t u32Offset, const uint8_t *pu8Data, uint32_t u32Len)
{
    if ((u32Offset > HC_FLASH_CALIB_SIZE) ||
        (u32Len > (HC_FLASH_CALIB_SIZE - u32Offset))) {
        return LL_ERR_INVD_PARAM;
    }
    return HcFlash_Write(HC_FLASH_CALIB_ADDR + u32Offset, pu8Data, u32Len);
}

/**
 * @brief  读标定区（参数为标定区内偏移，越界直接拒绝）
 * @param  u32Offset  标定区内偏移
 * @param  pu8Data    输出缓冲
 * @param  u32Len     读取长度（须非 0）
 * @retval LL_OK / LL_ERR_INVD_PARAM
 * @note   接口说明见 hc_flash.h
 */
int32_t HcFlash_CalibRead(uint32_t u32Offset, uint8_t *pu8Data, uint32_t u32Len)
{
    if ((0UL == u32Len) ||
        (u32Offset > HC_FLASH_CALIB_SIZE) ||
        (u32Len > (HC_FLASH_CALIB_SIZE - u32Offset))) {
        return LL_ERR_INVD_PARAM;
    }
    return HcFlash_Read(HC_FLASH_CALIB_ADDR + u32Offset, pu8Data, u32Len);
}

/**
 * @brief  校验标定区指定偏移处的数据与给定数据一致
 * @param  u32Offset  标定区内偏移
 * @param  pu8Data    期望数据
 * @param  u32Len     比对长度（须非 0）
 * @retval LL_OK 一致 / LL_ERR 不一致 / LL_ERR_INVD_PARAM 参数非法
 * @note   接口说明见 hc_flash.h；标定表写后回读校验即走本接口
 */
int32_t HcFlash_CalibVerify(uint32_t u32Offset, const uint8_t *pu8Data, uint32_t u32Len)
{
    if ((0UL == u32Len) ||
        (u32Offset > HC_FLASH_CALIB_SIZE) ||
        (u32Len > (HC_FLASH_CALIB_SIZE - u32Offset))) {
        return LL_ERR_INVD_PARAM;
    }
    return HcFlash_Verify(HC_FLASH_CALIB_ADDR + u32Offset, pu8Data, u32Len);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
