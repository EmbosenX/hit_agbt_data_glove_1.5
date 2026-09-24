/**
 *******************************************************************************
 * @file    hc_flash.h
 * @brief   片内 EFM Flash 驱动（通用擦写读 + 标定表掉电存储）
 *
 *          HC32F460 内部 512KB Flash 布局（与 HIT_IAP_BOOT_UART 的 IAP 布局对齐）：
 *
 *          +---------------------+ 0x00000000
 *          | Bootloader 代码     |  0x00000000 ~ 0x00005FFF  (boot 工程，禁区)
 *          +---------------------+ 0x00006000
 *          | IAP 标志扇区        |  0x00006000 ~ 0x00007FFF  (0x7FF4/0x7FF8/0x7FFC，禁区)
 *          +---------------------+ 0x00008000
 *          | Application         |  0x00008000 ~ 0x0007DFFF  (472KB，见 sct 0x00076000)
 *          +---------------------+ 0x0007E000
 *          | 标定区 8KB          |  0x0007E000 ~ 0x0007FFFF  (本模块独占)
 *          +---------------------+ 0x00080000
 *
 *          地址来源说明：
 *          - boot 区大小 0x8000 取自 boot 工程 hc32f4xx_conf.h 的 IAP_BOOT_SIZE；
 *          - IAP 标志地址 0x00007FF4/0x00007FF8/0x00007FFC 与本工程
 *            source/app_iap.h 的 APP_IAP_RUN_FLAG_ADDR/APP_IAP_UPGRADE_FLAG_ADDR 一致；
 *          - 0x7FF8 的扇区基址 = 0x7FF8 & ~(0x2000-1) = 0x00006000，故 IAP 标志
 *            落在 boot 区内而非末扇区，末扇区 0x7E000 完全空闲、可安全用作标定区
 *            （擦标定区不会影响 IAP 升级标志）。
 *
 *          实现要点：编程用 EFM_SequenceProgram(__RAM_FUNC 运行于 RAM)；
 *          擦除用 EFM_SectorErase（代码所在扇区与擦除目标扇区不重叠时可跨扇区
 *          执行，标定区 0x7E000 与 APP 0x8000~0x7DFFF 不重叠，满足该前提）；
 *          读前关闭 Flash Cache 以保证回读到最新数据。
 *          擦写期间总线状态保持 HOLD（EFM_SetBusStatus）：EFM_SectorErase 在 Flash
 *          内运行，20~40ms 擦除期间 CPU 仍从 Flash 取指，HOLD 使其被硬件停顿；
 *          擦写内核期用 __disable_irq 全屏蔽中断，退出时按 PRIMASK 原值恢复。
 *
 *          【禁区保护】0x00000000 ~ 0x00007FFF（boot 代码 + IAP 标志扇区）一律
 *          拒绝擦写，返回 LL_ERR_INVD_PARAM，防止误操作导致设备变砖。
 *
 *          【容量边界】按《上传与标定协议使用说明.md》PRESS_LUT_MAX_SIZE=62、
 *          rows=2、最多 6 部位 × 3 区域估算，全量 LUT 约 6×3×2×62×2B ≈ 4.5KB，
 *          8KB 标定区可容纳。若后续增行或增点数致容量不足，只能压缩点数或
 *          改用 APP 区尾部（届时须同步收缩 sct 的 0x00076000）。
 *******************************************************************************
 */
#ifndef __HC_FLASH_H__
#define __HC_FLASH_H__

#include <stdint.h>
#include "hc32_ll_efm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ 地址布局 */
#define HC_FLASH_BASE               (0x00000000UL)      /* Flash 起始地址        */
#define HC_FLASH_END                (0x0007FFFFUL)      /* Flash 结束地址        */
#define HC_FLASH_SECTOR_SIZE        (0x00002000UL)      /* 扇区大小 8KB          */

/* IAP 分区：来源 boot 工程 hc32f4xx_conf.h 的 IAP_BOOT_SIZE(0x8000)，
 * 与本工程 source/app_iap.h 的标志地址严格一致，切勿单独修改本处数值 */
#define HC_FLASH_BOOT_SIZE          (0x00008000UL)      /* boot 区大小 32KB      */
#define HC_FLASH_APP_ADDR           (0x00008000UL)      /* APP 起始地址          */
#define HC_FLASH_APP_SIZE           (0x00076000UL)      /* APP 区大小 472KB      */
#define HC_FLASH_APP_END            (0x0007DFFFUL)      /* APP 结束地址          */

/* IAP 标志扇区：擦除该扇区会连带清掉 0x7FF4/0x7FF8/0x7FFC，破坏 boot 升级判定 */
#define HC_FLASH_IAP_FLAG_SECTOR    (0x00006000UL)
/* 禁区上界：地址 <= 该值一律拒绝擦写（boot 代码 + IAP 标志扇区） */
#define HC_FLASH_PROTECT_END        (0x00007FFFUL)

#define HC_FLASH_CALIB_ADDR         (0x0007E000UL)      /* 标定区基址（末扇区）  */
#define HC_FLASH_CALIB_SIZE         (0x00002000UL)      /* 标定区大小 8KB        */

/* 写入搬运缓冲：EFM_SequenceProgram 要求源地址 4 字节对齐，故按块对齐搬运 */
#define HC_FLASH_BOUNCE_WORDS       (32U)                             /* 缓冲字数 */
#define HC_FLASH_BOUNCE_BYTES       (HC_FLASH_BOUNCE_WORDS * 4U)       /* 128B */

/* ------------------------------------------------------------------ API */
/**
 * @brief  初始化 EFM（使能 FWMC、解锁保护、设置擦写期间保持总线、设置工作模式）
 * @note   上电后调用一次即可；之后 HcFlash_Write/Erase 内部自行加解锁保护。
 *         本函数无临界段，可在调度器启动前（main 早期）安全调用。
 */
void    HcFlash_Init(void);

/**
 * @brief  擦除单个扇区
 * @param  u32Addr  扇区地址（须 8KB 对齐，且不得落在禁区 0x0000~0x7FFF）
 * @retval LL_OK / LL_ERR_INVD_PARAM / LL_ERR
 * @note   擦除约 20~40ms，期间内部用 __disable_irq 全屏蔽中断、退出时按 PRIMASK
 *         原值恢复，不依赖 FreeRTOS 内核，可在调度器启动前调用。
 */
int32_t HcFlash_EraseSector(uint32_t u32Addr);

/**
 * @brief  擦除地址区间（地址与长度均须 8KB 扇区对齐，且不得落入禁区）
 */
int32_t HcFlash_EraseRegion(uint32_t u32Addr, uint32_t u32Size);

/**
 * @brief  写 Flash
 * @param  u32Addr   目标地址（须 4 字节对齐，且不得落在禁区 0x0000~0x7FFF）
 * @param  pu8Data   源数据
 * @param  u32Len    数据长度（任意，尾部不足 4 字节自动补 0xFF）
 * @note   目标区域须已擦除（写入为非 1 覆盖，未擦除会写入失败）。
 *         内部用 __disable_irq 全屏蔽中断并恢复 PRIMASK，不依赖 FreeRTOS 内核。
 */
int32_t HcFlash_Write(uint32_t u32Addr, const uint8_t *pu8Data, uint32_t u32Len);

/**
 * @brief  读 Flash（内部临时关闭 Cache，保证读到最新写入内容）
 */
int32_t HcFlash_Read(uint32_t u32Addr, uint8_t *pu8Data, uint32_t u32Len);

/**
 * @brief  回读并与给定数据逐字节比对，校验写入结果
 * @retval LL_OK 一致 / LL_ERR 不一致 / LL_ERR_INVD_PARAM 参数非法
 */
int32_t HcFlash_Verify(uint32_t u32Addr, const uint8_t *pu8Data, uint32_t u32Len);

/* ------------------------------------------------- 标定区封装（区内偏移） */
/**
 * @brief  擦除整个标定区（8KB）
 */
int32_t HcFlash_CalibErase(void);

/**
 * @brief  写标定区
 * @param  u32Offset  标定区内偏移（0 ~ HC_FLASH_CALIB_SIZE）
 * @param  pu8Data    源数据
 * @param  u32Len     数据长度
 * @note   目标区域须已擦除；上层保存逻辑应先 HcFlash_CalibErase 再写。
 */
int32_t HcFlash_CalibWrite(uint32_t u32Offset, const uint8_t *pu8Data, uint32_t u32Len);

/**
 * @brief  读标定区
 * @param  u32Offset  标定区内偏移（0 ~ HC_FLASH_CALIB_SIZE）
 */
int32_t HcFlash_CalibRead(uint32_t u32Offset, uint8_t *pu8Data, uint32_t u32Len);

/**
 * @brief  校验标定区指定偏移处的数据与给定数据一致
 */
int32_t HcFlash_CalibVerify(uint32_t u32Offset, const uint8_t *pu8Data, uint32_t u32Len);

#ifdef __cplusplus
}
#endif

#endif /* __HC_FLASH_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
