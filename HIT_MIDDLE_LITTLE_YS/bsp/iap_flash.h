/**
 *******************************************************************************
 * @file    iap_flash.h
 * @brief   IAP 在线升级 Flash 驱动接口（主机/从机共用）
 *
 *          HC32F460 内部 512KB Flash 分区布局：
 *          +------------------+ 0x00000000
 *          | Bootloader 16KB  |  0x00000000 ~ 0x00003FFF
 *          +------------------+ 0x00004000
 *          | Application 240KB|  0x00004000 ~ 0x0003FFFF
 *          +------------------+ 0x00040000
 *          | 升级缓存区 248KB |  0x00040000 ~ 0x0007DFFF  (暂存新固件)
 *          +------------------+ 0x0007E000
 *          | 升级标志区 8KB   |  0x0007E000 ~ 0x0007FFFF  (升级描述符)
 *          +------------------+ 0x00080000
 *******************************************************************************
 */
#ifndef __IAP_FLASH_H__
#define __IAP_FLASH_H__

#include <stdint.h>
#include "hc32_ll_efm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ 分区布局 */
#define IAP_BOOT_ADDR           (0x00000000UL)      /* Bootloader 起始地址   */
#define IAP_BOOT_SIZE           (0x00004000UL)      /* Bootloader 16KB        */
#define IAP_APP_ADDR            (0x00004000UL)      /* Application 起始地址   */
#define IAP_APP_SIZE            (0x0003C000UL)      /* Application 240KB      */
#define IAP_CACHE_ADDR          (0x00040000UL)      /* 升级缓存区起始地址     */
#define IAP_CACHE_SIZE          (0x0003E000UL)      /* 升级缓存区 248KB       */
#define IAP_FLAG_ADDR           (0x0007E000UL)      /* 升级标志区起始地址     */
#define IAP_FLAG_SIZE           (0x00002000UL)      /* 升级标志区 8KB         */

#define IAP_FW_MAX_SIZE         (IAP_APP_SIZE)      /* 固件最大长度           */

/* 升级标志 magic 字 */
#define IAP_FLAG_MAGIC          (0x49415048UL)      /* "HAPI" (小端存储)      */

/* SPI 转发 / Flash 写缓存的分块大小（须为 4 的倍数） */
#define IAP_CHUNK_SIZE          (1024UL)

/* ------------------------------------------------------------------ 数据结构 */
/* 升级标志（写入 IAP_FLAG_ADDR 所在扇区，Bootloader 上电读取） */
typedef struct {
    uint32_t u32Magic;          /* 必须等于 IAP_FLAG_MAGIC */
    uint32_t u32FwSize;         /* 固件字节数 */
    uint32_t u32FwCrc;          /* 固件 CRC32 */
    uint32_t u32Reserved;
} iap_boot_flag_t;

/* IAP_VERIFY 应答载荷（从机返回其缓存区校验结果） */
typedef struct {
    uint32_t u32Crc;            /* 缓存区固件 CRC32 */
    uint32_t u32Size;           /* 缓存区固件字节数 */
} iap_verify_resp_t;

/* ------------------------------------------------------------------ API */
void      IapFlash_Init(void);

/* 擦除地址区域（地址与长度必须 8KB 扇区对齐） */
int32_t   IapFlash_EraseRegion(uint32_t u32Addr, uint32_t u32Size);

/* 写 Flash（地址必须 4 字节对齐，长度任意，尾部自动补 0xFF） */
int32_t   IapFlash_Write(uint32_t u32Addr, const uint8_t *pu8Data, uint32_t u32Len);

/* 读 Flash（内部关闭 Cache 以保证读到最新数据） */
int32_t   IapFlash_Read(uint32_t u32Addr, uint8_t *pu8Data, uint32_t u32Len);

/* CRC32 (IEEE 802.3, poly 0xEDB88320) 增量接口 */
uint32_t  IapFlash_CRC32_Init(void);
uint32_t  IapFlash_CRC32_Update(uint32_t u32Crc, const uint8_t *pu8Data, uint32_t u32Len);
uint32_t  IapFlash_CRC32_Final(uint32_t u32Crc);
uint32_t  IapFlash_CRC32(const uint8_t *pu8Data, uint32_t u32Len);
/* 计算 Flash 地址区间的 CRC32 */
uint32_t  IapFlash_CRC32Range(uint32_t u32Addr, uint32_t u32Len);

/* 升级标志读写 */
int32_t   IapFlash_WriteBootFlag(uint32_t u32FwSize, uint32_t u32FwCrc);
int32_t   IapFlash_ReadBootFlag(iap_boot_flag_t *pstFlag);
int32_t   IapFlash_ClearBootFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* __IAP_FLASH_H__ */
