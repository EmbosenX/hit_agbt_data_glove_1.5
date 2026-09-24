/**
 * @file  iap_boot.h
 * @brief IAP Bootloader 逻辑接口（主/从机共用）
 */
#ifndef __IAP_BOOT_H__
#define __IAP_BOOT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bootloader 主逻辑：检查升级标志并搬运固件，随后跳转 App，永不返回 */
void     IapBoot_Check(void);

/* 判断 App 区是否存在有效程序 */
uint32_t IapBoot_IsAppValid(void);

/* 跳转到指定地址的应用程序（设置 MSP/VTOR/PC） */
void     Iap_JumpToApp(uint32_t u32Addr);

/* App 复位，进入 Bootloader（升级触发入口） */
void     Iap_JumpToBoot(void);

#ifdef __cplusplus
}
#endif

#endif /* __IAP_BOOT_H__ */
