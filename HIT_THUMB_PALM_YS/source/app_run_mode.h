/*******************************************************************************
 * @file  app_run_mode.h
 * @brief 运行模式（boot / app）的唯一声明处
 *
 * 本工程提供两个 MDK target，用于切换固件的链接基址与向量表基址：
 *   - HIT_TB_PM_APP       (target: app)      基址 0x00008000，配合 HIT_IAP_BOOT_UART
 *                                            bootloader 使用，支持 YMODEM IAP 升级
 *   - HIT_TB_PM_APP_BOOT  (target: app_boot) 基址 0x00000000，不带 bootloader 的独立
 *                                            直跑固件，便于 MDK 直接下载调试与单固件烧录
 *
 * 重要：
 *   1. 中断向量表基址**不由此处的宏决定**，而是由链接结果决定
 *      （main() 中取链接符 Image$$ER_IROM1$$Base 写入 SCB->VTOR）。
 *      模式宏仅用于可读性、启动自检与产物区分，避免"宏与 sct 不一致"导致中断跑飞。
 *   2. 新增 .c/.h 源文件时，需同时在两个 target 中加入（uVision 按 target 记录文件）。
 *   3. boot 模式下无 bootloader，且代码会占用 0x00006000~0x00007FFF 的 IAP 标志扇区，
 *      该模式下 IAP 升级功能不可用（代码照旧编译，不做条件编译）。
 ******************************************************************************/

#ifndef __APP_RUN_MODE_H__
#define __APP_RUN_MODE_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 运行模式选择：0 = app 模式(0x00008000)，1 = boot/直跑模式(0x00000000)
 * @note  由 MDK target 的 Define 传入：
 *          target app      -> 不定义（默认 0）
 *          target app_boot -> APP_RUN_IN_BOOT_MODE=1
 */
#ifndef APP_RUN_IN_BOOT_MODE
#define APP_RUN_IN_BOOT_MODE                (0)
#endif

/** @brief app 模式固件基址（bootloader 之后，IAP_BOOT_SIZE = 0x8000） */
#define APP_BASE_ADDR_APP                   (0x00008000UL)
/** @brief boot/直跑模式固件基址（Flash 起始） */
#define APP_BASE_ADDR_BOOT                  (0x00000000UL)

/** @brief 当前 target 期望的固件基址，仅用于启动自检（实际基址以 sct 链接结果为准） */
#define APP_EXPECTED_BASE                   ((APP_RUN_IN_BOOT_MODE) ? APP_BASE_ADDR_BOOT : APP_BASE_ADDR_APP)

#ifdef __cplusplus
}
#endif

#endif /* __APP_RUN_MODE_H__ */
