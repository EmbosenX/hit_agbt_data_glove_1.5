#ifndef __APP_IAP_TASK_H__
#define __APP_IAP_TASK_H__

#include <stdint.h>

#define IAP_FW_BUF_SIZE     256U

/* IAP全局缓冲区，对外暴露供SPI任务写入固件数据 */
extern uint8_t  g_u8IapFwBuf[IAP_FW_BUF_SIZE];
extern uint16_t g_u16IapFwLen;
extern uint32_t g_u32IapFwAddr;

/**
 * @brief  初始化并创建IAP升级任务（静态内存分配）
 * @retval 1=创建成功，0=创建失败
 */
uint8_t AppIapTask_Init(void);

/**
 * @brief  向IAP任务发送升级通知
 */
void AppIapTask_NotifyUpgrade(void);

#endif /* __APP_IAP_TASK_H__ */


