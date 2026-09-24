#ifndef __APP_FINGER_TASK_H__
#define __APP_FINGER_TASK_H__

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/**
 * @brief  初始化并创建SPI通讯任务（静态内存分配）
 * @retval 1=创建成功，0=创建失败
 */
uint8_t AppFingerTask_Init(void);

/**
 * @brief  通知SPI任务接收完成（任务上下文调用）
 */
void AppFingerTask_NotifyRx(void);

/**
 * @brief  通知SPI任务接收完成（中断上下文调用）
 */
void AppFingerTask_NotifyRxFromISR(void);

extern SemaphoreHandle_t m_xRxSem;


#endif /* __APP_FINGER_TASK_H__ */


