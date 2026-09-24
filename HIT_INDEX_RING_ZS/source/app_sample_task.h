#ifndef __APP_SAMPLE_TASK_H__
#define __APP_SAMPLE_TASK_H__

#include <stdint.h>
#include "adc_sample.h"

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t  xSampleTaskHandle;
/**
 * @brief  初始化并创建触觉采集任务（静态内存分配）
 * @retval 1=创建成功，0=创建失败
 */
uint8_t AppSampleTask_Init(void);

#endif /* __APP_SAMPLE_TASK_H__ */

