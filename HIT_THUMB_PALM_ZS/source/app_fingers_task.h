/*******************************************************************************
 * @file        app_fingers_task.h
 * @brief       触觉手指业务任务头文件
 * @author      
 * @date        2026-07-21
 * @version     V1.0
 * @note        HC32F460 + FreeRTOS 多手指触觉采集任务
 ******************************************************************************/
#ifndef __APP_FINGERS_TASK_H__
#define __APP_FINGERS_TASK_H__

#include "hc32_ll.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"



/* 手指采集任务资源：栈 1024U（含两从机 SPI 收发缓冲与帧解析）；
   优先级比采样任务低 1 档(configMAX_PRIORITIES-3)，仅被采样任务通知唤醒 */
#define FINGER_TASK_STACK_SIZE    1024U
#define FINGER_TASK_PRIORITY      (configMAX_PRIORITIES - 3)

/* 外部全局句柄声明 */
extern TaskHandle_t xAppFingersTaskHandle;

/**
 * @brief  创建手指采集任务（静态内存分配，不启动采集）
 * @note   须在调度器启动前调用；任务被采样任务通知后才执行一轮 SPI 采集，
 *         完成后经 m_xFingerDoneSem 回报（该信号量由 APP_Sample_TaskCreate 创建）
 */
void APP_Fingers_TaskCreate(void);
uint8_t Fingers_IsLastRoundValid(void);

#endif /* __APP_FINGERS_TASK_H__ */


