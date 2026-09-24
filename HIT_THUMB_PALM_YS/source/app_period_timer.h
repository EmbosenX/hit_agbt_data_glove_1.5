/*******************************************************************************
 * @file        app_period_timer.h
 * @brief       周期节拍模块：以 FreeRTOS 软件定时器驱动"周期产帧"节拍
 * @date        2026-09-16
 * @version     V1.0
 * @note        本模块以独立软件定时器承载两个周期节拍：caliber 10ms 周期上传与
 *              官方协议 125Hz 周期上报；usart 任务只保留"接收调度 + 命令解析"职责。
 *
 * 设计要点：
 *  - 两个自动重载定时器（8ms 周期上报 / 10ms caliber），静态创建无堆分配；
 *  - 回调运行在**定时器守护任务(优先级 configMAX_PRIORITIES-1)**上下文，会抢占
 *    usart(30)/sample(29) 任务，故回调内禁止阻塞，且不得与 usart 侧共用组帧缓冲
 *    （缓冲已按上下文拆分，见 app_official_proto.c / app_usart_task.c 注释）；
 *  - 启停为"命令 + 状态双保险"：定时器命令可能因守护任务队列满而丢失，故回调内
 *    一律再判一次开关/互斥状态，保证"关了就一定不发"。
 *
 * 调用约束：
 *  - APP_PeriodTimer_Init()：调度器启动前调用（创建定时器，不启动）；
 *  - AppPeriodTimer_Start()：调度器运行后、usart 任务首轮调用（按当前开关启动）；
 *  - 其余启停接口在 usart 任务上下文调用（0x15 开关 / caliber start-stop / 升级）。
 ******************************************************************************/
#ifndef __APP_PERIOD_TIMER_H__
#define __APP_PERIOD_TIMER_H__

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief  创建周期节拍定时器（静态内存分配，不启动任何定时器）
 * @retval 1=成功，0=失败
 * @note   须在 vTaskStartScheduler() 之前、APP_Usart_TaskCreate() 之前调用：
 *         定时器对象先于使用方就绪；实际启动由 AppPeriodTimer_Start() 完成。
 */
uint8_t APP_PeriodTimer_Init(void);

/**
 * @brief  按当前开关状态启动周期上报定时器（调度器运行后调用）
 * @note   由 usart 任务主体首轮调用一次（此时守护任务已就绪，xTimerStart 命令
 *         必定被处理）；定时器命令在调度器启动前不入队，故启动不能放在 Init 内。
 */
void AppPeriodTimer_Start(void);

/**
 * @brief  周期上报开关变更（0x30/0x15 与升级路径收口）
 * @param  u8En  1=开启周期上报，0=关闭
 * @note   caliber 周期上传期间即使开启也不启动上报定时器（互斥，避免争抢发送队列
 *         并污染标定数据）；回调内仍会再判状态，命令丢失也能保证停发。
 */
void AppPeriodTimer_SetCycleReportEn(uint8_t u8En);

/**
 * @brief  进入 caliber 周期上传：启动 10ms 上传定时器并停掉周期上报
 */
void AppPeriodTimer_CaliberStart(void);

/**
 * @brief  退出 caliber 周期上传：停掉上传定时器，按需恢复周期上报
 */
void AppPeriodTimer_CaliberStop(void);

/**
 * @brief  查询最近一次"节拍驱动采集"的时刻
 * @retval 最近的驱动时刻(xTaskGetTickCount 值)
 * @note   供 usart 任务空闲驱动判定使用：以该时间戳判断"刚驱动过采集"，
 *         避免定时器刚驱动完采集又立刻补驱动一轮。
 */
TickType_t AppPeriodTimer_GetLastDriveTick(void);

#endif /* __APP_PERIOD_TIMER_H__ */
