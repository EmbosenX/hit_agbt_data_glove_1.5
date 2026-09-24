#ifndef __APP_USART_TASK_H__
#define __APP_USART_TASK_H__

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define USART_TASK_STACK_SIZE      1024U
/* 串口收发已队列/信号量化：
 * 接收=接收完成信号量唤醒、任务内攒帧/解析；发送=应用层发送队列由独立发送任务
 * app_usart_tx_task 承担（阻塞取帧→USART_BSP_Send→等发送完成(TX_CPLT)通知，
 * 见 app_usart_tx_task.h），本任务只负责入队。
 * 本任务为最高应用优先级(比采样/手指任务高 2 档，仅低于 FreeRTOS 定时器守护任务)，
 * 仅在等接收信号量(空闲节拍上限 5ms)时阻塞让出 CPU；发送任务同为本档优先级，
 * 保证 TX_CPLT 唤醒后能立即抢占采样任务继续发下一包，否则每包要多等一个时间片
 * (1ms)，在 120Hz(8.3ms)查询周期下不可接受） */
#define USART_TASK_PRIORITY        (configMAX_PRIORITIES - 2)

/* caliber 周期上传周期(ms)：对齐 SAMPLE_PERIOD_MS。
 * 节拍由 app_period_timer 的软件定时器驱动，宏提至头文件供节拍模块引用 */
#define USART_CALIBER_UPLOAD_PERIOD_MS  (10U)

extern TaskHandle_t  xUsartTaskHandle;
extern SemaphoreHandle_t m_xRxSem;
/**
 * @brief  初始化并创建USART应用任务（静态内存分配）
 * @retval 1=成功，0=失败
 */
uint8_t APP_Usart_TaskCreate(void);

/**
 * @brief  查询 caliber 周期上传模式是否使能
 * @retval 1=周期上传中，0=未使能
 * @note   跨上下文访问接口：usart 任务写(caliber start/stop)、定时器守护任务回调读
 *         （10ms 上传节拍与 8ms 周期上报互斥判定），标志为 volatile 单字节。
 */
uint8_t AppUsart_IsCaliberUploadEn(void);

/**
 * @brief  一轮 caliber 周期上传：按 拇指→食指→中指→无名指→小指→掌心 顺序组帧并入队
 * @param  pu8FrameBuf  组帧工作缓冲（至少 USART_BSP_MAX_FRAME_LEN 字节，由调用方提供）
 * @retval 成功入队的帧数
 * @note   由 app_period_timer 的 10ms 定时器回调调用（守护任务上下文），故组帧缓冲
 *         必须由调用方提供且与 usart 任务侧互不共享，避免高优先级回调抢占后改写
 *         本任务正在使用的帧缓冲。
 *         入队走 AppUsartTxTask_Send（非阻塞、线程安全），由 app_usart_tx_task 发送
 *         任务统一发出；队列满沿用"失败静默丢弃、不补发"语义。
 */
uint16_t AppUsart_UploadCaliberFrames(uint8_t *pu8FrameBuf);

/* 发送接口见 app_usart_tx_task.h：AppUsartTxTask_Send / AppUsartTxTask_QueueReset /
   AppUsartTxTask_IsIdle；本任务只负责接收解析并入队 */

#endif /* __APP_USART_TASK_H__ */
