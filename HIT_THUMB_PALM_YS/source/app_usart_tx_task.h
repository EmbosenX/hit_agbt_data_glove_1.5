#ifndef __APP_USART_TX_TASK_H__
#define __APP_USART_TX_TASK_H__

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* 串口发送任务资源与优先级：
 *  - 栈 512U：本任务调用链极浅（xQueueReceive→USART_BSP_Send→等发送完成通知），
 *    且 1KB 出队帧缓冲为文件级 static（不落栈）；usart 任务取 1024U 是因其要
 *    容纳协议解析与组帧大缓冲，本任务不需要。
 *  - 优先级与 usart 任务同为最高应用档(configMAX_PRIORITIES-2)：TX_CPLT 中断唤醒
 *    本任务后须能立即抢占采样/手指任务(-3)继续发下一包，否则每包要多等一个时间
 *    片(1ms)造成帧间抖动。两任务同档但互斥阻塞：usart 阻塞在接收信号量、本任务
 *    阻塞在发送队列，同时就绪时按时间片轮转（usart 解析期间本任务最多延迟 1 个
 *    时间片，usart 多数时间在等信号量，实际几乎不冲突）。 */
#define USART_TX_TASK_STACK_SIZE   512U
#define USART_TX_TASK_PRIORITY     (configMAX_PRIORITIES - 2)

/* 发送任务句柄：TX_CPLT 发送完成中断经其投递任务通知（见 usart_bsp.c） */
extern TaskHandle_t  xUsartTxTaskHandle;

/**
 * @brief  初始化并创建USART发送任务（静态内存分配：发送队列 + 任务）
 * @retval 1=成功，0=失败
 * @note   须在 APP_Usart_TaskCreate() 之前调用：usart 任务解析路径随即会调用
 *         AppUsartTxTask_Send() 入队，队列未创建时入队一律返回 LL_ERR_BUSY 丢弃本帧。
 *         USART 硬件/接收链初始化在 APP_Usart_TaskCreate() 内完成，本函数不碰硬件。
 */
uint8_t APP_UsartTx_TaskCreate(void);

/**
 * @brief  应用层发送接口（线程安全，任意任务/解析路径可调用，非阻塞）
 * @param  pu8Data 待发送数据指针（整帧，如 0xAA 官方应答帧 / 3C 标定 ACK 帧）
 * @param  u16Len  数据长度（不得超过 USART_BSP_MAX_FRAME_LEN）
 * @retval LL_OK=本帧已拷入发送队列待发；LL_ERR_INVD_PARAM=参数错误；
 *         LL_ERR_BUSY=发送队列满(或未创建)，本帧被丢弃（调用方沿用"失败静默丢弃、
 *         不补发"语义，缺失包由请求方自行超时重查）
 * @note   整帧拷贝入静态发送队列后立即返回（非阻塞），实际发送由发送任务完成
 *         （阻塞取帧→USART_BSP_Send 启动 DMA→等 TX_CPLT 发送完成通知→下一帧），
 *         保证帧序与源缓冲生命周期受控。本函数不阻塞，可在协议解析路径安全调用。
 */
int32_t AppUsartTxTask_Send(const uint8_t *pu8Data, uint16_t u16Len);

/**
 * @brief  丢弃发送队列中全部待发帧
 * @note   用于 caliber 周期上传模式进入/退出时清场：进入时丢弃进入前残留的待发帧、
 *         退出时丢弃队列中尚未发出的 3C 数据帧，避免退出后继续吐旧数据。
 *         只清队列中尚未发出的帧，不影响正在线上发送中的那一帧。
 *         仅任务上下文调用（caliber 命令解析路径），**禁止在中断中调用**。
 */
void AppUsartTxTask_QueueReset(void);

/**
 * @brief  查询发送侧是否空闲（无待发帧且当前无帧在发送中）
 * @retval 1=空闲，0=有发送活动（有待发帧或某帧正在发送）
 * @note   供 usart 任务"本轮空闲才驱动一轮采集"的判定使用：
 *         无待发帧且无帧在发送中，才认为发送侧空闲。
 */
uint8_t AppUsartTxTask_IsIdle(void);

#endif /* __APP_USART_TX_TASK_H__ */
