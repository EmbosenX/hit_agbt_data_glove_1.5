/*******************************************************************************
 * @file        app_sample_task.h
 * @brief       多通道传感器ADC采集业务任务，FreeRTOS静态创建无堆分配
 * @author      
 * @date        2026-07-21
 * @version     V1.0
 * @note        适配触觉/手指多路ADC采集：共享缓存由 m_hSampleDataMutex 保护，
 *              跨任务读取走 TacConfig 发布快照（免锁），fingers 完成回报走专用信号量
 ******************************************************************************/
#ifndef __APP_SAMPLE_TASK_H__
#define __APP_SAMPLE_TASK_H__

#include "hc32_ll.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "adc_sample.h"
#include "main.h"
#include "app_tac_config.h"

/* -------------------------- 任务配置宏 -------------------------- */
#define SAMPLE_TASK_STACK_SIZE      768U
#define SAMPLE_TASK_PRIORITY        (configMAX_PRIORITIES - 3)
#define SAMPLE_MSG_QUEUE_DEPTH      10U
#define SAMPLE_PERIOD_MS            10U    /* 采集节拍周期 10ms/轮：由 usart 发令或周期定时器驱动，本任务被动唤醒 */

/* 采集整轮耗时调试打印开关：1=开启（每轮结束打印本轮耗时，含手指任务采集+本地ADC/标定/滑移，
   时间源 xTaskGetTickCount()，分辨率=1 个 Tick=1ms，耗时不足 1ms 时显示 0）；0=关闭（默认，
   不产生任何代码与运行时开销）。调试期临时使用，正式版本保持 0

   默认按打印通道自动跟随 LL_PRINT_ENABLE：本开关的产物只有 DDL_Printf，而
   LL_PRINT_ENABLE=DDL_OFF 时 DDL_Printf 是空的变参函数（无任何输出），此时置 1
   只会白跑一遍耗时计算，故自动置 0。需要调试时把 LL_PRINT_ENABLE 改为 DDL_ON
   即可，也可用编译器 -DSAMPLE_LOOP_TIME_DEBUG=... 覆盖本默认值 */
#ifndef SAMPLE_LOOP_TIME_DEBUG
#if (LL_PRINT_ENABLE == DDL_ON)
#define SAMPLE_LOOP_TIME_DEBUG      1U
#else
#define SAMPLE_LOOP_TIME_DEBUG      1U
#endif
#endif

/* -------------------------- 采集缓存配置 -------------------------- */
#define SAMPLE_PLAT_NUM             (PALM_ROW*PALM_COL*DATA_FRAME_BYTE_NUM)     // 采集通道总数（手指触觉）

/* -------------------------- 对外全局句柄与状态声明 -------------------------- */
extern TaskHandle_t        m_hSampleTask;
extern SemaphoreHandle_t   m_hSampleDataMutex;

/* fingers→sample「本轮采集已完成」专用握手信号量（Fingers 任务 give，Sample 任务 take）。
 * 必须与 m_hSampleTask 通知通道分开：该通道被 usart 任务（空闲节拍）独占用于「采集发令」，
 * 共用同一 32bit 通知计数会让发令提前满足采样任务对 fingers「本轮完成」的等待，使采样任务
 * 在 fingers 尚未写完 cfg 手指区时就整轮处理（读到上一轮旧手指数据），破坏严格乒乓节拍。 */
extern SemaphoreHandle_t   m_xFingerDoneSem;


/* -------------------------- 对外业务API -------------------------- */
void APP_Sample_TaskCreate(void);

#if TAC_CALIB_EN
/* 重新零点控制接口（供官方协议 0x30/0x11 get_calib 触发零点重新快照） */
void    Sample_RequestRecalib(void);            /* 请求重新零点校准 */
uint8_t Sample_IsRecalibRequested(void);        /* 查询是否有待处理的零点校准请求 */
void    Sample_ClearRecalibRequest(void);       /* 清除零点校准请求标志 */
#endif /* TAC_CALIB_EN */

#endif /* __APP_SAMPLE_TASK_H__ */


