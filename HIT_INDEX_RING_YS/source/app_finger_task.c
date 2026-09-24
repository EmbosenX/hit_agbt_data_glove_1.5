#include "app_finger_task.h"
#include "main.h"
#include "spi_fingers.h"
#include "app_tac_config.h"
#include "app_sample_task.h"

/******************************************************************************
 * 配置宏
 ******************************************************************************/
/* 无 SPI 事件时 xSemaphoreTake 的阻塞等待时长（tick，1 tick = 1ms）。
   非 0 值让本任务在无事可做时主动让出 CPU，供 SampleTask（优先级更低）连续采集；
   信号量一旦可用会被 DMA 完成中断立即唤醒并返回，不损失 SPI 响应性。
   置 0 则退化为完全非阻塞空转：本任务常驻占用 CPU，SampleTask 只能靠时间片
   轮转获得执行时间，且每个空转周期都会 xTaskNotifyGive 一次导致通知值暴涨。 */
#ifndef SPI_RX_WAIT_TICK
#define SPI_RX_WAIT_TICK            (5U)
#endif

/******************************************************************************
 * 静态任务资源（仅本文件内可见）
 ******************************************************************************/
static StaticTask_t  xSpiTaskTcb;
static StackType_t   uxSpiTaskStack[768];
static TaskHandle_t  xSpiTaskHandle = NULL;

/* 静态二值信号量：SPI接收完成通知 */
static StaticSemaphore_t m_xRxSemBuffer;
SemaphoreHandle_t m_xRxSem = NULL;

/******************************************************************************
 * 内部任务函数声明
 ******************************************************************************/
static void AppTaskSpiComm(void *pvParameters);

/******************************************************************************
 * 对外接口函数
 ******************************************************************************/

uint8_t AppFingerTask_Init(void)
{
		Slave_SPI_Init();

    /* 创建"接收完成"信号量：SPI 接收中断里直接给该句柄 */
    m_xRxSem = xSemaphoreCreateBinaryStatic(&m_xRxSemBuffer);
    if (m_xRxSem == NULL) {
        return 0U;
    }

    /* 创建静态任务 */
    xSpiTaskHandle = xTaskCreateStatic(
        AppTaskSpiComm,
        "SpiCommTask",
        768U,
        NULL,
        9U,
        uxSpiTaskStack,
        &xSpiTaskTcb
    );

    if (xSpiTaskHandle != NULL) {
        DDL_Printf("SpiCommTask success!\r\n");
        return 1U;
    }
    return 0U;
}

void AppFingerTask_NotifyRx(void)
{
    if (m_xRxSem != NULL) {
        (void)xSemaphoreGive(m_xRxSem);
    }
}

void AppFingerTask_NotifyRxFromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* 初始化窗口内 m_xRxSem 可能仍为 NULL：Slave_SPI_Init() 早于信号量创建，
       而 CS 中断在 Slave_SPI_Init() 内已使能，主机可能在信号量创建前就发起事务。
       直接给裸 NULL 句柄会命中 configASSERT 死循环导致从机永久僵死，故必须判空 */
    if (m_xRxSem != NULL) {
        (void)xSemaphoreGiveFromISR(m_xRxSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/******************************************************************************
 * 任务主体：SPI通讯业务逻辑
 ******************************************************************************/
static void AppTaskSpiComm(void *pvParameters)
{
    static uint8_t u8Preload[SPI_MAX_FRAME_LEN];
    uint16_t u16PreCrc;

    (void)pvParameters;

    /* 上电先预装一帧"空载荷 OK 应答"：保证主控首拍事务不会读到全 0 帧 */
    memset(u8Preload, 0x00, sizeof(u8Preload));
    u8Preload[0] = SPI_RESP_HEADER;
    u8Preload[1] = SPI_STAT_OK;
    u16PreCrc    = CRC16_CCITT(u8Preload, 6U);
    u8Preload[6] = (uint8_t)((u16PreCrc >> 8U) & 0xFFU);
    u8Preload[7] = (uint8_t)(u16PreCrc & 0xFFU);
    u8Preload[8] = 0x0DU;
    u8Preload[9] = 0x0DU;
    (void)Slave_SPI_LoadTxData(u8Preload, SPI_RESP_HEAD_LEN);

    for (;;) 
		{
        /* 等待"传输完成"信号量：超时为 SPI_RX_WAIT_TICK，无事件时阻塞让出 CPU 供
           SampleTask（优先级 8，低于本任务的 9）连续采集；SPI 帧到达时由 DMA 完成
           中断给出信号量并立即唤醒，不损失响应性 */
        if (xSemaphoreTake(m_xRxSem, SPI_RX_WAIT_TICK) == pdTRUE) 
				{
            /* 处理SPI DMA接收数据、解析协议帧、执行业务逻辑 */
            spi_dma_rx_data_process();
        }
				else{
						/* 本轮无 SPI 事件：驱动采样任务采集一轮。因阻塞等待的存在，
						   该通知每个等待周期最多触发一次，不会导致通知值暴涨 */
						if (xSampleTaskHandle != NULL) {
								(void)xTaskNotifyGive(xSampleTaskHandle);
						}
				}
    }
}


