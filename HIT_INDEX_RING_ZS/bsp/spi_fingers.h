/**
 *******************************************************************************
 * @file  spi_fingers.h
 * @brief This file provides firmware functions to W25QXX group spi flash.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#ifndef __SPI_FINGERS_H__
#define __SPI_FINGERS_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll_def.h"
#include "hc32_ll.h"
#include "hc32_ll_spi.h"
#include "hc32_ll_aos.h"

#include <string.h>
#include "adc_sample.h"
#include "clock_cfg_bsp.h"


/*******************************************************************************
 * 协议宏定义（与主机完全一致）
 ******************************************************************************/
#define SPI_REQ_FRAME_LEN       10U
#define SPI_RESP_HEAD_LEN       10U
#define SPI_MAX_PAYLOAD_LEN     500U
#define SPI_MAX_FRAME_LEN       (SPI_RESP_HEAD_LEN + SPI_MAX_PAYLOAD_LEN)

#define SPI_REQ_HEADER          0xAAU       /* 主机请求帧头 */
#define SPI_RESP_HEADER         0x55U       /* 从机应答帧头 */

#define SPI_CMD_NULL					0x00 //心跳探测，查询从机在线状态
#define SPI_CMD_READ_TOUCH_DATA			0x01 //读取从机 触觉ADC 采集数据（核心业务命令）
#define SPI_CMD_CALIB_WRITE		0x02 //向从机写入标定参数
#define SPI_CMD_CALIB_READ		0x03 //读取从机当前标定参数
#define SPI_CMD_SAVE_PARAM		0x04 //将当前标定参数保存至从机 Flash
#define SPI_CMD_RESET_COMM		0x0E //复位从机通信状态机
#define SPI_CMD_SYS_RESET			0x0F //请求从机执行软件复位

#define SPI_STAT_OK						0x00 //执行成功，数据有效
#define SPI_STAT_CRC_ERR			0x01 //请求帧头部 CRC 校验失败
#define SPI_STAT_UNKNOWN_CMD	0x02 //无法识别的命令码
#define SPI_STAT_LEN_LIMIT		0x03 //请求读取长度超出 阈值字节上限
#define SPI_STAT_SLAVE_BUSY		0x04 //从机繁忙，无法响应本次请求
#define SPI_STAT_NO_DATA			0x05 //从机暂无可用的 ADC 采集数据
#define SPI_STAT_ALIGN_ERR		0x06 //16bit 模式下数据长度未按 2 字节对齐
#define SPI_STAT_CALIB_FAIL		0x07 //标定参数写入 / 保存操作失败

/*
位段					位宽		名称				说明
bit15 ~ bit3	13bit		ChSel				ADC 通道 / 采样分组选择

bit2					1bit		DataFormat	数据格式选择：
0 = 8bit 单字节模式
1 = 16bit 双字节模式

bit1 ~ bit0		2bit		Reserve			预留，置 0
*/
#define ADC_FMT_8BIT        (0U << 2)
#define ADC_FMT_16BIT       (1U << 2)
#define ADC_FMT_MASK        (1U << 2)
#pragma pack(1)
// 主机请求帧（固定10字节）
typedef struct
{
    uint8_t     header;         // 帧头 0xAA
    uint8_t     cmd;            // 命令码
    uint16_t    cmd_param;      // 命令参数
    uint16_t    rd_max_len;     // 最大读取字节数
    uint16_t    head_crc;       // 头部CRC
    uint16_t    tailer;        // 尾部
} spi_req_t;

// 从机应答头部（固定10字节：header+status+sta_param(应答序号)+data_len+head_crc+tailer）
typedef struct
{
    uint8_t     header;         // 帧头 0x55
    uint8_t     status;         // 状态码
		uint16_t    sta_param;      // 状态参数
    uint16_t    data_len;       // 载荷长度
    uint16_t    head_crc;       // 头部CRC
		uint16_t    tailer;        // 尾部
} spi_resp_head_t;
#pragma pack()

/* 接口函数 */
void Slave_SPI_Init(void);
int32_t Slave_SPI_LoadTxData(const uint8_t *pu8Data, uint16_t u16Len);
int32_t Slave_SPI_GetRxData(uint8_t *pu8RxBuf, uint16_t *pu16Len);
uint8_t Slave_SPI_IsRxComplete(void);
void Slave_SPI_ClearRxFlag(void);

uint16_t CRC16_CCITT(const uint8_t *pData, uint16_t len);
void spi_dma_rx_data_process(void);


#ifdef __cplusplus
}
#endif

#endif

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
