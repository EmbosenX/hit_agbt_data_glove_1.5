/**
 *******************************************************************************
 * @file  spi_fingers.h
 * @brief This file provides firmware functions to fingers group spi.
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
#include "clock_cfg_bsp.h"
#include <string.h>
#include "hc32_ll_spi.h"
#include "hc32_ll_aos.h"

/*******************************************************************************
 * 协议宏定义（与从机完全一致）
 ******************************************************************************/
#define SPI_REQ_FRAME_LEN       10U
#define SPI_RESP_HEAD_LEN       10U
#define SPI_MAX_PAYLOAD_LEN     500U
#define SPI_MAX_FRAME_LEN       (SPI_RESP_HEAD_LEN + SPI_MAX_PAYLOAD_LEN)

#define SPI_REQ_HEADER          0xAAU
#define SPI_RESP_HEADER         0x55U

/* 命令码 */
#define SPI_CMD_NULL            0x00U
#define SPI_CMD_READ_DATA       0x01U
#define SPI_CMD_CALIB_WRITE     0x02U
#define SPI_CMD_CALIB_READ      0x03U
#define SPI_CMD_SAVE_PARAM      0x04U
#define SPI_CMD_RESET_COMM      0x0EU
#define SPI_CMD_SYS_RESET       0x0FU

/* 状态码 */
#define SPI_STAT_OK             0x00U
#define SPI_STAT_CRC_ERR        0x01U
#define SPI_STAT_UNKNOWN_CMD    0x02U
#define SPI_STAT_LEN_LIMIT      0x03U
#define SPI_STAT_SLAVE_BUSY     0x04U
#define SPI_STAT_NO_DATA        0x05U
#define SPI_STAT_ALIGN_ERR      0x06U
#define SPI_STAT_CALIB_FAIL     0x07U

/* 错误码 */
#define SPI_OK                  0
#define SPI_ERR_PARAM           -1
#define SPI_ERR_TIMEOUT         -2
#define SPI_ERR_FRAME           -3

/* ADC格式选择 */
#define ADC_FMT_8BIT            (0U << 2)
#define ADC_FMT_16BIT           (1U << 2)
#define ADC_FMT_MASK            (1U << 2)
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

// 从机应答头部（固定10字节：header+status+sta_param(序号)+data_len+head_crc+tailer）
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

/*******************************************************************************
 * 接口函数
 ******************************************************************************/
void     Fingers_SPI_Config(void);
/**
 * @brief  SPI 标准事务接口（组请求头+CRC -> 全双工收发 -> 应答校验）
 * @retval SPI_OK / SPI_ERR_*（见文件头错误码定义）
 */
int32_t  Fingers_SPI_Transact(uint8_t u8SlaveIdx, uint8_t u8Cmd, uint16_t u16CmdParam,
                              uint16_t u16RdMaxLen, uint8_t *pu8RespPayload,
                              uint16_t *pu16RespLen, uint8_t *pu8Status);




#ifdef __cplusplus
}
#endif

#endif

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
