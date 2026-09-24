/*******************************************************************************
 * @file        app_official_proto.h
 * @brief       官方《华威科-智元1.5T触觉数据手套通讯协议》实现模块
 * @author
 * @date        2026-08-28
 * @version     V1.0
 * @note        请求-应答模式，主控以"触觉传感器(0x30)"身份响应上位机查询；
 *              与既有 3C 3C 标定协议通过帧头隔离共存。
 *
 * 帧格式（总帧长 = len + 2，crc16 不参与 len 计算）：
 *   -----------------------------------------------------------------
 *   | 偏移 | 字节数 | 字段             | 说明                        |
 *   -----------------------------------------------------------------
 *   | 0    | 1     | sof              | 帧头，固定 0xAA              |
 *   | 1    | 1     | ver              | 协议版本 [7:5]主 [4:2]次    |
 *   | 2    | 1     | len              | 头部+数据段长度 = 12+data_len |
 *   | 3    | 1     | crc8             | 对 sof~len 共 3 字节校验     |
 *   | 4    | 1     | sender_module_id | 发送方模块 id                |
 *   | 5    | 1     | sender_sub_id    | 发送方子模块 id              |
 *   | 6    | 1     | receiver_module_id | 接收方模块 id              |
 *   | 7    | 1     | receiver_sub_id  | 接收方子模块 id              |
 *   | 8    | 1     | cmd_set          | 指令集（通用/触觉）          |
 *   | 9    | 1     | cmd_id           | 指令 id                      |
 *   | 10   | 1     | seq_num          | 包序号（应答回填请求序号）   |
 *   | 11   | 1     | cmd_type         | 指令类型（无应答/请求/应答） |
 *   | 12   | n     | data             | 数据段（n = len - 12）       |
 *   | 12+n | 2     | crc16            | 对 sof~data 全部字节校验，低字节在前 |
 *   -----------------------------------------------------------------
 ******************************************************************************/
#ifndef __APP_OFFICIAL_PROTO_H__
#define __APP_OFFICIAL_PROTO_H__

#include <stdint.h>
#include "app_tac_config.h"

/* -------------------------- 帧格式常量 -------------------------- */
#define OFFICIAL_SOF                (0xAAU)   /* 帧头 */
#define OFFICIAL_VER                (0x20U)   /* 协议版本 V1.0（主版本1，次版本0） */
#define OFFICIAL_MAJOR_VER          (0x01U)   /* 协议主版本号（ver[7:5]） */
#define OFFICIAL_HEADER_LEN         (12U)     /* sof~cmd_type 共12字节（不含crc16） */
#define OFFICIAL_MAX_DATA_LEN       (242U)    /* 数据段最大长度（256-14） */
#define OFFICIAL_MAX_FRAME_LEN      (OFFICIAL_HEADER_LEN + OFFICIAL_MAX_DATA_LEN + 2U)  /* 256 */

/* -------------------------- 模块 ID -------------------------- */
#define OFFICIAL_MODULE_HC          (0x10U)   /* 手掌主控 */
#define OFFICIAL_MODULE_TS          (0x30U)   /* 触觉传感器（本机身份） */
#define OFFICIAL_MODULE_PC          (0x90U)   /* 上位机 */

/* 触觉传感器子模块 ID（手指） */
#define OFFICIAL_SUB_THUMB          (0x01U)
#define OFFICIAL_SUB_INDEX          (0x02U)
#define OFFICIAL_SUB_MIDDLE         (0x03U)
#define OFFICIAL_SUB_RING           (0x04U)
#define OFFICIAL_SUB_LITTLE         (0x05U)
#define OFFICIAL_SUB_PALM           (0x06U)
#define OFFICIAL_SUB_BROADCAST      (0xFFU)   /* 子模块广播 */

/* -------------------------- 命令类型 -------------------------- */
#define OFFICIAL_CMD_NO_RESP        (0x00U)   /* 无需应答 */
#define OFFICIAL_CMD_REQUEST        (0x01U)   /* 请求需应答 */
#define OFFICIAL_CMD_RESPONSE       (0x02U)   /* 应答包 */

/* -------------------------- 命令集 -------------------------- */
#define OFFICIAL_CMD_SET_COMMON     (0x00U)   /* 通用命令集 */
#define OFFICIAL_CMD_SET_TACTILE    (0x30U)   /* 触觉传感器命令集 */

/* 通用命令集 cmd_id */
#define OFFICIAL_CMD_GET_VERSION    (0x00U)
#define OFFICIAL_CMD_GET_UID        (0x01U)
#define OFFICIAL_CMD_GET_ID         (0x02U)
#define OFFICIAL_CMD_SET_ID         (0x03U)
#define OFFICIAL_CMD_GET_VENDOR     (0x04U)
#define OFFICIAL_CMD_GET_HAND       (0x05U)

/* 触觉命令集 cmd_id */
#define OFFICIAL_CMD_GET_ORI_DATA   (0x10U)
#define OFFICIAL_CMD_GET_CALIB      (0x11U)
#define OFFICIAL_CMD_GET_FORCE      (0x12U)
#define OFFICIAL_CMD_GET_STATE      (0x13U)
#define OFFICIAL_CMD_GET_SKID_DATA  (0x14U)   /* get_touch_sensor_skid_data：滑移信息 */
#define OFFICIAL_CMD_SET_CYCLE_SW   (0x15U)   /* set_touch_sensor_cycle_sw：触觉信息周期上报开关（协议 v1.8） */

/* get_ori_data 分包（v1.6 全手统一分包）：
 * 应答 sender_sub_id=0x00；数据段 data[0]=总包数 data[1]=当前包号(0x01 起，各 1 字节)
 * data[2..]=点位力值（单位 = 克重/APP_FORCE_CALIB_OUT_UNIT_DIV，默认 2g 单位）；
 * 每包最多 240 点；全手有效点 750 个 → 4 包(240+240+240+30) */
#define OFFICIAL_ORI_POINTS_PER_PACKET (240U)
#define OFFICIAL_ORI_MAX_POINT_CNT  (750U)    /* 全手有效传感点总数（附录5 编号 1~750） */

/* set_touch_sensor_cycle_sw（0x30/0x15，协议 v1.8）数据段取值：
 * 请求 data[0]：0x00=开启周期上报 0x01=关闭周期上报
 * 应答 data[0]：回显当前生效状态（同上取值）
 * 周期上报内容与 get_ori_data 应答**逐字节同格式**：
 *   sender_sub_id=0x00、receiver_module_id=0x90(上位机)、receiver_sub_id=0x00、
 *   cmd_set=0x30、cmd_id=0x10、cmd_type=0x02、seq_num=分包号(0x01~0x04)、
 *   data[0]=总包数、data[1]=当前包号、data[2..]=点位力值（默认 2g 单位）。
 * 固件上电默认关闭（s_u8CycleReportEn = 0），掉电复位恢复该默认值。 */
#define OFFICIAL_CYCLE_SW_ON        (0x00U)   /* 周期上报：开启 */
#define OFFICIAL_CYCLE_SW_OFF       (0x01U)   /* 周期上报：关闭 */

/* 周期上报周期(ms)：125Hz = 8ms；节拍由 app_period_timer 的软件定时器驱动
 * （configTICK_RATE_HZ=1000，故 8ms 为整 tick，无量化误差） */
#define OFFICIAL_CYCLE_REPORT_PERIOD_MS (8U)

/* 组帧缓冲上下文：全手力值组帧(Official_SendOriDataBurst)的大缓冲为文件级静态
 * 缓冲，存在两条并发调用路径，必须各用一套，否则高优先级回调抢占后会把另一条
 * 路径的 4 分包内容改写（撕裂/错包）：
 *   - USART：usart 任务上下文（get_ori_data 查询应答）；
 *   - TIMER：定时器守护任务上下文（125Hz 周期上报，见 app_period_timer.c）。 */
#define OFFICIAL_BUF_CTX_USART          (0U)
#define OFFICIAL_BUF_CTX_TIMER          (1U)

/* get_touch_sensor_skid_data 应答（v2.1 起支持轨迹采样）：逐条滑移事件记录连续打包，每条 11 字节：
 * [0]     记录序号标识（0x01 起递增）
 * [1..2]  滑移起点矩阵编号（协议附录5 全局编号 1~750，uint16 小端，事件身份标识）
 * [3..4]  滑移方向角（uint16 小端，0.01°）
 * [5..6]  滑移长度（uint16 小端，0.01 step）
 * [7..8]  滑移平均速度（uint16 小端，0.01 step/采样帧，单位约定待厂商确认）
 * [9..10] 滑移中当前质心矩阵编号（协议附录5 全局编号 1~750，uint16 小端；
 *         滑移期间固件每帧刷新、事件结束保留末点，0=未滑移；上位机按轮询
 *         采样该点连成移动轨迹）
 * 应答 sender_sub_id=0x00、seq_num 回填请求序号；仅上报已发生滑移的事件记录；
 * 记录总长超一帧数据容量(242B=22 条)时分帧发送、记录序号跨帧续编 */
#define OFFICIAL_SKID_DATA_LEN      (11U)

/* -------------------------- 帧解析结构体 -------------------------- */
typedef struct {
    uint8_t  ver;                 /* 协议版本 */
    uint8_t  len;                 /* 数据长度（= 12 + data_len） */
    uint8_t  sender_module_id;    /* 发送模块 id */
    uint8_t  sender_sub_id;       /* 发送子模块 id */
    uint8_t  receiver_module_id;  /* 接收模块 id */
    uint8_t  receiver_sub_id;     /* 接收子模块 id */
    uint8_t  cmd_set;             /* 指令集 */
    uint8_t  cmd_id;              /* 指令 id */
    uint8_t  seq_num;             /* 包序号 */
    uint8_t  cmd_type;            /* 指令类型 */
    const uint8_t *data;          /* 数据段指针（指向帧内） */
    uint16_t data_len;            /* 数据段长度 */
} official_frame_t;

/* -------------------------- 对外接口 -------------------------- */
/**
 * @brief  处理一帧官方协议数据（校验 + 命令分发 + 应答）
 * @param  buf  接收到的帧数据
 * @param  len  帧长度
 * @retval 0=已处理，1=帧非法忽略
 */
uint8_t OfficialProto_Process(const uint8_t *buf, uint16_t len);

/**
 * @brief  官方协议 CRC8（poly 0x07, msb_first 不反转, init 0x00）
 */
uint8_t official_crc8(const uint8_t *data, uint16_t len);

/**
 * @brief  官方协议 CRC16（Modbus poly 0xA001, lsb_first 反转, init 0xFFFF）
 */
uint16_t official_crc16(const uint8_t *data, uint16_t len);

/**
 * @brief  组一帧官方协议应答帧
 * @param  buf              输出缓冲（至少 OFFICIAL_MAX_FRAME_LEN）
 * @param  sender_sub_id    发送子模块 id
 * @param  receiver_module_id  接收模块 id（回填请求方）
 * @param  receiver_sub_id  接收子模块 id（回填请求方）
 * @param  cmd_set / cmd_id / seq_num / cmd_type  回填请求
 * @param  data / data_len  数据段
 * @retval 帧总长度
 */
uint16_t OfficialProto_BuildFrame(uint8_t *buf, uint8_t sender_sub_id,
                                  uint8_t receiver_module_id, uint8_t receiver_sub_id,
                                  uint8_t cmd_set, uint8_t cmd_id, uint8_t seq_num,
                                  uint8_t cmd_type,
                                  const uint8_t *data, uint16_t data_len);

/**
 * @brief  设置触觉信息周期上报开关（0x30/0x15，协议 v1.8）
 * @param  u8Enable  1=开启 0=关闭
 * @note   仅本模块内部（0x15 命令）与 usart 任务上下文调用（升级触发停发）；
 *         无并发保护（单字节状态，调用方串行）。本函数为周期上报开关的唯一收口：
 *         置位后同步启停 app_period_timer 的 8ms 上报定时器。
 */
void OfficialProto_SetCycleReport(uint8_t u8Enable);

/**
 * @brief  查询触觉信息周期上报开关当前状态
 * @retval 1=开启 0=关闭
 */
uint8_t OfficialProto_GetCycleReport(void);

/**
 * @brief  主动投递一轮全手周期上报数据（125Hz，125Hz 周期上报核心）
 * @note   内容与 get_ori_data 应答逐字节同格式（见本文件 OFFICIAL_CYCLE_SW_ON 注释），
 *         固定 receiver_module_id=OFFICIAL_MODULE_PC(0x90)、receiver_sub_id=0x00；
 *         一轮 4 分包在同一循环内全部入队构成单突发；无有效点时静默不发送。
 *         **仅定时器守护任务上下文调用**（节拍见 app_period_timer.c），使用
 *         OFFICIAL_BUF_CTX_TIMER 组帧缓冲，与查询应答路径互不干扰。
 */
void OfficialProto_SendCycleReport(void);

#endif /* __APP_OFFICIAL_PROTO_H__ */
