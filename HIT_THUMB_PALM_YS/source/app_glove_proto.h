/*******************************************************************************
 * @file    app_glove_proto.h
 * @brief   灏存《上传与标定协议》协议层：Channel 0x07 标定命令 + Channel 0x02 上传帧
 * @date    2026-09-16
 *
 * 外层帧（与协议文档一致）：3C 3C | Channel | Flags | Length(u16 LE) | Payload |
 *          CRC16(u16 LE, 0xA001/初值 0x0000, 只校验 Payload) | 3E 3E。
 * Flags = (package_id << 2) | method；method：0=PUT、1=GET、2=RESPONSE。
 *
 *  - Channel 0x07 PUT：标定表下发（descriptor=part<<4|region_count，逐区域
 *    rows=2、2≤cols≤62、AD 严格递增/力值不降、Payload 精确吻合，整部位替换）
 *    与擦除（Payload=01 part_id）；两者均无独立应答；
 *  - Channel 0x07 GET：00 selector 状态查询，RESPONSE 复用 package_id；
 *  - Channel 0x02 PUT：每部位一帧上传 descriptor|format(0x01)|rows|cols|uint8
 *    有效点（按传感点编号升序连续，Payload = 4 + 有效点数 N，rows/cols 仍为
 *    物理矩阵尺寸）。
 *
 * 标定命令处理期间自动暂停 0x02 周期上传（AppGloveProto_UploadPaused），
 * 处理完毕恢复，见协议文档第 8 节。
 ******************************************************************************/
#ifndef __APP_GLOVE_PROTO_H__
#define __APP_GLOVE_PROTO_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Channel 编号（协议文档第 1 节） */
#define GLOVE_CH_UPLOAD      (0x02U)   /* 主动上传 */
#define GLOVE_CH_CALIB       (0x07U)   /* 标定/状态查询/擦除 */

/* method 编码（Flags 低两位） */
#define GLOVE_METHOD_PUT     (0x00U)
#define GLOVE_METHOD_GET     (0x01U)
#define GLOVE_METHOD_RESP    (0x02U)

/* 部位容量（与协议一致：6 部位、单部位 ≤3 区域、单区域 ≤62 点） */
#define GLOVE_PART_MAX       (6U)
#define GLOVE_REGION_MAX     (3U)
#define GLOVE_POINTS_MAX     (62U)

/* PUT 接收容量（参考固件 Payload 缓冲 512B） */
#define GLOVE_PUT_PAYLOAD_MAX (512U)

/**
 * @brief  灏存外层帧 CRC16（poly 0xA001，初值 0x0000，只校验 Payload）
 * @note   供接收攒帧侧校验与发送组帧侧共用，保证收发算法同源
 */
uint16_t AppGloveProto_Crc16(const uint8_t *data, uint16_t len);

/**
 * @brief  3C 命令帧入口（已通过外层头尾/CRC 校验的完整 Payload）
 * @param  channel  帧 Channel 字段（仅处理 0x07，其余忽略）
 * @param  flags    帧 Flags 字段（method + package_id）
 * @param  payload  Payload 首地址（可为 NULL 当 len==0）
 * @param  len      Payload 长度
 * @note   usart 接收任务上下文串行调用；内部处理期间暂停 0x02 周期上传，
 *         返回前恢复。无应答的命令静默处理。
 */
void AppGloveProto_HandleCmdFrame(uint8_t channel, uint8_t flags,
                                  const uint8_t *payload, uint16_t len);

/**
 * @brief  组一帧 Channel 0x02 部位上传帧
 *         （descriptor|format|rows|cols|有效点按编号升序连续的 uint8）
 * @param  part   部位 1~6
 * @param  buf    输出帧缓冲（掌心最大 4+141+10=155 字节）
 * @param  size   缓冲容量
 * @retval 实际帧长度；part 非法、装配失败或缓冲不足返回 0
 * @note   Payload = 4 + N（N=有效点数：拇指 101/食指 130/中指 137/无名指 130/
 *         小指 111/掌心 141），无效点不再占位，故 Length ≠ rows×cols；
 *         rows/cols 仍填物理矩阵尺寸（14/14、17/12、17/13、17/12、17/11、15/16），
 *         上位机按 mapping 取前 N 个字节。
 *         数据取自采样发布快照（TacConfig_GetSnapshot），读侧免锁；
 *         定时器回调上下文调用（与标定命令上下文不同，经暂停标志互斥）。
 */
uint16_t AppGloveProto_BuildUploadFrame(uint8_t part, uint8_t *buf, uint16_t size);

/**
 * @brief  查询标定命令处理中（0x02 周期上传应暂停）
 * @retval 1=暂停 0=正常
 */
uint8_t AppGloveProto_UploadPaused(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_GLOVE_PROTO_H__ */
