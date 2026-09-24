/*******************************************************************************
 * @file    app_official_proto.c
 * @brief   官方《华威科-智元1.5T触觉数据手套通讯协议》实现（请求-应答）
 * @date    2026-08-28
 * @note    主控以"触觉传感器(0x30)"身份应答，与 3C 3C 标定协议按帧头隔离共存。
 * @note    两条并发调用路径（本模块任何接口均禁止在中断上下文调用）：
 *          - usart 任务：0xAA 整帧攒帧/切帧/解析后调用 OfficialProto_Process，
 *            执行查询应答（见 app_usart_task.c 的 AppUsart_HandleAaTask）；
 *          - app_period_timer 软件定时器（守护任务，全系统最高任务优先级）：
 *            执行 125Hz 周期上报。
 *          两条路径的组帧大缓冲各备一套（OFFICIAL_BUF_CTX_USART / _TIMER）互不共享。
 * @note    应答统一经应用层发送接口 AppUsartTxTask_Send()（app_usart_tx_task.h）入
 *          静态发送队列即返回、任务不阻塞；实际发送由 app_usart_tx_task 任务
 *          "出队 -> USART_BSP_Send -> 等发送完成通知"逐帧阻塞式完成。
 * @note    采集数据一律读采样发布快照（TacConfig_GetSnapshot，见 app_tac_config.c）：
 *          写侧（采样任务）轮末把成品帧整结构发布到双缓冲快照并原子切换活动索引，
 *          读侧无需取 m_hSampleDataMutex 即可读到完整一致帧，故应答路径不会被整轮采集
 *          写窗口阻塞；禁止直接读工作区 tac_data_cfg（会被写侧就地改写，导致撕裂帧）。
 * @note    SN：第 1-13 位固定前缀（含 57UU56 物料段，上位机据此判定左右手布局），
 *          第 14-37 位为 MCU 硬件唯一 ID（EFM UQID0/UQID1/UQID2 共 96bit）派生的
 *          24 位大写十六进制；SN 不含生产日期与流水号，上位机按 37 位分段解读。
 ******************************************************************************/
#include "app_official_proto.h"
#include "usart_bsp.h"
#include "app_usart_task.h"
#include "app_usart_tx_task.h"   /* 发送出口 AppUsartTxTask_Send */
#include "app_period_timer.h"    /* 周期上报节拍：SetCycleReport 收口启停 8ms 定时器 */
#include "app_sample_task.h"
#include "app_skid_detect.h"
#include "hc32_ll_efm.h"    /* EFM_GetUID / stc_efm_unique_id_t：MCU 硬件唯一 ID，用于派生 SN 后 24 位 */
#include <string.h>
#include <stddef.h>

/* ============================ 设备静态参数 ============================ */
static uint8_t s_u8DeviceId = 0x01U; /* 设备地址，默认0x01 */
static uint8_t s_u8HandSide = 0x00U; /* 设备左右：0=左手 1=右手（本工程实物装配为左手手套） */

/* 触觉信息周期上报开关（0x30/0x15，协议 v1.8）：1=开启 0=关闭；
 * 上电默认关闭（0），掉电复位恢复该默认值。
 * volatile：usart 任务写(0x15/升级)、定时器守护任务回调读(8ms 上报节拍)，
 * 跨上下文共享必须保证可见性（单字节读写本身原子，无需临界区） */
static volatile uint8_t s_u8CycleReportEn = 0U;

static const uint8_t s_au8Version[3] = { 0x01U, 0x02U, 0x00U };                            /* 主/次/修订 = v1.2.0 */

/* SN 前 13 位固定段：AX7(型号) + ZZ(预留) + XX(类型) + PPPPPP(物料段 57UU56，左手触觉内层)。
 * 第 8-13 位物料段是上位机判定左右手布局的依据（57UU56=左手 / 57UU57=右手），严禁改动。
 * 注：第 6-7 位（'0','1'，协议附录6 定义为"软件版本"段，即 XX）本次**不随固件版本递增**，
 *     保持 '01' 不变——变更该段会改变全部设备的序列号标识与上位机 SN 校验；
 *     固件版本以 s_au8Version / get_version(0x30/0x00) 为准（当前 v1.2.0） */
static const uint8_t s_au8SnPrefix[13] = { 'A','X','7','0','1','0','1','5','7','U','U','5','6' };

/* SN 运行期缓冲（37 字节 ASCII）：第 1-13 位由 s_au8SnPrefix 拷贝，第 14-37 位（索引 13..36）
 * 由主控 MCU 硬件唯一 ID（96bit）派生为 24 位大写十六进制，保证每台产品 SN 唯一；
 * SN 中不含生产日期（无法凭 SN 追溯生产批次，已确认接受该代价）。
 * 由 Official_InitUid() 在首次 get_uid 查询时一次性生成，之后单次上电周期内恒定 */
static uint8_t s_au8Uid[37];
static uint8_t s_u8UidReady = 0U;                                                           /* SN 生成标志：0=未生成 1=已生成 */

static const uint8_t s_au8Vendor[6]  = { 0x56U, 0x41U, 0x31U, 0x31U, 0x37U, 0x35U };        /* "VA1175" */

/* ============================ 内部函数声明 ============================ */
static void     Official_InitUid(void);
static uint8_t  Official_ParseFrame(const uint8_t *buf, uint16_t len, official_frame_t *frame);
static void     Official_SendResponse(const official_frame_t *frame, const uint8_t *data, uint16_t data_len);
static uint8_t  Official_GetPoint(const tac_data_cfg_t *p, tac_area_t area, uint16_t row, uint16_t col);
static uint16_t Official_CollectValidPoints(const tac_data_cfg_t *p, const tac_data_cfg_t *mask,
                                            uint8_t *out, uint16_t max_len);
static uint32_t Official_CalcTotalForce(const tac_data_cfg_t *p);
static uint16_t Official_SendOriDataBurst(uint8_t u8ReceiverModuleId, uint8_t u8ReceiverSubId,
                                          uint8_t u8BufCtx);
static void Official_HandleGetVersion(const official_frame_t *frame);
static void Official_HandleGetUid(const official_frame_t *frame);
static void Official_HandleGetId(const official_frame_t *frame);
static void Official_HandleSetId(const official_frame_t *frame);
static void Official_HandleGetVendor(const official_frame_t *frame);
static void Official_HandleGetHand(const official_frame_t *frame);
static void Official_HandleGetOriData(const official_frame_t *frame);
#if TAC_CALIB_EN
static void Official_HandleGetCalib(const official_frame_t *frame);
#endif
static void Official_HandleGetForce(const official_frame_t *frame);
static void Official_HandleGetState(const official_frame_t *frame);
static void Official_HandleGetSkidData(const official_frame_t *frame);
static void Official_HandleSetCycleSw(const official_frame_t *frame);

/* ============================ CRC 计算 ============================ */
/**
 * @brief CRC8：poly 0x07, msb_first 不反转, init 0x00（与协议附录 crc8_atm_table 等价）
 */
uint8_t official_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t  crc = 0x00U;
    uint16_t i;

    for (i = 0U; i < len; i++)
    {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (uint8_t)((crc & 0x80U) ? ((crc << 1U) ^ 0x07U) : (crc << 1U));
        }
    }
    return crc;
}

/**
 * @brief CRC16：Modbus, poly 0xA001, lsb_first 反转, init 0xFFFF（与协议附录 modbus_crc16_table 等价）
 */
uint16_t official_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    for (i = 0U; i < len; i++)
    {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (uint16_t)((crc & 0x0001U) ? ((crc >> 1U) ^ 0xA001U) : (crc >> 1U));
        }
    }
    return crc;
}

/* ============================ 帧解析 ============================ */
/**
 * @brief  解析并校验一帧官方协议数据
 * @param  buf    接收到的帧数据缓冲区
 * @param  len    缓冲区实际长度
 * @param  frame  解析结果输出（仅校验通过时有效）
 * @retval 1U 校验通过并填充 frame；0U 帧非法
 * @note   校验顺序：长度下限 -> 帧头 0xAA -> 主版本号(ver[7:5]) -> len 上界/与总长匹配
 *         -> CRC8(覆盖 sof+ver+len) -> CRC16(覆盖 sof~data 末尾) -> 填充 frame
 */
static uint8_t Official_ParseFrame(const uint8_t *buf, uint16_t len, official_frame_t *frame)
{
    uint16_t u16FrameLen;
    uint16_t u16CrcRecv; /* 帧尾 crc16 接收值（小端） */

    if ((buf == NULL) || (frame == NULL) || (len < (OFFICIAL_HEADER_LEN + 2U)))
    {
        return 0U;
    }
    if (buf[0] != OFFICIAL_SOF)
    {
        return 0U;
    }
    /* 协议主版本校验（ver[7:5]） */
    if (((buf[1] >> 5U) & 0x07U) != OFFICIAL_MAJOR_VER)
    {
        return 0U;
    }
    /* len 上界：数据段不得超过协议上限 242B（防超长帧占用接收/处理资源） */
    if (buf[2] > (uint8_t)(OFFICIAL_HEADER_LEN + OFFICIAL_MAX_DATA_LEN))
    {
        return 0U;
    }
    /* len 下限与帧总长匹配：总长 = len + crc16(2字节) */
    u16FrameLen = (uint16_t)buf[2] + 2U;
    if ((u16FrameLen != len) || (buf[2] < OFFICIAL_HEADER_LEN))
    {
        return 0U;
    }
    /* crc8 校验：覆盖 sof+ver+len */
    if (official_crc8(buf, 3U) != buf[3])
    {
        return 0U;
    }
    /* crc16 校验：覆盖 sof~data 末尾，帧尾按小端存放 */
    u16CrcRecv = (uint16_t)(buf[len - 2U] | ((uint16_t)buf[len - 1U] << 8U));
    if (official_crc16(buf, len - 2U) != u16CrcRecv)
    {
        return 0U;
    }

    frame->ver                = buf[1];
    frame->len                = buf[2];
    frame->sender_module_id   = buf[4];
    frame->sender_sub_id      = buf[5];
    frame->receiver_module_id = buf[6];
    frame->receiver_sub_id    = buf[7];
    frame->cmd_set            = buf[8];
    frame->cmd_id             = buf[9];
    frame->seq_num            = buf[10];
    frame->cmd_type           = buf[11];
    frame->data               = &buf[OFFICIAL_HEADER_LEN];
    frame->data_len           = (uint16_t)buf[2] - OFFICIAL_HEADER_LEN;

    return 1U;
}

/* ============================ 组帧 ============================ */
/**
 * @brief  按官方协议格式组装一帧数据（含 crc8 包头校验 + crc16 帧尾校验）
 * @param  buf              输出缓冲区（容量需 >= OFFICIAL_MAX_FRAME_LEN）
 * @param  sender_sub_id    发送方子模块 id（手指 id 或 0x00）
 * @param  receiver_module_id  接收方模块 id（应答时回填请求方）
 * @param  receiver_sub_id  接收方子模块 id（应答时回填请求方）
 * @param  cmd_set / cmd_id / cmd_type  原样回填请求帧字段；seq_num 由调用方按应答语义给出
 *         （普通命令恒 0x01；0x30/0x10 多分包应答传分包号 0x01~0x04），不再回显请求序号
 * @param  data / data_len  数据段内容与长度（data_len 不得超过 OFFICIAL_MAX_DATA_LEN）
 * @retval 组帧总长度（= OFFICIAL_HEADER_LEN + data_len + 2）；参数非法返回 0
 * @note   ver 固定 OFFICIAL_VER(0x20)，发送方模块固定 OFFICIAL_MODULE_TS(0x30)
 */
uint16_t OfficialProto_BuildFrame(uint8_t *buf, uint8_t sender_sub_id,
                                  uint8_t receiver_module_id, uint8_t receiver_sub_id,
                                  uint8_t cmd_set, uint8_t cmd_id, uint8_t seq_num,
                                  uint8_t cmd_type,
                                  const uint8_t *data, uint16_t data_len)
{
    uint16_t crc;

    if ((buf == NULL) || (data_len > OFFICIAL_MAX_DATA_LEN))
    {
        return 0U;
    }

    buf[0] = OFFICIAL_SOF;
    buf[1] = OFFICIAL_VER;
    buf[2] = (uint8_t)(OFFICIAL_HEADER_LEN + data_len);
    buf[3] = official_crc8(buf, 3U);                    /* 包头校验 */
    buf[4] = OFFICIAL_MODULE_TS;                        /* 本机为触觉传感器 */
    buf[5] = sender_sub_id;
    buf[6] = receiver_module_id;
    buf[7] = receiver_sub_id;
    buf[8] = cmd_set;
    buf[9] = cmd_id;
    buf[10] = seq_num;
    buf[11] = cmd_type;

    if ((data != NULL) && (data_len > 0U))
    {
        memcpy(&buf[OFFICIAL_HEADER_LEN], data, data_len);
    }

    crc = official_crc16(buf, OFFICIAL_HEADER_LEN + data_len);
    buf[OFFICIAL_HEADER_LEN + data_len]      = (uint8_t)(crc & 0xFFU);
    buf[OFFICIAL_HEADER_LEN + data_len + 1U] = (uint8_t)((crc >> 8U) & 0xFFU);

    return (uint16_t)(OFFICIAL_HEADER_LEN + data_len + 2U);
}

/* ============================ 数据访问 ============================ */
/**
 * @brief  按区域/行列读取单个传感点的力值
 * @param  p     采样数据配置指针（GetTacDataCfg() 返回值）
 * @param  area  节级区域枚举（如 TAC_AREA_THUMB_DIP）
 * @param  row   区域内行索引
 * @param  col   区域内列索引
 * @retval 该点力值（0~255，单位 = 克重/APP_FORCE_CALIB_OUT_UNIT_DIV，默认 2g
 *         → 上限 510g）；非法区域返回 0
 */
static uint8_t Official_GetPoint(const tac_data_cfg_t *p, tac_area_t area, uint16_t row, uint16_t col)
{
    switch (area)
    {
        case TAC_AREA_THUMB_DIP:   return p->thumb_dip[row][col];
        case TAC_AREA_THUMB_MCP:   return p->thumb_mcp[row][col];
        case TAC_AREA_INDEX_DIP:   return p->index_dip[row][col];
        case TAC_AREA_INDEX_PIP:   return p->index_pip[row][col];
        case TAC_AREA_INDEX_MCP:   return p->index_mcp[row][col];
        case TAC_AREA_MIDDLE_DIP:  return p->middle_dip[row][col];
        case TAC_AREA_MIDDLE_PIP:  return p->middle_pip[row][col];
        case TAC_AREA_MIDDLE_MCP:  return p->middle_mcp[row][col];
        case TAC_AREA_RING_DIP:    return p->ring_dip[row][col];
        case TAC_AREA_RING_PIP:    return p->ring_pip[row][col];
        case TAC_AREA_RING_MCP:    return p->ring_mcp[row][col];
        case TAC_AREA_LITTLE_DIP:  return p->little_dip[row][col];
        case TAC_AREA_LITTLE_PIP:  return p->little_pip[row][col];
        case TAC_AREA_LITTLE_MCP:  return p->little_mcp[row][col];
        case TAC_AREA_PALM:        return p->palm[row][col];
        default:                   return 0U;
    }
}

/**
 * @brief  收集全手有效传感点力值（默认 2g 单位），按协议附录5 全局编号 1~N 顺序输出
 * @param  p        采样数据配置指针（GetTacDataCfg() 返回值）
 * @param  mask     掩码配置指针（GetTacMaskData() 返回值，NULL 时返回 0）
 * @param  out      输出缓冲区（第 i 个元素 = 全局编号 i+1 对应点的力值，默认 2g 单位）
 * @param  max_len  输出缓冲区容量
 * @retval 收集到的有效点总数（全手 750）；参数非法返回 0
 * @note   节遍历顺序 = tac_area_t 枚举序（拇指远节→…→掌心），与协议点位顺序一致；
 *         data/mask 成员数组按区域相对行列存放，故行/列索引取 0~(区域尺寸-1)，
 *         不使用 TacAreaRange 的阵列绝对坐标（对 thumb 等区域会越界读相邻内存）；
 *         掩码每格值为该节局部编号(1 起行优先连续)，以"全局前缀+局部号"作桶索引
 *         写入，与掩码是否为镜像布局（ys/zs）无关，保证输出即全局编号顺序
 */
static uint16_t Official_CollectValidPoints(const tac_data_cfg_t *p, const tac_data_cfg_t *mask,
                                            uint8_t *out, uint16_t max_len)
{
    uint16_t u16Base = 0U;
    tac_area_t area;

    if ((p == NULL) || (mask == NULL) || (out == NULL))
    {
        return 0U;
    }

    for (area = (tac_area_t)TAC_AREA_THUMB_DIP; area < TAC_AREA_MAX; area = (tac_area_t)(area + 1U))
    {
        const tac_area_range_t *range = &TacAreaRange[area];
        uint16_t u16Rows = (uint16_t)(range->row_end - range->row_start + 1U);
        uint16_t u16Cols = (uint16_t)(range->col_end - range->col_start + 1U);
        uint16_t u16RowCnt = 0U;
        uint16_t r, c;

        for (r = 0U; r < u16Rows; r++)
        {
            for (c = 0U; c < u16Cols; c++)
            {
                uint8_t u8Local = Official_GetPoint(mask, area, r, c);
                uint16_t u16Idx;

                if (u8Local == 0U)
                {
                    continue;   /* 无效位（无传感点） */
                }
                u16Idx = (uint16_t)(u16Base + (uint16_t)u8Local - 1U);
                if (u16Idx >= max_len)
                {
                    return u16Base;   /* 缓冲不足，防御性截断 */
                }
                out[u16Idx] = Official_GetPoint(p, area, r, c);
                u16RowCnt++;
            }
        }
        u16Base += u16RowCnt;
    }
    return u16Base;
}

/**
 * @brief  统计全手所有传感点力值总和（默认 2g 单位）
 * @param  p  采样数据配置指针
 * @retval 总力值（默认 2g 单位；uint32_t 累加，防止大量传感点累加溢出）
 */
static uint32_t Official_CalcTotalForce(const tac_data_cfg_t *p)
{
    uint32_t total = 0U;
    tac_area_t area;

    for (area = (tac_area_t)TAC_AREA_THUMB_DIP; area < TAC_AREA_MAX; area = (tac_area_t)(area + 1U))
    {
        const tac_area_range_t *range = &TacAreaRange[area];
        uint16_t r, c;

        for (r = range->row_start; r <= range->row_end; r++)
        {
            for (c = range->col_start; c <= range->col_end; c++)
            {
                total += Official_GetPoint(p, area, r, c);
            }
        }
    }
    return total;
}

/* ============================ 应答发送 ============================ */
/**
 * @brief  以"触觉传感器"身份组一帧应答并入发送队列（usart 任务上下文专用）
 * @param  frame     收到的请求帧（用于回填接收方模块/子模块与 cmd 字段）
 * @param  data      应答数据段内容
 * @param  data_len  应答数据段长度
 * @note   发送方子模块固定 0x00；cmd_type 固定 OFFICIAL_CMD_RESPONSE(0x02)；
 *         seq_num 恒 0x01（对齐厂商实测：普通命令应答序号不递增、不回显请求序号）；
 *         本模块整体在 usart 任务上下文单实例执行，应答经应用层发送队列
 *         AppUsartTxTask_Send() 入队：整帧拷入发送队列即返回、任务不阻塞，实际发送
 *         由 app_usart_tx_task 发送任务（出队→USART_BSP_Send→等发送完成通知）阻塞式
 *         逐帧完成；入队失败（队列满/参数错）时静默丢弃本帧，不补发、不中断后续发送
 */
static void Official_SendResponse(const official_frame_t *frame, const uint8_t *data, uint16_t data_len)
{
    /* 组帧缓冲：usart 任务单实例串行执行，静态化避免大缓冲占用任务栈 */
    static uint8_t au8Tx[OFFICIAL_MAX_FRAME_LEN];
    uint16_t u16FrameLen;

    u16FrameLen = OfficialProto_BuildFrame(au8Tx, 0x00U,
                                           frame->sender_module_id, frame->sender_sub_id,
                                           frame->cmd_set, frame->cmd_id, 0x01U,
                                           OFFICIAL_CMD_RESPONSE, data, data_len);
    if (u16FrameLen > 0U)
    {
        (void)AppUsartTxTask_Send(au8Tx, u16FrameLen);
    }
}

/* ============================ 采样数据读保护策略 ============================ */
/* 采样缓存 tac_data_cfg 由 sample/fingers 任务在 m_hSampleDataMutex 临界段内整轮
 * 就地改写；官方应答侧（get_ori_data/get_force 等）不再直接读该工作区、也不取该
 * 互斥锁，而是统一读"轮末发布快照" TacConfig_GetSnapshot()：写侧发布时先把工作区
 * 整结构拷入非活动缓冲、再原子切换活动索引（见 app_tac_config.c），故读侧读到的
 * 必是完整成品帧、不会撕裂，同时也不必等待整轮采集写窗口，消除应答延迟抖动。
 * 注意：后续若新增路径确需直接读工作区 tac_data_cfg，须自行取 m_hSampleDataMutex
 *（由 APP_Sample_TaskCreate 静态创建，创建前为 NULL）保护整段读取。 */

/* ============================ 通用命令集 0x00 ============================ */
/**
 * @brief  get_version：返回固件版本号
 * @note   应答数据段 = s_au8Version[3] = {主版本, 次版本, 修订号}
 */
static void Official_HandleGetVersion(const official_frame_t *frame)
{
    Official_SendResponse(frame, s_au8Version, sizeof(s_au8Version));
}

/**
 * @brief  生成产品序列码（SN）：首次调用时一次性完成，之后 SN 恒定
 * @note   第 1-13 位直接拷贝固定前缀 s_au8SnPrefix（含物料段，上位机左右手判定依据）；
 *         第 14-37 位取主控 MCU 硬件唯一 ID（UQID0/UQID1/UQID2 完整 96bit），
 *         按 UQID0(最高 32 位) -> UQID1 -> UQID2(最低 32 位) 顺序，每个 32bit 字输出 8 位
 *         大写十六进制（字符集 0-9/A-F，字内高位在前），共 24 位。
 *         为规避嵌入式 printf 的 64 位整形格式化依赖，此处手工 nibble 循环生成。
 * @note   由 Official_HandleGetUid 懒加载调用，运行在 usart 任务上下文，单实例串行、无中断并发，无需加锁
 */
static void Official_InitUid(void)
{
    stc_efm_unique_id_t stcUid;
    uint32_t            au32UidWord[3];
    uint8_t             i;
    uint8_t             j;

    (void)memcpy((void *)s_au8Uid, (const void *)s_au8SnPrefix, sizeof(s_au8SnPrefix));

    EFM_GetUID(&stcUid);                                            /* 读只读寄存器 CM_EFM->UQID0/1/2 */
    au32UidWord[0] = stcUid.u32UniqueID0;                           /* 最高 32 位 */
    au32UidWord[1] = stcUid.u32UniqueID1;
    au32UidWord[2] = stcUid.u32UniqueID2;                           /* 最低 32 位 */

    for (i = 0U; i < 3U; i++)                                       /* 3 个 32bit 字 */
    {
        for (j = 0U; j < 8U; j++)                                   /* 每字高位 -> 低位，各 8 字符 */
        {
            uint8_t u8Nib = (uint8_t)((au32UidWord[i] >> (4U * (7U - j))) & 0x0FU);

            s_au8Uid[13U + (8U * i) + j] =
                (u8Nib < 10U) ? (uint8_t)('0' + u8Nib) : (uint8_t)('A' + (u8Nib - 10U));
        }
    }

    s_u8UidReady = 1U;
}

/**
 * @brief  get_uid：返回设备产品序列码（SN）
 * @note   应答数据段 = s_au8Uid[37]，37 字节 ASCII 字符串，帧长/CRC 与协议保持完全一致
 *         （协议层零改动）。第 1-13 位为固定前缀（含物料段，上位机据此判定左右手布局）；
 *         第 14-37 位为 MCU 硬件唯一 ID（96bit）派生的 24 位大写十六进制（每台产品唯一），
 *         故 SN 中不含生产日期/流水号，上位机按 37 位分段解读
 * @note   首次查询时懒加载生成 SN（Official_InitUid），后续查询直接复用 RAM 缓冲
 */
static void Official_HandleGetUid(const official_frame_t *frame)
{
    if (s_u8UidReady == 0U)
    {
        Official_InitUid();
    }

    Official_SendResponse(frame, s_au8Uid, sizeof(s_au8Uid));
}

/**
 * @brief  get_id：返回设备地址（默认 0x01）
 * @note   应答数据段 = s_u8DeviceId（1 字节）
 */
static void Official_HandleGetId(const official_frame_t *frame)
{
    Official_SendResponse(frame, &s_u8DeviceId, 1U);
}

/**
 * @brief  set_id：写入设备地址（仅 RAM 生效，掉电复位恢复默认值）
 * @note   请求数据段须为 1 字节且 <= 0xFE；应答 0x00=写入成功 0x01=参数非法
 */
static void Official_HandleSetId(const official_frame_t *frame)
{
    uint8_t u8Ack;

    if ((frame->data_len == 1U) && (frame->data[0] <= 0xFEU))
    {
        s_u8DeviceId = frame->data[0];
        u8Ack = 0x00U; /* 写入成功 */
    }
    else
    {
        u8Ack = 0x01U; /* 写入失败 */
    }
    Official_SendResponse(frame, &u8Ack, 1U);
}

/**
 * @brief  get_vendor：返回厂商信息
 * @note   应答数据段 = s_au8Vendor[6] = ASCII "VA1175"
 */
static void Official_HandleGetVendor(const official_frame_t *frame)
{
    Official_SendResponse(frame, s_au8Vendor, sizeof(s_au8Vendor));
}

/**
 * @brief  get_hand：返回设备左右属性
 * @note   应答数据段 = s_u8HandSide（0=左手 1=右手）
 */
static void Official_HandleGetHand(const official_frame_t *frame)
{
    Official_SendResponse(frame, &s_u8HandSide, 1U);
}

/* ============================ 触觉命令集 0x30 ============================ */
/**
 * @brief  组一轮全手有效传感点力值分包帧并整批入发送队列（单突发）
 * @param  u8ReceiverModuleId  接收方模块 id（应答=回填请求方；周期上报=OFFICIAL_MODULE_PC）
 * @param  u8ReceiverSubId     接收方子模块 id（应答=回填请求方；周期上报=0x00）
 * @retval 实际成功入队的帧数（无有效点/掩码不可用时为 0，静默不发送）
 * @note   帧格式（get_ori_data 应答与 125Hz 周期上报**逐字节同格式**）：
 *         sender_sub_id=0x00、cmd_set=0x30、cmd_id=0x10、cmd_type=0x02(应答包)、
 *         seq_num=当前分包号(0x01~0x04，与 data[1] 一致)；
 *         数据段 data[0]=总包数 data[1]=当前包号(0x01~0x04，从 0x01 起)
 *         data[2..]=点位力值（默认 2g 单位），点数顺序即协议附录5 全局编号 1~750；
 *         每包最多 240 点（240B，数据段 242B 正好等于协议上限）→ 750 点共 4 包
 * @note   数据来源 TacConfig_GetSnapshot()（采样任务轮末发布的最近一帧完整快照，
 *         免锁读取、不阻塞在整轮采集写窗口上）；掩码来源 GetTacMaskData()（当前
 *         生效 ys/zs，按掩码值定位保证编号顺序与镜像无关）。
 * @note   整批投递：4 包先在本函数内全部组帧并 AppUsartTxTask_Send() 入队，函数返回后
 *         由 app_usart_tx_task 发送任务在一次取帧循环中连续发出（单突发，帧间不被
 *         其它发送路径插入），避免逐包"入队-等发送完成-再入队"造成帧间空隙。
 */
static uint16_t Official_SendOriDataBurst(uint8_t u8ReceiverModuleId, uint8_t u8ReceiverSubId,
                                          uint8_t u8BufCtx)
{
    /* 组帧大缓冲：按调用上下文**各备一套**（见 app_official_proto.h OFFICIAL_BUF_CTX_*）。
       原因：周期上报在定时器守护任务(最高任务优先级)回调中执行，会抢占 usart 任务
       的查询应答路径；二者若共用同一缓冲，抢占后会把对方尚未投递完的 4 分包内容
       改写，产出撕裂/错序帧。静态化避免大缓冲占用任务栈（守护任务栈尤为紧张）。 */
    static uint8_t s_au8OriPoints[2][OFFICIAL_ORI_MAX_POINT_CNT];  /* 全手有效点缓冲(2×750B) */
    static uint8_t s_au8OriData[2][OFFICIAL_MAX_DATA_LEN];         /* 数据段缓冲 */
    static uint8_t s_au8OriTx[2][OFFICIAL_MAX_FRAME_LEN];          /* 组帧缓冲(投递后即复用) */
    uint8_t  u8Ctx = (u8BufCtx == OFFICIAL_BUF_CTX_TIMER) ? OFFICIAL_BUF_CTX_TIMER
                                                          : OFFICIAL_BUF_CTX_USART;
    uint8_t *pu8Points = &s_au8OriPoints[u8Ctx][0];
    uint8_t *pu8Data   = &s_au8OriData[u8Ctx][0];
    uint8_t *pu8Tx     = &s_au8OriTx[u8Ctx][0];
    uint16_t u16PointCnt;
    uint16_t u16PacketTotal;
    uint16_t u16PacketIdx;
    uint16_t u16Sent = 0U;

    /* 读轮末发布快照：写侧只在非活动缓冲上拷贝、拷贝完成后才切换活动索引，
       故此处取到的一定为完整一致帧（无撕裂）；快照读取全程免锁，不会阻塞在
       采样任务的整轮采集写窗口上。掩码为 const 只读数据，无需保护 */
    u16PointCnt = Official_CollectValidPoints(TacConfig_GetSnapshot(), GetTacMaskData(),
                                              pu8Points, OFFICIAL_ORI_MAX_POINT_CNT);
    if (u16PointCnt == 0U)
    {
        return 0U;   /* 数据/掩码不可用，不发送 */
    }

    u16PacketTotal = (uint16_t)((u16PointCnt + OFFICIAL_ORI_POINTS_PER_PACKET - 1U)
                                / OFFICIAL_ORI_POINTS_PER_PACKET);

    /* 4 包在同一循环内全部组帧入队（AppUsartTxTask_Send 为非阻塞整帧拷贝入队），
       本函数返回后由 app_usart_tx_task 发送任务一次性连续发出，构成单突发 */
    for (u16PacketIdx = 1U; u16PacketIdx <= u16PacketTotal; u16PacketIdx++)
    {
        uint16_t u16Start = (uint16_t)((u16PacketIdx - 1U) * OFFICIAL_ORI_POINTS_PER_PACKET);
        uint16_t u16Pts   = u16PointCnt - u16Start;
        uint16_t u16FrameLen;

        if (u16Pts > OFFICIAL_ORI_POINTS_PER_PACKET)
        {
            u16Pts = OFFICIAL_ORI_POINTS_PER_PACKET;
        }

        pu8Data[0] = (uint8_t)u16PacketTotal;
        pu8Data[1] = (uint8_t)u16PacketIdx;
        memcpy(&pu8Data[2], &pu8Points[u16Start], u16Pts);

        u16FrameLen = OfficialProto_BuildFrame(pu8Tx, 0x00U,
                                               u8ReceiverModuleId, u8ReceiverSubId,
                                               OFFICIAL_CMD_SET_TACTILE, OFFICIAL_CMD_GET_ORI_DATA,
                                               (uint8_t)u16PacketIdx,
                                               OFFICIAL_CMD_RESPONSE, pu8Data, (uint16_t)(2U + u16Pts));
        if (u16FrameLen == 0U)
        {
            continue;   /* 组帧失败，跳过该帧 */
        }
        if (AppUsartTxTask_Send(pu8Tx, u16FrameLen) != LL_OK) {
            /* 投递失败（发送队列满，本帧数据未入队、缓冲可复用）：本包丢弃即丢弃、
               本机不自动补发/重发，后续包仍按原序投递；缺失包由请求方按其自身机制
               自行重新发起 get_ori_data 查询（不依赖/不预设前端自动补发） */
            continue;
        }
        u16Sent++;
    }

    return u16Sent;
}

/**
 * @brief  get_ori_data：返回全手有效传感点力值（默认 2g 单位；主上报通道，v1.6 全手统一分包）
 * @note   请求：cmd_set=0x30 cmd_id=0x10；receiver_sub_id 为 0x00 或广播 0xFF 均响应
 * @note   应答帧格式与分包规则见 Official_SendOriDataBurst()；接收方模块/子模块回填请求方
 */
static void Official_HandleGetOriData(const official_frame_t *frame)
{
    /* usart 任务上下文：使用 USART 组帧缓冲，与定时器周期上报路径互不共享 */
    (void)Official_SendOriDataBurst(frame->sender_module_id, frame->sender_sub_id,
                                    OFFICIAL_BUF_CTX_USART);
}

/**
 * @brief  125Hz 周期上报投递入口：内容与 get_ori_data 应答逐字节同格式
 * @note   由 app_period_timer 的 8ms 软件定时器回调按 OFFICIAL_CYCLE_REPORT_PERIOD_MS
 *          (8ms=125Hz) 节拍调用（守护任务上下文，会抢占 usart 任务，故使用
 *          OFFICIAL_BUF_CTX_TIMER 组帧缓冲）；
 *         固定发往上位机（receiver_module_id=OFFICIAL_MODULE_PC(0x90)、receiver_sub_id=0x00）
 */
void OfficialProto_SendCycleReport(void)
{
    (void)Official_SendOriDataBurst(OFFICIAL_MODULE_PC, 0x00U, OFFICIAL_BUF_CTX_TIMER);
}

#if TAC_CALIB_EN
/**
 * @brief get_calib：触发重新零点校准，300ms 内应答。
 *        data[0]=0x01 触发校准，应答 0x00=成功 0x01=失败。
 */
static void Official_HandleGetCalib(const official_frame_t *frame)
{
    uint8_t u8Ack;

    if ((frame->data_len == 1U) && (frame->data[0] == 0x01U))
    {
        Sample_RequestRecalib();
        u8Ack = 0x00U;
    }
    else
    {
        u8Ack = 0x01U;
    }
    Official_SendResponse(frame, &u8Ack, 1U);
}
#endif /* TAC_CALIB_EN */

/**
 * @brief  get_force：返回全手总力值（默认 2g 单位，所有传感点累加）
 * @note   应答数据段 = uint32 小端序 4 字节；数据取自轮末发布快照
 *         （TacConfig_GetSnapshot()），为完整一致帧且免锁读取，不会阻塞在
 *         采样任务的整轮采集写窗口上
 */
static void Official_HandleGetForce(const official_frame_t *frame)
{
    uint8_t au8Data[4];
    uint32_t u32Total;

    u32Total = Official_CalcTotalForce(TacConfig_GetSnapshot());

    au8Data[0] = (uint8_t)(u32Total & 0xFFU);
    au8Data[1] = (uint8_t)((u32Total >> 8U) & 0xFFU);
    au8Data[2] = (uint8_t)((u32Total >> 16U) & 0xFFU);
    au8Data[3] = (uint8_t)((u32Total >> 24U) & 0xFFU);
    Official_SendResponse(frame, au8Data, sizeof(au8Data));
}

/**
 * @brief  get_state：返回触觉传感器工作状态
 * @note   应答数据段 = u8State（0x00=无错误/状态正常），当前固定返回 0x00
 */
static void Official_HandleGetState(const official_frame_t *frame)
{
    uint8_t u8State = 0x00U; /* 0x00=无错误（触觉传感器状态正常） */
    Official_SendResponse(frame, &u8State, 1U);
}

/**
 * @brief  get_touch_sensor_skid_data：返回全手滑移事件记录（v2.1 起支持轨迹采样）
 * @note   请求：cmd_set=0x30 cmd_id=0x14；receiver_sub_id 为 0x00 或广播 0xFF 均响应
 * @note   应答：逐条滑移事件记录连续打包，每条 OFFICIAL_SKID_DATA_LEN(11) 字节：
 *         [0]     记录序号标识（0x01 起递增，协议通信示例区域标识 0x01/0x02…）
 *         [1..2]  滑移起点矩阵编号（协议附录5 全局编号 1~750，uint16 小端，事件身份标识）
 *         [3..4]  滑移方向角（0.01°，uint16 小端）
 *         [5..6]  单次事件滑移长度（0.01 step，uint16 小端）
 *         [7..8]  滑移平均速度（0.01 step/采样帧，uint16 小端，单位待厂商确认）
 *         [9..10] 滑移中当前质心矩阵编号（协议附录5 全局编号 1~750，uint16 小端；
 *                 滑移期间滑移模块每帧刷新、事件结束保留末点；0=未滑移/无效。
 *                 上位机按轮询采样该点连成移动轨迹）
 *         帧头 sender_sub_id=0x00、seq_num=0x01（普通命令应答序号恒 0x01，跨帧不再重复递增）；
 *         记录总长超一帧数据容量(242B=22 条)时按条切分多帧、逐帧组帧后经
 *         AppUsartTxTask_Send() 入发送队列（多帧顺序由发送队列 FIFO 保证），
 *         记录序号跨帧续编，无事件槽不输出
 * @note   记录来源 = 拇指~掌心各手指 Skid_GetSnapshot（任务版快照读取，内部自带
 *         任务临界区）中 matrix_no!=0 的事件槽（仅上报已发生的滑移，快照由滑移
 *         检测模块后台持续刷新）；本处理器运行于 usart 任务上下文，使用任务版
 *         快照接口而非 Isr 版。
 */
static void Official_HandleGetSkidData(const official_frame_t *frame)
{
    static uint8_t au8Data[OFFICIAL_MAX_DATA_LEN]; /* 单帧数据段 ≤242B，每条11B→≤22条/帧；静态化避免占任务栈 */
    static uint8_t au8Tx[OFFICIAL_MAX_FRAME_LEN];
    uint16_t u16DataLen = 0U;
    uint8_t  u8RecSeq   = 0U;    /* 记录序号标识（跨帧续编，0xFF 上限防御） */
    uint8_t  u8Finger;

    for (u8Finger = 0U; u8Finger < (uint8_t)TAC_FINGER_MAX; u8Finger++)
    {
        static skid_snapshot_t astSnap[SKID_EVENT_MAX]; /* usart 任务单实例串行执行，静态化避免占任务栈 */
        uint8_t u8Cnt = Skid_GetSnapshot((uint8_t)(u8Finger + 1U), astSnap, SKID_EVENT_MAX);
        uint8_t u8Slot;

        for (u8Slot = 0U; u8Slot < u8Cnt; u8Slot++)
        {
            const skid_snapshot_t *pSnap = &astSnap[u8Slot];

            if (pSnap->matrix_no == 0U)
            {
                continue;   /* 无事件槽不输出 */
            }
            if (u8RecSeq == 0xFFU)
            {
                return;     /* 记录序号耗尽，防御性终止（理论最多 60 事件，不会到达） */
            }

            /* 当前帧数据段已满：先投递本帧再续填（记录条边界切分，投递即拷出，缓冲可复用） */
            if (u16DataLen > (uint16_t)(OFFICIAL_MAX_DATA_LEN - OFFICIAL_SKID_DATA_LEN))
            {
                uint16_t u16Len = OfficialProto_BuildFrame(au8Tx, 0x00U,
                                    frame->sender_module_id, frame->sender_sub_id,
                                    frame->cmd_set, frame->cmd_id, 0x01U,
                                    OFFICIAL_CMD_RESPONSE, au8Data, u16DataLen);
                if (u16Len > 0U)
                {
                    (void)AppUsartTxTask_Send(au8Tx, u16Len);
                }
                u16DataLen = 0U;
            }

            u8RecSeq++;
            au8Data[u16DataLen + 0U] = u8RecSeq;
            au8Data[u16DataLen + 1U] = (uint8_t)(pSnap->matrix_no & 0xFFU);
            au8Data[u16DataLen + 2U] = (uint8_t)((pSnap->matrix_no >> 8U) & 0xFFU);
            au8Data[u16DataLen + 3U] = (uint8_t)(pSnap->angle_x100 & 0xFFU);
            au8Data[u16DataLen + 4U] = (uint8_t)((pSnap->angle_x100 >> 8U) & 0xFFU);
            au8Data[u16DataLen + 5U] = (uint8_t)(pSnap->len_x100 & 0xFFU);
            au8Data[u16DataLen + 6U] = (uint8_t)((pSnap->len_x100 >> 8U) & 0xFFU);
            au8Data[u16DataLen + 7U] = (uint8_t)(pSnap->speed_x100 & 0xFFU);
            au8Data[u16DataLen + 8U] = (uint8_t)((pSnap->speed_x100 >> 8U) & 0xFFU);
            au8Data[u16DataLen + 9U] = (uint8_t)(pSnap->cur_no & 0xFFU);
            au8Data[u16DataLen + 10U] = (uint8_t)((pSnap->cur_no >> 8U) & 0xFFU);
            u16DataLen += OFFICIAL_SKID_DATA_LEN;
        }
    }

    /* 投递最后一帧；无任何事件时仍回一帧空数据应答，保证请求必有应答 */
    {
        uint16_t u16Len = OfficialProto_BuildFrame(au8Tx, 0x00U,
                            frame->sender_module_id, frame->sender_sub_id,
                            frame->cmd_set, frame->cmd_id, 0x01U,
                            OFFICIAL_CMD_RESPONSE, au8Data, u16DataLen);
        if (u16Len > 0U)
        {
            (void)AppUsartTxTask_Send(au8Tx, u16Len);
        }
    }
}

/**
 * @brief  set_touch_sensor_cycle_sw：设置触觉信息周期上报开关（0x30/0x15，协议 v1.8）
 * @note   请求 data[0]：OFFICIAL_CYCLE_SW_ON(0x00)=开启 / OFFICIAL_CYCLE_SW_OFF(0x01)=关闭；
 *         应答 data[0] 回显**当前生效状态**（同上取值）。
 *         参数非法（数据段长度非 1 或取值非 0x00/0x01）时不改动开关、仅回显当前状态。
 *         开关上电默认关闭；开启后按节拍投递，关闭后立即停止周期投递（收口于
 *         OfficialProto_SetCycleReport，同步停掉 app_period_timer 的 8ms 上报定时器）。
 */
static void Official_HandleSetCycleSw(const official_frame_t *frame)
{
    uint8_t u8Ack;

    if ((frame->data_len == 1U) &&
        ((frame->data[0] == OFFICIAL_CYCLE_SW_ON) || (frame->data[0] == OFFICIAL_CYCLE_SW_OFF)))
    {
        /* 走统一收口：置位开关 + 启停周期上报定时器（避免漏掉启停动作） */
        OfficialProto_SetCycleReport((frame->data[0] == OFFICIAL_CYCLE_SW_ON) ? 1U : 0U);
    }

    u8Ack = (s_u8CycleReportEn != 0U) ? OFFICIAL_CYCLE_SW_ON : OFFICIAL_CYCLE_SW_OFF;
    Official_SendResponse(frame, &u8Ack, 1U);
}

/**
 * @brief  设置触觉信息周期上报开关（供 usart 任务侧调用，如升级触发时立即停发）
 * @param  u8Enable  1=开启 0=关闭
 * @note   本模块由 usart 任务单实例串行访问，无并发保护；开关状态跨上下文共享
 *         （定时器回调读取）故声明为 volatile。
 * @note   本函数为周期上报的唯一收口：置位后同步启停 app_period_timer 的 8ms 上报
 *         定时器；回调内仍会再判一次本开关，即使定时器命令因守护任务队列满未送达
 *         也保证"关了就一定不发"。
 */
void OfficialProto_SetCycleReport(uint8_t u8Enable)
{
    s_u8CycleReportEn = (u8Enable != 0U) ? 1U : 0U;
    AppPeriodTimer_SetCycleReportEn(s_u8CycleReportEn);
}

/**
 * @brief  查询触觉信息周期上报开关当前状态
 * @retval 1=开启 0=关闭
 */
uint8_t OfficialProto_GetCycleReport(void)
{
    return s_u8CycleReportEn;
}

/* ============================ 对外入口 ============================ */
/**
 * @brief  官方协议帧入口：校验 -> 模块路由 -> 命令分发 -> 应答
 * @param  buf  接收帧缓冲区
 * @param  len  帧长度
 * @retval 0U 已处理（含合法但忽略的帧）；1U 帧非法（校验失败/接收方非本机）
 * @note   分发流程：
 *         1) Official_ParseFrame 完成全部校验；
 *         2) 接收方模块必须为本机 OFFICIAL_MODULE_TS(0x30)，否则忽略；
 *         3) 子模块广播(0xFF)：全模块数据查询命令 get_ori_data(0x10) 与
 *            get_touch_sensor_skid_data(0x14) 直接响应（应答 sender_sub_id=0x00），其余忽略；
 *         4) cmd_type 为 OFFICIAL_CMD_NO_RESP 的指令不返回应答；
 *         5) 按 cmd_set -> cmd_id 两级分发到对应 Handler
 *            （receiver_sub_id=0x00 的点对点查询同样走此分发）；
 *         6) 触觉命令集 0x15 set_touch_sensor_cycle_sw 为点对点设置命令（协议示例
 *            receiver_sub_id=0x00），只落在此两级分发路径，广播路径不处理。
 */
uint8_t OfficialProto_Process(const uint8_t *buf, uint16_t len)
{
    official_frame_t frame;

    if (!Official_ParseFrame(buf, len, &frame))
    {
        return 1U;
    }
    /* 接收方模块必须是触觉传感器 */
    if (frame.receiver_module_id != OFFICIAL_MODULE_TS)
    {
        return 1U;
    }

    /* 子模块广播：多指应答命令（get_ori_data / get_touch_sensor_skid_data）在此处理，其余忽略 */
    if (frame.receiver_sub_id == OFFICIAL_SUB_BROADCAST)
    {
        if ((frame.cmd_set == OFFICIAL_CMD_SET_TACTILE) && (frame.cmd_id == OFFICIAL_CMD_GET_ORI_DATA))
        {
            Official_HandleGetOriData(&frame);
        }
        else if ((frame.cmd_set == OFFICIAL_CMD_SET_TACTILE) && (frame.cmd_id == OFFICIAL_CMD_GET_SKID_DATA))
        {
            Official_HandleGetSkidData(&frame);
        }
        return 0U;
    }

    /* 无需应答的指令类型不返回 */
    if (frame.cmd_type == OFFICIAL_CMD_NO_RESP)
    {
        return 0U;
    }

    switch (frame.cmd_set)
    {
        case OFFICIAL_CMD_SET_COMMON:
            switch (frame.cmd_id)
            {
                case OFFICIAL_CMD_GET_VERSION:  Official_HandleGetVersion(&frame);  break;
                case OFFICIAL_CMD_GET_UID:      Official_HandleGetUid(&frame);      break;
                case OFFICIAL_CMD_GET_ID:       Official_HandleGetId(&frame);       break;
                case OFFICIAL_CMD_SET_ID:       Official_HandleSetId(&frame);       break;
                case OFFICIAL_CMD_GET_VENDOR:   Official_HandleGetVendor(&frame);   break;
                case OFFICIAL_CMD_GET_HAND:     Official_HandleGetHand(&frame);     break;
                default:                                                             break;
            }
            break;

        case OFFICIAL_CMD_SET_TACTILE:
            switch (frame.cmd_id)
            {
                case OFFICIAL_CMD_GET_ORI_DATA:  Official_HandleGetOriData(&frame);  break;
#if TAC_CALIB_EN
                case OFFICIAL_CMD_GET_CALIB:     Official_HandleGetCalib(&frame);    break;
#endif
                case OFFICIAL_CMD_GET_FORCE:     Official_HandleGetForce(&frame);    break;
                case OFFICIAL_CMD_GET_STATE:     Official_HandleGetState(&frame);    break;
                case OFFICIAL_CMD_GET_SKID_DATA: Official_HandleGetSkidData(&frame); break;
                case OFFICIAL_CMD_SET_CYCLE_SW:  Official_HandleSetCycleSw(&frame);  break;
                default:                                                              break;
            }
            break;

        default:
            break;
    }

    return 0U;
}

