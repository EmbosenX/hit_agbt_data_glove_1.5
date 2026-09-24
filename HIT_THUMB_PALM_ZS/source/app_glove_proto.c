/*******************************************************************************
 * @file    app_glove_proto.c
 * @brief   灏存《上传与标定协议》协议层实现
 * @date    2026-09-16
 *
 * 调用上下文：
 *  - AppGloveProto_HandleCmdFrame：usart 接收任务上下文（3C 帧分发，单实例串行），
 *    PUT 写表涉及 Flash 整扇区擦写（HcFlash 内部临界区，约几十 ms），期间置
 *    s_u8UploadPaused 暂停 0x02 周期上传，返回前恢复（协议第 8 节）；
 *  - AppGloveProto_BuildUploadFrame：app_period_timer 定时器回调（守护任务），
 *    只读采样发布快照，与标定命令上下文经 s_u8UploadPaused 互斥。
 ******************************************************************************/
#include <string.h>
#include "app_glove_proto.h"
#include "app_force_calib.h"
#include "app_tac_config.h"
#include "app_usart_tx_task.h"
#include "hc32_ll_utility.h" /* DDL_Printf（标定表写后回读校验打印；LL_PRINT_ENABLE=DDL_OFF 时为空函数） */

/* 编译期断言：协议单区域点数上界不得超过标定核心 LUT 容量
 * （0x07 解析缓冲与运行表/LUT 数组共用同一上界，任一侧改动越界都会在此编译失败） */
typedef char glove_points_max_fit_check[(GLOVE_POINTS_MAX <= APP_FORCE_CALIB_LUT_POINTS_MAX) ? 1 : -1];

/* ============================ 内部状态 ============================ */
/* 标定命令处理中标志：usart 任务写，定时器守护任务回调读，volatile 保证可见性 */
static volatile uint8_t s_u8UploadPaused = 0U;

/* ============================ 帧层工具 ============================ */
/**
 * @brief  外层帧 CRC16（poly 0xA001，初值 0x0000，只校验 Payload）
 * @param  data  Payload 首地址
 * @param  len   Payload 长度
 * @retval CRC16 值（组帧时按小端写入帧尾）
 * @note   收（攒帧校验）发（组帧）共用同一实现，保证两侧算法同源（协议第 1 节）
 */
uint16_t AppGloveProto_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0x0000U;
    uint16_t i;
    uint8_t  bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x0001U) != 0U) ? (uint16_t)((crc >> 1U) ^ 0xA001U)
                                          : (uint16_t)(crc >> 1U);
        }
    }

    return crc;
}

/**
 * @brief  组外层帧：3C 3C | ch | flags | len(LE) | payload | crc(LE) | 3E 3E
 * @param  channel  帧 Channel 字段
 * @param  flags    帧 Flags 字段 = (package_id << 2) | method
 * @param  payload  Payload 首地址
 * @param  len      Payload 长度
 * @param  out      输出缓冲
 * @param  size     输出缓冲容量（须 >= len + 10）
 * @retval 实际帧长度；参数非法或缓冲不足返回 0
 * @note   帧尾 CRC16 只覆盖 Payload，按小端写入
 */
static uint16_t glove_build_frame(uint8_t channel, uint8_t flags,
                                  const uint8_t *payload, uint16_t len,
                                  uint8_t *out, uint16_t size)
{
    uint16_t need = (uint16_t)(len + 10U);
    uint16_t crc;
    uint16_t pos;

    if ((NULL == out) || (NULL == payload) || (size < need))
    {
        return 0U;
    }

    out[0] = 0x3CU;
    out[1] = 0x3CU;
    out[2] = channel;
    out[3] = flags;
    out[4] = (uint8_t)(len & 0xFFU);
    out[5] = (uint8_t)((len >> 8U) & 0xFFU);
		
    memcpy(&out[6], payload, len);

    crc     = AppGloveProto_Crc16(payload, len);
    pos     = (uint16_t)(6U + len);
    out[pos]      = (uint8_t)(crc & 0xFFU);
    out[pos + 1U] = (uint8_t)((crc >> 8U) & 0xFFU);
    out[pos + 2U] = 0x3EU;
    out[pos + 3U] = 0x3EU;

    return need;
}

/**
 * @brief  发送 RESPONSE 帧：组帧后入发送队列即返回（不等待发完）
 * @param  package_id  复用请求帧的 package_id
 * @param  payload     应答 Payload
 * @param  len         应答 Payload 长度（<= GLOVE_PUT_PAYLOAD_MAX）
 * @note   使用静态组帧缓冲（单实例禁重入）；组帧失败或队列已满时静默丢弃；
 *         调用上下文：usart 接收任务
 */
static void glove_send_response(uint8_t package_id, const uint8_t *payload, uint16_t len)
{
    static uint8_t sau8TxBuf[GLOVE_PUT_PAYLOAD_MAX + 10U];
    uint16_t frame_len;

    frame_len = glove_build_frame(GLOVE_CH_CALIB,
                                  (uint8_t)((package_id << 2) | GLOVE_METHOD_RESP),
                                  payload, len, sau8TxBuf, (uint16_t)sizeof(sau8TxBuf));

    if (frame_len > 0U)
    {
        (void)AppUsartTxTask_Send(sau8TxBuf, frame_len);
    }
}

/* ============================ Channel 0x07 命令 ============================ */
/**
 * @brief  处理标定表下发 PUT（协议第 4 节，一个部位一帧、整部位替换、无独立应答）
 * @param  payload  Payload 首地址
 * @param  len      Payload 长度
 * @note   Payload[0] = descriptor = (part<<4)|region_count，其后逐区域
 *         rows(1)+cols(1)+2*rows*cols 字节 uint16 小端（第一行 AD、第二行力值）。
 *         校验：part 1~6、region_count 1~3、每区域 rows==2、2<=cols<=62、
 *         Payload 长度精确吻合（1 + Σ(2+4*cols)）、AD 严格递增、力值不降；
 *         任一校验或落盘失败即放弃写入并保留旧表。
 *         调用上下文：usart 接收任务（单实例串行）；写表期间其上层已暂停 0x02 上传
 */
static void glove_handle_put_table(const uint8_t *payload, uint16_t len)
{
    static uint16_t sau16Ad[GLOVE_REGION_MAX][GLOVE_POINTS_MAX];
    static uint16_t sau16Force[GLOVE_REGION_MAX][GLOVE_POINTS_MAX];
    app_force_calib_table_t tables[GLOVE_REGION_MAX];
    uint8_t  part;
    uint8_t  region_count;
    uint16_t expect_len;
    uint16_t ofs;
    uint8_t  r;
    uint16_t i;

    if ((NULL == payload) || (len < 1U))
    {
        return;
    }

    part         = (uint8_t)(payload[0] >> 4);
    region_count = (uint8_t)(payload[0] & 0x0FU);

    if ((part < 1U) || (part > GLOVE_PART_MAX) ||
        (region_count < 1U) || (region_count > GLOVE_REGION_MAX))
    {
        return; /* 部位/区域数不在设备支持范围：不写入 */
    }

    /* 先按声明区域数核算期望长度，再逐区域解析 */
    expect_len = 1U;
    ofs        = 1U;

    for (r = 0U; r < region_count; r++)
    {
        uint16_t cols;

        if ((uint16_t)(ofs + 2U) > len)
        {
            return; /* 区域包缺失：不写入 */
        }

        if (payload[ofs] != 2U)
        {
            return; /* 当前固件只接受 rows=2 */
        }

        cols = payload[ofs + 1U];

        if ((cols < 2U) || (cols > GLOVE_POINTS_MAX))
        {
            return; /* 2<=cols<=62 */
        }

        ofs        = (uint16_t)(ofs + 2U + (4U * cols));
        expect_len = (uint16_t)(expect_len + 2U + (4U * cols));
    }

    if ((ofs != len) || (expect_len != len))
    {
        return; /* 尾部多余字节或 Length 不符：不写入 */
    }

    /* 逐区域解析：区域数据按行连续排列（协议 4.2），rows=2 时先整行 AD 再整行力值，
       每项 uint16 小端；不是「AD/力值逐点交错」 */
    ofs = 1U;

    for (r = 0U; r < region_count; r++)
    {
        uint16_t cols = payload[ofs + 1U];

        ofs += 2U;

        for (i = 0U; i < cols; i++)
        {
            sau16Ad[r][i] = (uint16_t)(payload[ofs] | ((uint16_t)payload[ofs + 1U] << 8));
            ofs           = (uint16_t)(ofs + 2U);
        }

        for (i = 0U; i < cols; i++)
        {
            sau16Force[r][i] = (uint16_t)(payload[ofs] | ((uint16_t)payload[ofs + 1U] << 8));
            ofs              = (uint16_t)(ofs + 2U);
        }

        tables[r].adc   = sau16Ad[r];
        tables[r].force = sau16Force[r];
        tables[r].count = cols;
    }

    /* AD 严格递增/力值不降/持久化由标定核心校验；失败保留旧表且无应答（协议语义不变） */
    {
        bool bSaved = AppForceCalib_SetPart(part, tables, region_count);

#if (APP_FORCE_CALIB_FLASH_TEST_EN != 0U)
        if (true == bSaved)
        {
            /* 写表成功：立即从 Flash 回读校验并打印完整 LUT（便于与下发帧逐值核对） */
            DDL_Printf("\r\n[FC] PUT table accepted: part=%u regions=%u payload=%uB\r\n",
                       (unsigned int)part, (unsigned int)region_count, (unsigned int)len);
            AppForceCalib_DumpPart(part);
        }
        else
        {
            DDL_Printf("\r\n[FC] PUT table REJECTED: part=%u regions=%u payload=%uB "
                       "(invalid LUT, keep old table)\r\n",
                       (unsigned int)part, (unsigned int)region_count, (unsigned int)len);
        }
#else
        (void)bSaved;
#endif
    }
}

/**
 * @brief  处理标定状态查询 GET（协议第 5 节）
 * @param  package_id  请求帧 package_id（应答复用）
 * @param  payload     Payload 首地址（须为 00 + selector 两字节）
 * @param  len         Payload 长度（须为 2）
 * @note   selector 0=全部部位、1~6=单部位；
 *         应答 = 0x80 | count | count×(descriptor + mask(u16 LE))，
 *         其中 descriptor 低四位为该部位已存储区域数，mask 的 bit(r-1)=区域 r 有效
 */
static void glove_handle_get_status(uint8_t package_id, const uint8_t *payload, uint16_t len)
{
    static uint8_t sau8Resp[2U + (3U * GLOVE_PART_MAX)];
    uint8_t selector;
    uint8_t first, last, part;
    uint8_t count = 0U;
    uint8_t region_count = 0U;
    uint16_t mask = 0U;
    uint8_t ofs;

    if ((NULL == payload) || (len != 2U) || (payload[0] != 0x00U))
    {
        return; /* 仅支持 00 selector 状态查询 */
    }

    selector = payload[1];

    if (selector == 0x00U)
    {
        first = 1U;
        last  = GLOVE_PART_MAX;
    }
    else if (selector <= GLOVE_PART_MAX)
    {
        first = selector;
        last  = selector;
    }
    else
    {
        return; /* selector 非法 */
    }

    sau8Resp[0] = 0x80U;

    for (part = first; part <= last; part++)
    {
        region_count = 0U;
        mask         = 0U;

        if (false == AppForceCalib_GetStatus(part, &region_count, &mask))
        {
            return; /* 内部参数异常：不输出残缺应答 */
        }

        ofs = (uint8_t)(2U + (3U * count));

        sau8Resp[ofs]      = (uint8_t)((part << 4) | (region_count & 0x0FU)); /* 低四位=已存储区域数 */
        sau8Resp[ofs + 1U] = (uint8_t)(mask & 0xFFU);                        /* mask 低字节 */
        sau8Resp[ofs + 2U] = (uint8_t)((mask >> 8U) & 0xFFU);                /* mask 高字节，恒 0x00 */

        count++;
    }

    sau8Resp[1] = count;

    glove_send_response(package_id, sau8Resp, (uint16_t)(2U + (3U * count)));
}

/**
 * @brief  3C 命令帧入口（接口说明见 app_glove_proto.h）
 * @note   仅处理 Channel 0x07；处理期间置 s_u8UploadPaused 暂停 0x02 周期上传，
 *         返回前恢复；PUT 写表含整扇区 Flash 擦写（约几十 ms）；
 *         调用上下文：usart 接收任务（单实例串行）
 */
void AppGloveProto_HandleCmdFrame(uint8_t channel, uint8_t flags,
                                  const uint8_t *payload, uint16_t len)
{
    uint8_t method;
    uint8_t package_id;

    if (channel != GLOVE_CH_CALIB)
    {
        return; /* 仅实现标定/查询/擦除通道，其余忽略 */
    }

    /* 标定命令处理期间暂停 0x02 周期上传，返回前恢复（含 PUT 写 Flash 期间） */
    s_u8UploadPaused = 1U;

    method     = (uint8_t)(flags & 0x03U);
    package_id = (uint8_t)(flags >> 2);

    if (GLOVE_METHOD_PUT == method)
    {
        if ((NULL != payload) && (len == 2U) && (payload[0] == 0x01U) &&
            (payload[1] >= 1U) && (payload[1] <= GLOVE_PART_MAX))
        {
            /* 擦除：01 part_id，无独立应答（协议第 6 节） */
            (void)AppForceCalib_ClearPart(payload[1]);
        }
        else
        {
            glove_handle_put_table(payload, len);
        }
    }
    else if (GLOVE_METHOD_GET == method)
    {
        glove_handle_get_status(package_id, payload, len);
    }
    else
    {
        /* RESPONSE 等其余 method 不处理 */
    }

    s_u8UploadPaused = 0U;
}

/* ============================ Channel 0x02 上传 ============================ */
/**
 * @brief  组一帧 Channel 0x02 部位上传帧（接口说明见 app_glove_proto.h）
 * @note   使用静态 Payload 缓冲（单实例禁重入）；数据取自采样发布快照（读侧免锁）；
 *         调用上下文：app_period_timer 定时器回调（守护任务），与标定命令上下文
 *         经 AppGloveProto_UploadPaused 互斥
 */
uint16_t AppGloveProto_BuildUploadFrame(uint8_t part, uint8_t *buf, uint16_t size)
{
    /* data 区按有效点数分配：掩码为 0 的无效点不占位、不发送 */
    static uint8_t sau8Payload[4U + APP_FORCE_CALIB_POINTS_MAX];
    const tac_data_cfg_t *snap;
    uint8_t rows, cols;
    uint16_t payload_len;
    uint16_t point_num;

    rows = AppForceCalib_PartRows(part);
    cols = AppForceCalib_PartCols(part);

    /* rows/cols 仅作 part 合法性判断与帧字段，不参与长度计算 */
    if ((rows == 0U) || (cols == 0U) || (NULL == buf))
    {
        return 0U;
    }

    /* data 区：有效点按编号升序连续排列（掩码为 0 的点不占位），N = 有效点数 */
    snap = TacConfig_GetSnapshot();
    point_num = AppForceCalib_LoadPartStream(snap, part, &sau8Payload[4U],
                                            (uint16_t)(sizeof(sau8Payload) - 4U));

    if (0U == point_num)
    {
        return 0U;   /* 装配失败不发帧，沿用"丢弃本帧、不补发"语义 */
    }

    sau8Payload[0] = (uint8_t)((part << 4) | 1U);  /* descriptor：低四位固定 1 */
    sau8Payload[1] = 0x01U;                        /* format：每点 1 字节 */
    sau8Payload[2] = rows;                         /* y 方向点数（物理矩阵尺寸） */
    sau8Payload[3] = cols;                         /* x 方向点数（物理矩阵尺寸） */

    payload_len = (uint16_t)(4U + point_num);   /* 长度按有效点数 N，非 rows×cols */

    /* 外层帧固定开销 10 字节：3C 3C | ch | flags | len(2) | ... | crc(2) | 3E 3E */
    if (size < (uint16_t)(payload_len + 10U))
    {
        return 0U;
    }

    return glove_build_frame(GLOVE_CH_UPLOAD, 0x00U, sau8Payload, payload_len, buf, size);
}

/**
 * @brief  查询"标定命令处理中"（1=暂停 0x02 周期上传）
 * @retval 1=暂停 0=正常
 * @note   接口说明见 app_glove_proto.h；标定命令上下文写、定时器回调读
 */
uint8_t AppGloveProto_UploadPaused(void)
{
    return s_u8UploadPaused;
}

