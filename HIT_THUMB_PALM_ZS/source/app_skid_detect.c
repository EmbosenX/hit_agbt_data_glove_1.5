/*******************************************************************************
 * @file        app_skid_detect.c
 * @brief       触觉滑移检测算法实现
 * @author      
 * @date        2026-09-02
 * @version     V2.0
 * @note        对 15 个节级触觉区域独立跟踪接触部位，识别滑移事件；
 *              每个区域每帧先对接触点做 8 邻域连通域聚类，把空间上独立的
 *              接触部位分成多个簇（最多 SKID_TRACKER_PER_AREA 个），各簇
 *              独立完成"起点记录→滑移判定→方向/长度累积→事件结束"状态机；
 *              单点接触时聚类为 1 簇，行为与 V1.0 单部位跟踪一致。
 *              事件结束时把最终快照（节位/矩阵编号/方向角/滑移长度/平均速度）
 *              写入所属手指的事件槽，供 0x14 滑移命令应答读取。
 *              编号映射：GetTacMaskData() 返回各节独立行优先的局部编号表
 *              （0=无效点），写入快照前叠加节首全局偏移换算为协议附录5
 *              全局编号 1~750，保证上位机按编号直接定位。
 ******************************************************************************/
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "app_skid_detect.h"

/*******************************************************************************
 * 私有宏定义
 ******************************************************************************/
/* 接触阈值：区域内点值/AD 增量超过该值判定为接触点（0~255，按数据域设置）。
   注意：输出单位改为 APP_FORCE_CALIB_OUT_UNIT_DIV（默认 2 即 2g 单位）后，本阈值
   的物理含义同步放大同样倍数；启用滑移检测（Skid_Update）时需重新标定本阈值。
   当前 Skid_Update 未被调用（见 app_sample_task.c），阈值不产生实际效果 */
#define SKID_CONTACT_TH         5U
/* 滑移起始位移阈值（step×100，默认 0.5 step）：降低以提升触发灵敏度，
 * 较短/较慢的滑移也能在位移较小处即判定为滑移 */
#define SKID_START_TH           50U
/* 停滞判定：帧间位移小于该值（step×100，0.1 step）连续达到阈值帧数即事件结束 */
#define SKID_STALL_TH           10U
#define SKID_STALL_FRAMES       5U
/* 簇-跟踪器跨帧关联最大距离（step）：帧间质心位移通常 <1 step，设 4 step
 * 阈值可避免"旧簇消失+新簇出现"被误判为同一事件延续 */
#define SKID_MATCH_TH           4.0f
/* 输出上限保护 */
#define SKID_LEN_MAX            65535U
#define SKID_ANGLE_MAX          36000U

/* 状态机状态 */
#define SKID_STATE_IDLE         0U
#define SKID_STATE_TRACKING     1U

/* 聚类工作缓冲最大行列（取最大区域：掌心 15×16=240 点） */
#define SKID_AREA_ROW_MAX       15U
#define SKID_AREA_COL_MAX       16U

/* 各手指事件槽数 = 节数 × SKID_TRACKER_PER_AREA：
 * 拇指 2 节=8、食指/中指/无名指/小指 3 节=12、掌心 1 节=4 */
static const uint8_t s_au8SlotCnt[TAC_FINGER_MAX] = { 8U, 12U, 12U, 12U, 12U, 4U };

/* 各节首点全局编号偏移（协议附录5 编号 1~750，数组下标 = tac_area_t 枚举序）：
 * 拇指远 1~59 / 拇指近 60~101 / 食指远 102~159 / 食指中 160~195 / 食指近 196~231
 * 中指远 232~290 / 中指中 291~329 / 中指近 330~368 / 无名指远 369~426
 * 无名指中 427~462 / 无名指近 463~498 / 小指远 499~543 / 小指中 544~576
 * 小指近 577~609 / 掌心 610~750 */
static const uint16_t s_au16AreaGlobalOffset[TAC_AREA_MAX] = {
    0U, 59U, 101U, 159U, 195U, 231U, 290U, 329U, 368U, 426U, 462U, 498U, 543U, 576U, 609U
};

/*******************************************************************************
 * 私有数据类型
 ******************************************************************************/
/* 区域内单帧聚类结果：一个独立接触部位 */
typedef struct {
    uint16_t u16Cnt;      /* 簇内接触点数（用于面积优先排序） */
    float    fR;          /* 簇加权质心（行） */
    float    fC;          /* 簇加权质心（列） */
} skid_cluster_t;

typedef struct {
    uint8_t  u8State;        /* 状态机状态 */
    uint8_t  u8Slid;         /* 是否已判定滑移（起点位移≥SKID_START_TH） */
    uint8_t  u8StallCnt;     /* 停滞连续帧计数 */
    uint8_t  u8FrameCnt;     /* 跟踪持续采样帧数（用于平均速度） */
    uint16_t u16LenX100;     /* 事件累积路径总长（step×100） */
    uint16_t u16AngleX100;   /* 当前方向角（0.01°） */
    uint16_t u16StartNo;     /* 事件起点矩阵编号（协议附录5 全局号 1~750） */
    uint16_t u16CurNo;       /* 滑移中当前质心矩阵编号（协议附录5 全局号 1~750，未滑移=0） */
    float    fStartR;        /* 事件起点质心（行） */
    float    fStartC;        /* 事件起点质心（列） */
    float    fPrevR;         /* 上一帧质心（行） */
    float    fPrevC;         /* 上一帧质心（列） */
} skid_tracker_t;

/*******************************************************************************
 * 私有全局变量
 ******************************************************************************/
static skid_tracker_t  s_astTracker[TAC_AREA_MAX][SKID_TRACKER_PER_AREA];
static skid_snapshot_t s_astSnap[TAC_FINGER_MAX][SKID_EVENT_MAX];
/* 聚类工作缓冲（每帧仅处理一个区域，可复用；BFS 队列容量=最大区域点数 240） */
static uint8_t         s_au8Label[SKID_AREA_ROW_MAX][SKID_AREA_COL_MAX];
static uint16_t        s_au16Queue[SKID_AREA_ROW_MAX * SKID_AREA_COL_MAX];

/*******************************************************************************
 * 静态内部函数声明
 ******************************************************************************/
static uint8_t  Skid_AreaRowNum(tac_area_t area);
static uint8_t  Skid_AreaColNum(tac_area_t area);
static uint8_t  Skid_AreaPhalanx(tac_area_t area);
static uint8_t  Skid_AreaSlotBase(tac_area_t area);
static uint8_t  Skid_CopySnap(uint8_t sub_id, skid_snapshot_t *out, uint8_t max_cnt);
static uint16_t Skid_GetData(const tac_data_cfg_t *p, tac_area_t area, uint16_t u16Row, uint16_t u16Col);
static uint8_t  Skid_ClusterContacts(const tac_data_cfg_t *p, tac_area_t area, skid_cluster_t *out);
static uint16_t Skid_FindStartNo(const tac_data_cfg_t *mask, tac_area_t area, float fR, float fC);
static void     Skid_WriteSnap(tac_area_t area, uint8_t u8SubIdx, const skid_tracker_t *t);
static void     Skid_EndEvent(tac_area_t area, uint8_t u8SubIdx, skid_tracker_t *t);
static void     Skid_UpdateTracker(const tac_data_cfg_t *mask, tac_area_t area, uint8_t u8SubIdx,
                                   skid_tracker_t *t, float fR, float fC);
static void     Skid_UpdateArea(const tac_data_cfg_t *p, const tac_data_cfg_t *mask, tac_area_t area);

/*******************************************************************************
 * 静态内部函数实现
 ******************************************************************************/
/**
 * @brief  取区域行数（按 TacAreaRange 的行分段推导）
 * @param  area  节级区域枚举
 * @retval 行数
 */
static uint8_t Skid_AreaRowNum(tac_area_t area)
{
    return (uint8_t)(TacAreaRange[area].row_end - TacAreaRange[area].row_start + 1U);
}

/**
 * @brief  取区域列数（按 TacAreaRange 的列分段推导）
 * @param  area  节级区域枚举
 * @retval 列数
 */
static uint8_t Skid_AreaColNum(tac_area_t area)
{
    return (uint8_t)(TacAreaRange[area].col_end - TacAreaRange[area].col_start + 1U);
}

/**
 * @brief  取区域的节位值
 * @param  area  节级区域枚举
 * @retval 节位值：DIP→1、PIP→2、MCP→3、PALM→1
 */
static uint8_t Skid_AreaPhalanx(tac_area_t area)
{
    switch (area) {
        case TAC_AREA_THUMB_DIP:
        case TAC_AREA_INDEX_DIP:
        case TAC_AREA_MIDDLE_DIP:
        case TAC_AREA_RING_DIP:
        case TAC_AREA_LITTLE_DIP:
        case TAC_AREA_PALM:
            return 1U;   /* 远节（掌心=1） */
        case TAC_AREA_INDEX_PIP:
        case TAC_AREA_MIDDLE_PIP:
        case TAC_AREA_RING_PIP:
        case TAC_AREA_LITTLE_PIP:
            return 2U;   /* 中节 */
        case TAC_AREA_THUMB_MCP:
        case TAC_AREA_INDEX_MCP:
        case TAC_AREA_MIDDLE_MCP:
        case TAC_AREA_RING_MCP:
        case TAC_AREA_LITTLE_MCP:
            return 3U;   /* 近节 */
        default:
            return 0U;
    }
}

/**
 * @brief  取区域在其所属手指内的槽位基址（节序号 × SKID_TRACKER_PER_AREA）
 * @param  area  节级区域枚举
 * @retval 槽位基址：拇指 DIP/MCP=0/4、其余手指 DIP/PIP/MCP=0/4/8、掌心=0；
 *         实际槽号 = 基址 + 节内部位序号(0~3)，对应协议 seq_num = 槽号+1
 */
static uint8_t Skid_AreaSlotBase(tac_area_t area)
{
    switch (area) {
        case TAC_AREA_THUMB_DIP:        return 0U;
        case TAC_AREA_THUMB_MCP:        return (uint8_t)(1U * SKID_TRACKER_PER_AREA);
        case TAC_AREA_INDEX_DIP:        return 0U;
        case TAC_AREA_INDEX_PIP:        return (uint8_t)(1U * SKID_TRACKER_PER_AREA);
        case TAC_AREA_INDEX_MCP:        return (uint8_t)(2U * SKID_TRACKER_PER_AREA);
        case TAC_AREA_MIDDLE_DIP:       return 0U;
        case TAC_AREA_MIDDLE_PIP:       return (uint8_t)(1U * SKID_TRACKER_PER_AREA);
        case TAC_AREA_MIDDLE_MCP:       return (uint8_t)(2U * SKID_TRACKER_PER_AREA);
        case TAC_AREA_RING_DIP:         return 0U;
        case TAC_AREA_RING_PIP:         return (uint8_t)(1U * SKID_TRACKER_PER_AREA);
        case TAC_AREA_RING_MCP:         return (uint8_t)(2U * SKID_TRACKER_PER_AREA);
        case TAC_AREA_LITTLE_DIP:       return 0U;
        case TAC_AREA_LITTLE_PIP:       return (uint8_t)(1U * SKID_TRACKER_PER_AREA);
        case TAC_AREA_LITTLE_MCP:       return (uint8_t)(2U * SKID_TRACKER_PER_AREA);
        case TAC_AREA_PALM:             return 0U;
        default:                        return 0U;
    }
}

/**
 * @brief  读取区域内指定行列的传感点数值
 * @param  p       采集快照/工作区（矩阵指针按 DATA_FRAME_BYTE_NUM 解释）
 * @param  area    节级区域枚举
 * @param  u16Row  区域内的行坐标（0 起）
 * @param  u16Col  区域内的列坐标（0 起）
 * @retval 该点数值（1 字节帧格式为 0~255，2 字节帧格式为原始 uint16）
 * @note   兼容 DATA_FRAME_BYTE_NUM 的单字节/双字节两种帧格式
 */
static uint16_t Skid_GetData(const tac_data_cfg_t *p, tac_area_t area, uint16_t u16Row, uint16_t u16Col)
{
    switch (area) {
        case TAC_AREA_THUMB_DIP:    return p->thumb_dip[u16Row][u16Col];
        case TAC_AREA_THUMB_MCP:    return p->thumb_mcp[u16Row][u16Col];
        case TAC_AREA_INDEX_DIP:    return p->index_dip[u16Row][u16Col];
        case TAC_AREA_INDEX_PIP:    return p->index_pip[u16Row][u16Col];
        case TAC_AREA_INDEX_MCP:    return p->index_mcp[u16Row][u16Col];
        case TAC_AREA_MIDDLE_DIP:   return p->middle_dip[u16Row][u16Col];
        case TAC_AREA_MIDDLE_PIP:   return p->middle_pip[u16Row][u16Col];
        case TAC_AREA_MIDDLE_MCP:   return p->middle_mcp[u16Row][u16Col];
        case TAC_AREA_RING_DIP:     return p->ring_dip[u16Row][u16Col];
        case TAC_AREA_RING_PIP:     return p->ring_pip[u16Row][u16Col];
        case TAC_AREA_RING_MCP:     return p->ring_mcp[u16Row][u16Col];
        case TAC_AREA_LITTLE_DIP:   return p->little_dip[u16Row][u16Col];
        case TAC_AREA_LITTLE_PIP:   return p->little_pip[u16Row][u16Col];
        case TAC_AREA_LITTLE_MCP:   return p->little_mcp[u16Row][u16Col];
        case TAC_AREA_PALM:         return p->palm[u16Row][u16Col];
        default:                    return 0U;
    }
}

/**
 * @brief  把一个簇插入"面积优先"（点数降序）的 Top 列表
 * @param  top        Top 列表（长度 >= SKID_TRACKER_PER_AREA，原地更新）
 * @param  u8TopCnt   插入前的簇数（0 ~ SKID_TRACKER_PER_AREA）
 * @param  c          待插入的簇
 * @retval 插入后的簇数（列表满时保持 SKID_TRACKER_PER_AREA，且只保留更大的簇）
 */
static uint8_t Skid_InsertCluster(skid_cluster_t *top, uint8_t u8TopCnt, const skid_cluster_t *c)
{
    uint8_t u8Idx;

    if (u8TopCnt < SKID_TRACKER_PER_AREA) {
        /* 列表未满：放末尾后按点数上浮 */
        u8Idx = u8TopCnt;
        while ((u8Idx > 0U) && (top[u8Idx - 1U].u16Cnt < c->u16Cnt)) {
            top[u8Idx] = top[u8Idx - 1U];
            u8Idx--;
        }
        top[u8Idx] = *c;
        return (uint8_t)(u8TopCnt + 1U);
    }
    /* 已满：新簇点数不大于当前最小者则忽略 */
    if (c->u16Cnt <= top[SKID_TRACKER_PER_AREA - 1U].u16Cnt) {
        return u8TopCnt;
    }
    u8Idx = (uint8_t)(SKID_TRACKER_PER_AREA - 1U);
    while ((u8Idx > 0U) && (top[u8Idx - 1U].u16Cnt < c->u16Cnt)) {
        top[u8Idx] = top[u8Idx - 1U];
        u8Idx--;
    }
    top[u8Idx] = *c;
    return u8TopCnt;
}

/**
 * @brief  区域内接触点 8 邻域连通域聚类
 * @param  p     采集快照/工作区（只读）
 * @param  area  节级区域枚举
 * @param  out   输出簇列表（容量 >= SKID_TRACKER_PER_AREA）
 * @retval 簇数（0 = 区域无触点）
 * @note   BFS 标记各连通簇并累计克重加权质心，只输出面积最大的
 *         SKID_TRACKER_PER_AREA 个簇；使用文件级静态聚类缓冲，
 *         单实例禁重入（ISR 与任务不得并发调用）
 */
static uint8_t Skid_ClusterContacts(const tac_data_cfg_t *p, tac_area_t area, skid_cluster_t *out)
{
    uint8_t  u8RowNum = Skid_AreaRowNum(area);
    uint8_t  u8ColNum = Skid_AreaColNum(area);
    uint16_t u16Row, u16Col;
    uint8_t  u8TopCnt = 0U;

    memset(s_au8Label, 0, sizeof(s_au8Label));

    for (u16Row = 0U; u16Row < u8RowNum; u16Row++) {
        for (u16Col = 0U; u16Col < u8ColNum; u16Col++) {
            uint16_t u16Head, u16Tail;
            uint32_t u32WSum = 0U;
            skid_cluster_t stcClu;

            /* 已标记或非接触点，跳过 */
            if ((s_au8Label[u16Row][u16Col] != 0U) ||
                (Skid_GetData(p, area, u16Row, u16Col) <= SKID_CONTACT_TH)) {
                continue;
            }

            /* BFS 收集该连通簇 */
            memset(&stcClu, 0, sizeof(stcClu));
            s_au8Label[u16Row][u16Col] = 1U;
            s_au16Queue[0] = (uint16_t)(u16Row * u8ColNum + u16Col);
            u16Head = 0U;
            u16Tail = 1U;

            while (u16Head < u16Tail) {
                uint16_t u16Idx = s_au16Queue[u16Head++];
                uint16_t u16R = u16Idx / u8ColNum;
                uint16_t u16C = u16Idx % u8ColNum;
                int8_t s8Dr, s8Dc;
                uint16_t u16W = Skid_GetData(p, area, u16R, u16C);

                stcClu.u16Cnt++;
                u32WSum += u16W;
                stcClu.fR += (float)u16R * (float)u16W;
                stcClu.fC += (float)u16C * (float)u16W;

                /* 8 邻域扩展 */
                for (s8Dr = -1; s8Dr <= 1; s8Dr++) {
                    for (s8Dc = -1; s8Dc <= 1; s8Dc++) {
                        int16_t s16NR, s16NC;
                        if ((s8Dr == 0) && (s8Dc == 0)) {
                            continue;
                        }
                        s16NR = (int16_t)u16R + s8Dr;
                        s16NC = (int16_t)u16C + s8Dc;
                        if ((s16NR < 0) || (s16NR >= (int16_t)u8RowNum) ||
                            (s16NC < 0) || (s16NC >= (int16_t)u8ColNum)) {
                            continue;
                        }
                        if ((s_au8Label[s16NR][s16NC] != 0U) ||
                            (Skid_GetData(p, area, (uint16_t)s16NR, (uint16_t)s16NC) <= SKID_CONTACT_TH)) {
                            continue;
                        }
                        s_au8Label[s16NR][s16NC] = 1U;
                        s_au16Queue[u16Tail++] = (uint16_t)((uint16_t)s16NR * u8ColNum + (uint16_t)s16NC);
                    }
                }
            }

            /* 归一化为克重加权质心 */
            if (u32WSum > 0U) {
                stcClu.fR /= (float)u32WSum;
                stcClu.fC /= (float)u32WSum;
            } else {
                stcClu.fR /= (float)stcClu.u16Cnt;
                stcClu.fC /= (float)stcClu.u16Cnt;
            }

            /* 面积优先插入 Top 列表 */
            u8TopCnt = Skid_InsertCluster(out, u8TopCnt, &stcClu);
        }
    }
    return u8TopCnt;
}

/**
 * @brief  求滑移事件起点编号（协议附录5 全局编号 1~750）
 * @param  mask  点位掩码矩阵（只读）
 * @param  area  节级区域枚举
 * @param  fR    质心行坐标（区域内浮点坐标）
 * @param  fC    质心列坐标（区域内浮点坐标）
 * @retval 起点点的全局编号；找不到有效点时返回 0
 * @note   优先取质心四舍五入点，该点无效则取距质心最近的有效点
 */
static uint16_t Skid_FindStartNo(const tac_data_cfg_t *mask, tac_area_t area, float fR, float fC)
{
    uint8_t  u8RowNum = Skid_AreaRowNum(area);
    uint8_t  u8ColNum = Skid_AreaColNum(area);
    uint16_t u16Row, u16Col;
    uint16_t u16R0 = (uint16_t)(fR + 0.5f);
    uint16_t u16C0 = (uint16_t)(fC + 0.5f);
    uint16_t u16No;
    uint16_t u16BestNo = 0U;
    uint32_t u32BestDist = 0xFFFFFFFFUL;

    if ((u16R0 < u8RowNum) && (u16C0 < u8ColNum)) {
        u16No = Skid_GetData(mask, area, u16R0, u16C0);
        if (u16No != 0U) {
            return u16No;
        }
    }
    /* 兜底：区域全表扫描找最近有效点 */
    for (u16Row = 0U; u16Row < u8RowNum; u16Row++) {
        for (u16Col = 0U; u16Col < u8ColNum; u16Col++) {
            uint32_t u32R, u32C, u32Dist;
            u16No = Skid_GetData(mask, area, u16Row, u16Col);
            if (u16No == 0U) {
                continue;
            }
            u32R = (u16Row > u16R0) ? ((uint32_t)(u16Row - u16R0)) : ((uint32_t)(u16R0 - u16Row));
            u32C = (u16Col > u16C0) ? ((uint32_t)(u16Col - u16C0)) : ((uint32_t)(u16C0 - u16Col));
            u32Dist = u32R * u32R + u32C * u32C;
            if (u32Dist < u32BestDist) {
                u32BestDist = u32Dist;
                u16BestNo = u16No;
            }
        }
    }
    return u16BestNo;
}

/**
 * @brief  把跟踪器当前状态写入所属手指的快照槽
 * @param  area      节级区域枚举
 * @param  u8SubIdx  节内部位序号（0~3）
 * @param  t         跟踪器（只读）
 * @note   槽号 = 节基址(Skid_AreaSlotBase) + u8SubIdx；
 *         滑移过程中每帧实时刷新与事件结束时落最终值共用本函数
 */
static void Skid_WriteSnap(tac_area_t area, uint8_t u8SubIdx, const skid_tracker_t *t)
{
    tac_finger_t    finger = area_to_finger(area);
    uint8_t         u8Slot = (uint8_t)(Skid_AreaSlotBase(area) + u8SubIdx);
    skid_snapshot_t *pSnap = &s_astSnap[finger][u8Slot];

    /* matrix_no/cur_no 均已在换算处加节首全局偏移，为协议附录5 全局编号 1~750 */
    pSnap->phalanx    = Skid_AreaPhalanx(area);
    pSnap->matrix_no  = t->u16StartNo;
    pSnap->cur_no     = t->u16CurNo;
    pSnap->angle_x100 = t->u16AngleX100;
    pSnap->len_x100   = t->u16LenX100;
    /* 平均速度 = 累积总长 / 持续帧数（0.01 step/采样帧） */
    pSnap->speed_x100 = (t->u8FrameCnt != 0U)
                        ? (uint16_t)((uint32_t)t->u16LenX100 / t->u8FrameCnt)
                        : 0U;
}

/**
 * @brief  结束一个滑移事件：按判定结果写快照或清槽
 * @param  area      节级区域枚举
 * @param  u8SubIdx  节内部位序号（0~3）
 * @param  t         跟踪器（读写：状态复位并置空闲）
 * @note   已判定为滑移则写入快照（含当前质心末点），否则只清零该槽
 */
static void Skid_EndEvent(tac_area_t area, uint8_t u8SubIdx, skid_tracker_t *t)
{
    tac_finger_t    finger = area_to_finger(area);
    uint8_t         u8Slot = (uint8_t)(Skid_AreaSlotBase(area) + u8SubIdx);
    skid_snapshot_t *pSnap = &s_astSnap[finger][u8Slot];

    if (t->u8Slid) {
        Skid_WriteSnap(area, u8SubIdx, t);
    } else {
        memset(pSnap, 0, sizeof(*pSnap));
    }
    t->u8State      = SKID_STATE_IDLE;
    t->u8Slid       = 0U;
    t->u8StallCnt   = 0U;
    t->u8FrameCnt   = 0U;
    t->u16StartNo   = 0U;
    t->u16CurNo     = 0U;
    t->u16AngleX100 = 0U;
    t->u16LenX100   = 0U;
}

/**
 * @brief  单簇 TRACKING 状态机更新（路径长/方向角/停滞判定）
 * @param  mask      点位掩码矩阵（只读）
 * @param  area      节级区域枚举
 * @param  u8SubIdx  节内部位序号（0~3）
 * @param  t         跟踪器（读写）
 * @param  fR        本帧质心行坐标（区域内浮点坐标）
 * @param  fC        本帧质心列坐标（区域内浮点坐标）
 * @note   累计路径长超过阈值且方向一致时判定滑移成立，事件可持续刷新；
 *         停滞超过阈值或簇消失则由 Skid_EndEvent 收尾
 */
static void Skid_UpdateTracker(const tac_data_cfg_t *mask, tac_area_t area, uint8_t u8SubIdx,
                               skid_tracker_t *t, float fR, float fC)
{
    float    fDx = fC - t->fPrevC;
    float    fDy = fR - t->fPrevR;
    uint16_t u16Seg = (uint16_t)(sqrtf(fDx * fDx + fDy * fDy) * 100.0f + 0.5f);
    uint32_t u32Len = (uint32_t)t->u16LenX100 + u16Seg;
    float    fSx = fC - t->fStartC;
    float    fSy = fR - t->fStartR;
    float    fDisp = sqrtf(fSx * fSx + fSy * fSy) * 100.0f;   /* 相对起点位移 step×100 */

    /* 跟踪持续帧数递增（EndEvent 用于平均速度，防溢出按 255 封顶不影响比值） */
    if (t->u8FrameCnt < 0xFFU) {
        t->u8FrameCnt++;
    }

    /* 滑移判定：相对起点位移达到起始阈值 */
    if ((!t->u8Slid) && (fDisp >= (float)SKID_START_TH)) {
        t->u8Slid = 1U;
    }
    /* 方向角：起点→当前质心矢量（0°=列正方向，90°=行正方向） */
    if (t->u8Slid) {
        float fAngle = atan2f(fSy, fSx) * 57.2957795f;   /* ×180/π */
        if (fAngle < 0.0f) {
            fAngle += 360.0f;
        } else if (fAngle >= 360.0f) {
            fAngle -= 360.0f;
        }
        t->u16AngleX100 = (uint16_t)(fAngle * 100.0f + 0.5f);
        if (t->u16AngleX100 > SKID_ANGLE_MAX) {
            t->u16AngleX100 = SKID_ANGLE_MAX;
        }
    }
    /* 累积路径总长 */
    if (u32Len >= SKID_LEN_MAX) {
        u32Len = SKID_LEN_MAX;
    }
    t->u16LenX100 = (uint16_t)u32Len;
    t->fPrevR = fR;
    t->fPrevC = fC;

    /* 滑移进行中：把当前质心换算为全局编号并实时刷新快照槽（matrix_no 保持起点作为事件身份，
     * cur_no 每帧更新），使上位机 0x14 轮询能连续读到移动轨迹上的各点 */
    if (t->u8Slid) {
        uint16_t u16Cur = Skid_FindStartNo(mask, area, fR, fC);
        t->u16CurNo = (u16Cur == 0U) ? 0U : (uint16_t)(u16Cur + s_au16AreaGlobalOffset[area]);
        Skid_WriteSnap(area, u8SubIdx, t);
    }

    /* 停滞判定：帧间位移小于阈值连续 N 帧 → 事件结束 */
    if (u16Seg < SKID_STALL_TH) {
        if (++t->u8StallCnt >= SKID_STALL_FRAMES) {
            Skid_EndEvent(area, u8SubIdx, t);
        }
    } else {
        t->u8StallCnt = 0U;
    }
}

/**
 * @brief  单区域多部位状态机更新
 * @param  p     本轮成品帧（只读）
 * @param  mask  点位掩码矩阵（只读）
 * @param  area  节级区域枚举
 * @note   流程：1) 聚类得到至多 SKID_TRACKER_PER_AREA 个簇；
 *         2) 簇与活跃跟踪器做贪心最近邻匹配（距离 < SKID_MATCH_TH 防误关联）；
 *         3) 匹配到的簇继续状态机更新，新簇占用空闲跟踪器启动事件，丢失的簇结束事件
 */
static void Skid_UpdateArea(const tac_data_cfg_t *p, const tac_data_cfg_t *mask, tac_area_t area)
{
    skid_cluster_t stcClu[SKID_TRACKER_PER_AREA];
    uint8_t u8CluCnt = Skid_ClusterContacts(p, area, stcClu);
    uint8_t u8TrkMatch[SKID_TRACKER_PER_AREA];
    uint8_t u8CluMatch[SKID_TRACKER_PER_AREA];
    uint8_t u8SubIdx, u8CluIdx;

    memset(u8TrkMatch, 0, sizeof(u8TrkMatch));
    memset(u8CluMatch, 0, sizeof(u8CluMatch));

    /* 无接触：区域内所有活跃跟踪器结束事件 */
    if (u8CluCnt == 0U) {
        for (u8SubIdx = 0U; u8SubIdx < SKID_TRACKER_PER_AREA; u8SubIdx++) {
            skid_tracker_t *t = &s_astTracker[area][u8SubIdx];
            if (t->u8State == SKID_STATE_TRACKING) {
                Skid_EndEvent(area, u8SubIdx, t);
            }
        }
        return;
    }

    /* 贪心最近邻匹配：反复找全局最近且小于阈值的（簇,跟踪器）对 */
    for (;;) {
        float    fBestDist = SKID_MATCH_TH;
        uint8_t  u8BestTrk = SKID_TRACKER_PER_AREA;
        uint8_t  u8BestClu = u8CluCnt;

        for (u8SubIdx = 0U; u8SubIdx < SKID_TRACKER_PER_AREA; u8SubIdx++) {
            skid_tracker_t *t = &s_astTracker[area][u8SubIdx];
            if ((t->u8State != SKID_STATE_TRACKING) || u8TrkMatch[u8SubIdx]) {
                continue;
            }
            for (u8CluIdx = 0U; u8CluIdx < u8CluCnt; u8CluIdx++) {
                float fDx, fDy, fDist;
                if (u8CluMatch[u8CluIdx]) {
                    continue;
                }
                fDx = stcClu[u8CluIdx].fC - t->fPrevC;
                fDy = stcClu[u8CluIdx].fR - t->fPrevR;
                fDist = sqrtf(fDx * fDx + fDy * fDy);
                if (fDist < fBestDist) {
                    fBestDist = fDist;
                    u8BestTrk = u8SubIdx;
                    u8BestClu = u8CluIdx;
                }
            }
        }
        if ((u8BestTrk >= SKID_TRACKER_PER_AREA) || (u8BestClu >= u8CluCnt)) {
            break;   /* 无剩余可匹配对 */
        }
        u8TrkMatch[u8BestTrk] = 1U;
        u8CluMatch[u8BestClu] = 1U;
        /* 匹配到的簇沿用原状态机更新 */
        Skid_UpdateTracker(mask, area, u8BestTrk, &s_astTracker[area][u8BestTrk],
                           stcClu[u8BestClu].fR, stcClu[u8BestClu].fC);
    }

    /* 未匹配的簇：占用空闲跟踪器启动新事件 */
    for (u8CluIdx = 0U; u8CluIdx < u8CluCnt; u8CluIdx++) {
        skid_tracker_t *t;
        uint8_t u8Free = SKID_TRACKER_PER_AREA;

        if (u8CluMatch[u8CluIdx]) {
            continue;
        }
        for (u8SubIdx = 0U; u8SubIdx < SKID_TRACKER_PER_AREA; u8SubIdx++) {
            if (s_astTracker[area][u8SubIdx].u8State == SKID_STATE_IDLE) {
                u8Free = u8SubIdx;
                break;
            }
        }
        if (u8Free >= SKID_TRACKER_PER_AREA) {
            continue;   /* 无空闲跟踪器，忽略该簇 */
        }
        t = &s_astTracker[area][u8Free];
        t->u8State    = SKID_STATE_TRACKING;
        t->u8Slid     = 0U;
        t->u8StallCnt = 0U;
        t->u8FrameCnt = 0U;
        t->fStartR    = stcClu[u8CluIdx].fR;
        t->fStartC    = stcClu[u8CluIdx].fC;
        t->fPrevR     = stcClu[u8CluIdx].fR;
        t->fPrevC     = stcClu[u8CluIdx].fC;
        t->u16LenX100 = 0U;
        t->u16AngleX100 = 0U;
        t->u16CurNo = 0U;
        {
            uint16_t u16No = Skid_FindStartNo(mask, area, stcClu[u8CluIdx].fR, stcClu[u8CluIdx].fC);
            /* 节内局部编号 + 节首全局偏移 → 协议附录5 全局编号（找不到时保持 0） */
            t->u16StartNo = (u16No == 0U) ? 0U : (uint16_t)(u16No + s_au16AreaGlobalOffset[area]);
        }
        /* 新启动的跟踪器本帧已分配簇：置位本帧匹配标记，防止下方"簇已消失"清理分支
           在同一帧内立即回收，否则事件永远无法跨帧延续（状态恒为 IDLE） */
        u8TrkMatch[u8Free] = 1U;
    }

    /* 活跃但未被匹配的跟踪器：簇已消失（仅回收本帧匹配段存活、却未关联到任何簇
     * 的上一帧跟踪器；新启动的已通过 u8TrkMatch 排除）→ 结束事件 */
    for (u8SubIdx = 0U; u8SubIdx < SKID_TRACKER_PER_AREA; u8SubIdx++) {
        skid_tracker_t *t = &s_astTracker[area][u8SubIdx];
        if ((t->u8State == SKID_STATE_TRACKING) && (!u8TrkMatch[u8SubIdx])) {
            Skid_EndEvent(area, u8SubIdx, t);
        }
    }
}

/*******************************************************************************
 * 对外接口实现
 ******************************************************************************/
/**
 * @brief  滑移检测每帧更新
 * @param  p  本轮成品帧（采集快照/工作区，只读）
 * @note   遍历 TAC_AREA_MAX 个区域逐个执行 Skid_UpdateArea；只读输入、内部不自加锁；
 *         调用上下文：采样任务整轮写窗口内（持有 m_hSampleDataMutex），单实例禁重入
 */
void Skid_Update(const tac_data_cfg_t *p)
{
    const tac_data_cfg_t *mask;
    tac_area_t area;

    if (p == NULL) {
        return;
    }
    mask = GetTacMaskData();
    for (area = (tac_area_t)0U; area < TAC_AREA_MAX; area = (tac_area_t)(area + 1U)) {
        Skid_UpdateArea(p, mask, area);
    }
}

/**
 * @brief  快照拷贝核心（不带临界保护）
 * @param  sub_id   手指级编号 0~4（0=拇指 … 4=小指，与 SKID 快照数组下标一致）
 * @param  out      输出缓冲（容量 >= max_cnt）
 * @param  max_cnt  最多拷贝条数
 * @retval 实际拷贝条数
 * @note   调用方须自行包临界区：任务版用 taskENTER_CRITICAL，ISR 版用
 *         taskENTER_CRITICAL_FROM_ISR（见 Skid_GetSnapshot / Skid_GetSnapshotIsr）
 */
static uint8_t Skid_CopySnap(uint8_t sub_id, skid_snapshot_t *out, uint8_t max_cnt)
{
    uint8_t u8Finger, u8Cnt, u8Idx;

    if ((sub_id < 0x01U) || (sub_id > (uint8_t)TAC_FINGER_MAX) || (out == NULL) || (max_cnt == 0U)) {
        return 0U;
    }
    u8Finger = sub_id - 1U;
    u8Cnt = s_au8SlotCnt[u8Finger];
    if (u8Cnt > max_cnt) {
        u8Cnt = max_cnt;
    }
    for (u8Idx = 0U; u8Idx < u8Cnt; u8Idx++) {
        out[u8Idx] = s_astSnap[u8Finger][u8Idx];
    }
    return u8Cnt;
}

/**
 * @brief  读取某手指全部事件槽快照（任务上下文版，供采样/控制等任务按需读取）
 * @param  sub_id   手指级编号 0~4（0=拇指 … 4=小指）
 * @param  out      输出缓冲（容量 >= max_cnt）
 * @param  max_cnt  最多拷贝条数
 * @retval 实际拷贝条数
 * @note   官方协议 RTO 中断应答请使用 Skid_GetSnapshotIsr
 */
uint8_t Skid_GetSnapshot(uint8_t sub_id, skid_snapshot_t *out, uint8_t max_cnt)
{
    uint8_t u8Cnt;

    /* 快照由采样任务写入，短临界区拷贝避免撕裂读 */
    taskENTER_CRITICAL();
    u8Cnt = Skid_CopySnap(sub_id, out, max_cnt);
    taskEXIT_CRITICAL();
    return u8Cnt;
}

/**
 * @brief  读取某手指全部事件槽快照（ISR 上下文版，官方协议 RTO 中断应答用）
 * @param  sub_id   手指级编号 0~4（0=拇指 … 4=小指）
 * @param  out      输出缓冲（容量 >= max_cnt）
 * @param  max_cnt  最多拷贝条数
 * @retval 实际拷贝条数
 * @note   与 Skid_GetSnapshot 功能一致，临界保护采用 ISR 安全版宏
 *         (taskENTER_CRITICAL_FROM_ISR)，禁止在中断上下文误用任务版接口。
 */
uint8_t Skid_GetSnapshotIsr(uint8_t sub_id, skid_snapshot_t *out, uint8_t max_cnt)
{
    uint8_t u8Cnt;
    UBaseType_t uxSaved;

    uxSaved = taskENTER_CRITICAL_FROM_ISR();
    u8Cnt = Skid_CopySnap(sub_id, out, max_cnt);
    taskEXIT_CRITICAL_FROM_ISR(uxSaved);
    return u8Cnt;
}
