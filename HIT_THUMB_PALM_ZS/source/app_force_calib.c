/*******************************************************************************
 * @file    app_force_calib.c
 * @brief   力值标定核心 + HcFlash 持久化 + 部位/区域映射
 * @date    2026-09-16
 *
 * 组成：
 *  1) 标定核心：回调注入 port{load/save/region_id}，单实例禁重入；容量
 *     MAX_POINTS=62（对齐协议 2~62 点）；记录魔数 'HCL2'；表值 uint16、
 *     output_max=255；
 *  2) HcFlash 持久化：8KB 标定区（0x7E000~0x7FFFF）按每部位 1KB 槽位存 6 个
 *     紧凑 blob（uint16 AD/力值，764B/槽）；任一部位保存 = 整扇区擦除 +
 *     回写 6 槽（RAM 常驻镜像）+ 逐槽校验；逐字节落盘格式见第 2 节注释块；
 *  3) 部位映射：part_id 1~6（拇指/食指/中指/无名指/小指/掌心）↔
 *     tac_data_cfg_t 各子矩阵；上传矩形 = 子区域纵向拼接（小指 DIP 10 列，
 *     第 10 列补零）；**标定区域按物理节段划分**（见 fc_port_region_id）：
 *     拇指 DIP/MCP=区域 1/2、四指 DIP/PIP/MCP=区域 1/2/3、掌心=区域 1；
 *     点位有效性由掩码矩阵（GetTacMaskData）判定。
 *
 * 完整性门控（上电判定一次，见 fc_calib_is_complete / s_bCalibComplete）：
 *  6 个部位的区域数全部达到各自物理节段数且每区域有效才判"完整"，才允许
 *  AD->克重换算；不完整时 6 个部位统一保持减零点后的 AD 增量输出。
 *
 * 换算（完整且开关开启时，采样整轮写窗口内调用）：
 *  区域 AD 总和 → LUT 三段线性（低端自 (0,0) 过渡、高端末段外推、中间插值；单位克重 g）→ 截断除以
 *  APP_FORCE_CALIB_OUT_UNIT_DIV（默认 2，故对外输出以 2g 为单位）→ 各点按 AD 占比
 *  分摊 → 单点限幅 0~255（不做保底、不回补差额，故区域总和可小于插值力值）。
 *  注意：标定表本身（PUT 下发 / RAM / Flash blob）仍存克重 g，单位换算只作用于输出。
 ******************************************************************************/
#include <string.h>
#include "app_force_calib.h"
#include "app_tac_config.h"
#include "hc_flash.h"        /* 标定区持久化；LL_OK 由 hc_flash.h -> hc32_ll_efm.h 引入 */
#include "hc32_ll_utility.h" /* DDL_Printf（Flash 回读测试打印，LL_PRINT_ENABLE=DDL_OFF 时为空函数） */

/* ============================ 前向声明 ============================ */
static uint8_t fc_port_region_id(void *context, uint8_t part, uint16_t x, uint16_t y);
static uint8_t *fc_subarea_data_ptr(tac_data_cfg_t *cfg, uint8_t part, uint8_t idx);

/* ============================ 常量（前向使用） ============================ */
/* 部位子区域在部位矩形矩阵中的布局：rows×cols，行偏移 row_off。
 * 上传矩形 = 部位全部子区域纵向拼接（小指 DIP 仅 10 列，第 10 列为补零列）。
 *
 * 以下尺寸全部由 app_tac_config.h 的行列分段宏推导：tac_data_cfg_t 各子矩阵
 * 成员即按同一批宏声明，故布局表与采集数组天然一致，请勿改回字面量。 */
#define FC_MAX(a, b)      (((a) > (b)) ? (a) : (b))
#define FC_SUB_ROWS(name) ((uint8_t)(TAC_ROW_##name##_END - TAC_ROW_##name##_START + 1U))
#define FC_SUB_COLS(name) ((uint8_t)(TAC_COL_##name##_END - TAC_COL_##name##_START + 1U))

/* 段内纵向拼接偏移：DIP 从 0 行起，PIP 紧接 DIP，MCP 紧接 PIP
 * （拇指无 PIP，其 MCP 直接接在 THUMB_DIP 段高之后） */
#define FC_ROW_OFF_DIP       (0U)
#define FC_ROW_OFF_PIP(part) (FC_SUB_ROWS(part##_DIP))
#define FC_ROW_OFF_MCP(part) (FC_SUB_ROWS(part##_DIP) + FC_SUB_ROWS(part##_PIP))

/* 部位矩形尺寸（0x02 上传帧 rows/cols）：高 = 各段段高之和，宽 = 段内最大列宽。
 * 小指矩形列宽取 PIP/MCP 的 11 列（DIP 仅 10 列，第 10 列为补零列） */
#define FC_THUMB_RECT_ROWS    (FC_SUB_ROWS(THUMB_DIP) + FC_SUB_ROWS(THUMB_MCP))
#define FC_FINGER_RECT_ROWS(part) \
    (FC_SUB_ROWS(part##_DIP) + FC_SUB_ROWS(part##_PIP) + FC_SUB_ROWS(part##_MCP))
#define FC_THUMB_RECT_COLS    (FC_MAX(FC_SUB_COLS(THUMB_DIP), FC_SUB_COLS(THUMB_MCP)))
#define FC_FINGER_RECT_COLS(part) \
    (FC_MAX(FC_MAX(FC_SUB_COLS(part##_DIP), FC_SUB_COLS(part##_PIP)), FC_SUB_COLS(part##_MCP)))

/* 部位矩形最大尺寸（换算工作缓存容量）：四指矩形最高 17、中指最宽 13，
 * 掌心 15×16 取列宽最大 16；拇指 14×14 均不越界 */
#define FC_PART_RECT_ROWS_MAX   (FC_MAX(FC_FINGER_RECT_ROWS(INDEX), FC_SUB_ROWS(PALM)))
#define FC_PART_RECT_COLS_MAX   (FC_MAX(FC_FINGER_RECT_COLS(MIDDLE), FC_SUB_COLS(PALM)))
#define FC_PART_RECT_POINTS_MAX ((uint16_t)(FC_PART_RECT_ROWS_MAX * FC_PART_RECT_COLS_MAX))

/* 编译期断言：条件为假时 sizeof(char[-1]) 编译失败，零运行开销 */
#define FC_CONST_CHECK(cond)  do { (void)sizeof(char[(cond) ? 1 : -1]); } while (0U)

typedef struct
{
    uint8_t rows;
    uint8_t cols;
    uint8_t row_off;
} fc_subarea_t;

static const fc_subarea_t s_astPartLayout[APP_FORCE_CALIB_PART_MAX][3] =
{
    /* part1 拇指 14×14  */
    {{FC_SUB_ROWS(THUMB_DIP),  FC_SUB_COLS(THUMB_DIP),  FC_ROW_OFF_DIP},
     {FC_SUB_ROWS(THUMB_MCP),  FC_SUB_COLS(THUMB_MCP),  FC_SUB_ROWS(THUMB_DIP)},
     {0U, 0U, 0U}},
    /* part2 食指 17×12  */
    {{FC_SUB_ROWS(INDEX_DIP),  FC_SUB_COLS(INDEX_DIP),  FC_ROW_OFF_DIP},
     {FC_SUB_ROWS(INDEX_PIP),  FC_SUB_COLS(INDEX_PIP),  FC_ROW_OFF_PIP(INDEX)},
     {FC_SUB_ROWS(INDEX_MCP),  FC_SUB_COLS(INDEX_MCP),  FC_ROW_OFF_MCP(INDEX)}},
    /* part3 中指 17×13  */
    {{FC_SUB_ROWS(MIDDLE_DIP), FC_SUB_COLS(MIDDLE_DIP), FC_ROW_OFF_DIP},
     {FC_SUB_ROWS(MIDDLE_PIP), FC_SUB_COLS(MIDDLE_PIP), FC_ROW_OFF_PIP(MIDDLE)},
     {FC_SUB_ROWS(MIDDLE_MCP), FC_SUB_COLS(MIDDLE_MCP), FC_ROW_OFF_MCP(MIDDLE)}},
    /* part4 无名指 17×12 */
    {{FC_SUB_ROWS(RING_DIP),   FC_SUB_COLS(RING_DIP),   FC_ROW_OFF_DIP},
     {FC_SUB_ROWS(RING_PIP),   FC_SUB_COLS(RING_PIP),   FC_ROW_OFF_PIP(RING)},
     {FC_SUB_ROWS(RING_MCP),   FC_SUB_COLS(RING_MCP),   FC_ROW_OFF_MCP(RING)}},
    /* part5 小指 17×11  */
    {{FC_SUB_ROWS(LITTLE_DIP), FC_SUB_COLS(LITTLE_DIP), FC_ROW_OFF_DIP},
     {FC_SUB_ROWS(LITTLE_PIP), FC_SUB_COLS(LITTLE_PIP), FC_ROW_OFF_PIP(LITTLE)},
     {FC_SUB_ROWS(LITTLE_MCP), FC_SUB_COLS(LITTLE_MCP), FC_ROW_OFF_MCP(LITTLE)}},
    /* part6 掌心 15×16  */
    {{FC_SUB_ROWS(PALM),       FC_SUB_COLS(PALM),       FC_ROW_OFF_DIP},
     {0U, 0U, 0U},
     {0U, 0U, 0U}},
};
/* 每部位物理节段数（子区域个数）= 该部位"标定区域数"的期望值：
 * 拇指 DIP/MCP=2、食指/中指/无名指/小指 DIP/PIP/MCP=3、掌心单矩阵=1；
 * 区域映射（fc_port_region_id）与完整性判定（fc_calib_is_complete）共用本表，
 * 是"期望区域数"的唯一权威来源；改动上方布局宏时须同步复核本表 */
static const uint8_t s_au8PartSubCnt[APP_FORCE_CALIB_PART_MAX] = {2U, 3U, 3U, 3U, 3U, 1U};
static const uint8_t s_au8PartRows[APP_FORCE_CALIB_PART_MAX]   =
{
    FC_THUMB_RECT_ROWS, FC_FINGER_RECT_ROWS(INDEX),  FC_FINGER_RECT_ROWS(MIDDLE),
    FC_FINGER_RECT_ROWS(RING), FC_FINGER_RECT_ROWS(LITTLE), FC_SUB_ROWS(PALM)
};
static const uint8_t s_au8PartCols[APP_FORCE_CALIB_PART_MAX]   =
{
    FC_THUMB_RECT_COLS, FC_FINGER_RECT_COLS(INDEX),  FC_FINGER_RECT_COLS(MIDDLE),
    FC_FINGER_RECT_COLS(RING), FC_FINGER_RECT_COLS(LITTLE), FC_SUB_COLS(PALM)
};

/*******************************************************************************
 * 1. 标定核心（单实例；持久化与区域映射经 port 回调注入）
 ******************************************************************************/
#define FORCE_CALIBRATION_MAX_PARTS   APP_FORCE_CALIB_PART_MAX
#define FORCE_CALIBRATION_MAX_REGIONS APP_FORCE_CALIB_REGION_MAX
/* LUT 点数容量 = 协议单区域 2~62 点（**不是**部位有效传感点数上界 141）：
 * 该宏同时决定运行表与 Flash blob 的 uint16 数组容量，误用 141 会让
 * sizeof(fc_flash_blob_t) 涨到 1712B > 1KB 槽位，6 部位槽位首尾重叠 */
#define FORCE_CALIBRATION_MAX_POINTS  APP_FORCE_CALIB_LUT_POINTS_MAX
#define FORCE_CALIBRATION_RECORD_MAGIC 0x48434C32UL  /* 'HCL2'：RAM 运行格式标识 */

/* 单区域 LUT（RAM 运行格式，uint16；与 Flash blob 的 fc_flash_region_t、PUT 下发产物
 * app_force_calib_table_t 同型，故 table/record/blob 之间互转均为直接赋值）；
 * reserved/crc/version 为布局占位，既不参与通信字段也不落盘到 Flash blob */
typedef struct
{
    uint8_t  count;
    uint8_t  reserved[3];
    uint32_t crc;
    uint32_t version;
    uint16_t adc[FORCE_CALIBRATION_MAX_POINTS];     /* 减零点后分区合值，严格递增 */
    uint16_t force[FORCE_CALIBRATION_MAX_POINTS];   /* 标定克重值(g)，单调不降；与对外输出单位无关 */
} force_calibration_region_t;

typedef struct
{
    uint32_t                   magic;
    uint8_t                    region_count;
    uint8_t                    reserved[3];
    force_calibration_region_t region[FORCE_CALIBRATION_MAX_REGIONS];
} force_calibration_record_t;

typedef struct
{
    bool (*load)(void *context, uint8_t part, force_calibration_record_t *record);
    bool (*save)(void *context, uint8_t part, const force_calibration_record_t *record);
    /* 返回 0=无效点位、1 起为区域编号（按物理节段：拇指 2 区、四指 3 区、掌心 1 区） */
    uint8_t (*region_id)(void *context, uint8_t part, uint16_t x, uint16_t y);
    void *context;
} force_calibration_port_t;

typedef struct
{
    force_calibration_port_t   port;
    force_calibration_record_t records[FORCE_CALIBRATION_MAX_PARTS];
    force_calibration_record_t working;
    uint8_t                    part_count;
    uint16_t                   output_max;       /* 单点输出上限，255 */
} force_calibration_t;

#if (APP_FORCE_CALIB_SUM_SMOOTH_EN != 0U)
/* 区域 AD 总和的一阶 IIR 平滑状态（[部位][区域]，仅换算路径使用）：
 * 抑制"静止零漂 + LUT 低端陡斜率"放大出的读数跳变——被平滑的是**决定查表力值的区域
 * 总和**，逐点仍按瞬时占比分摊，故手型/受压分布不滞后。
 * 约定：槽值为 0 视为"无历史"（无接触/刚复位），此时直接取瞬时值，避免从 0 缓慢爬升；
 * 松开（sum==0）时该槽清零，保证无残留。清零点见 AppForceCalib_ResetSmoothState()。 */
static uint32_t s_au32SumSmooth[FORCE_CALIBRATION_MAX_PARTS][FORCE_CALIBRATION_MAX_REGIONS];
#endif

/**
 * @brief  部位号合法性校验
 * @param  calibration  标定核心实例（可为 NULL）
 * @param  part         部位号 1~6
 * @retval true  实例非空且 part 落在 [1, part_count] 内
 */
static bool fc_valid_part(const force_calibration_t *calibration, uint8_t part)
{
    return ((NULL != calibration) && (part >= 1U) && (part <= calibration->part_count)) ? true : false;
}

/**
 * @brief  运行记录（RAM 格式）有效性校验
 * @param  record  待校验记录
 * @retval true  记录合法
 * @note   判据：magic=='HCL2'、region_count<=3、每区域 count 2~62、
 *         adc 严格递增、force 单调不降
 *         （表值本身即 uint16，不需再做数值域检查）
 */
static bool fc_valid_record(const force_calibration_record_t *record)
{
    uint8_t  r;
    uint16_t i;

    if ((record->magic != FORCE_CALIBRATION_RECORD_MAGIC) ||
        (record->region_count > FORCE_CALIBRATION_MAX_REGIONS))
    {
        return false;
    }

    for (r = 0U; r < record->region_count; r++)
    {
        const force_calibration_region_t *region = &record->region[r];

        if ((region->count < 2U) || (region->count > FORCE_CALIBRATION_MAX_POINTS))
        {
            return false;
        }

        for (i = 0U; i < region->count; i++)
        {
            if (((i > 0U) && (region->adc[i] <= region->adc[i - 1U])) ||
                ((i > 0U) && (region->force[i] < region->force[i - 1U])))
            {
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief  初始化标定核心，并经 port->load 加载部位 1~part_count
 * @param  calibration  输出：标定核心实例
 * @param  port         持久化/区域映射端口（load 为 NULL 表示无持久化，全部按未标定）
 * @param  part_count   部位数 1~FORCE_CALIBRATION_MAX_PARTS
 * @retval true  初始化完成；false 参数非法（实例不修改）
 * @note   加载失败或记录无效的部位按未标定处理（该记录清零）；
 *         output_max 固定 255（单点输出上限）；
 *         可在调度器启动前调用（load 只读 Flash）
 */
static bool force_calibration_init(force_calibration_t *calibration, const force_calibration_port_t *port,
                                   uint8_t part_count)
{
    uint8_t part;

    if ((NULL == calibration) || (NULL == port) || (0U == part_count) ||
        (part_count > FORCE_CALIBRATION_MAX_PARTS))
    {
        return false;
    }

    memset(calibration, 0, sizeof(force_calibration_t));
    calibration->port           = *port;
    calibration->part_count     = part_count;
    calibration->output_max     = 255U;

    if (NULL != port->load)
    {
        for (part = 1U; part <= part_count; part++)
        {
            force_calibration_record_t *record = &calibration->records[part - 1U];

            if ((false == port->load(port->context, part, record)) ||
                (false == fc_valid_record(record)))
            {
                memset(record, 0, sizeof(force_calibration_record_t));
            }
        }
    }

    return true;
}

/**
 * @brief  查询部位标定状态
 * @param  calibration   标定核心实例
 * @param  part          部位 1~6
 * @param  region_count  输出：已存储区域数
 * @param  mask          输出：区域有效位掩码，bit(r-1)=区域 r 已标定（>=2 点）
 * @retval true  查询完成；false 实例/部位非法或输出指针为 NULL（输出不修改）
 */
static bool force_calibration_get_status(const force_calibration_t *calibration, uint8_t part,
                                         uint8_t *region_count, uint16_t *mask)
{
    const force_calibration_record_t *record;
    uint8_t r;

    if ((false == fc_valid_part(calibration, part)) || (NULL == region_count) || (NULL == mask))
    {
        return false;
    }

    record        = &calibration->records[part - 1U];
    *region_count = record->region_count;
    *mask         = 0U;

    for (r = 0U; r < record->region_count; r++)
    {
        if (record->region[r].count >= 2U)
        {
            *mask |= (uint16_t)(1UL << r);
        }
    }

    return true;
}

/**
 * @brief  整体替换指定部位全部区域 LUT：先落盘（失败保留旧表）再发布运行表
 * @param  calibration   标定核心实例
 * @param  part          部位 1~6
 * @param  tables        区域 LUT 数组（adc/force 指针 + count），长度 >= region_count
 * @param  region_count  区域数 1~3
 * @retval true  已保存并发布；false 参数/数值非法或持久化失败（运行表不变）
 * @note   校验项：每区域 count 2~62、adc 严格递增、force 不降；
 *         新表先写入暂存区 working，落盘成功后才整体替换运行记录
 */
static bool force_calibration_set_part(force_calibration_t *calibration, uint8_t part,
                                       const app_force_calib_table_t *tables, uint8_t region_count)
{
    force_calibration_record_t *next;
    uint8_t  r;
    uint16_t i;

    if ((false == fc_valid_part(calibration, part)) || (NULL == tables) ||
        (0U == region_count) || (region_count > FORCE_CALIBRATION_MAX_REGIONS))
    {
        return false;
    }

    next = &calibration->working;
    memset(next, 0, sizeof(force_calibration_record_t));
    next->magic        = FORCE_CALIBRATION_RECORD_MAGIC;
    next->region_count = region_count;

    for (r = 0U; r < region_count; r++)
    {
        if ((NULL == tables[r].adc) || (NULL == tables[r].force) ||
            (tables[r].count < 2U) || (tables[r].count > FORCE_CALIBRATION_MAX_POINTS))
        {
            return false;
        }

        next->region[r].count = (uint8_t)tables[r].count;

        for (i = 0U; i < tables[r].count; i++)
        {
            if ((i > 0U) &&
                ((tables[r].adc[i] <= tables[r].adc[i - 1U]) ||
                 (tables[r].force[i] < tables[r].force[i - 1U])))
            {
                return false;
            }

            next->region[r].adc[i]   = tables[r].adc[i];
            next->region[r].force[i] = tables[r].force[i];
        }
    }

    /* 先保存再发布，失败只影响暂存区，不替换当前运行表 */
    if ((NULL != calibration->port.save) &&
        (false == calibration->port.save(calibration->port.context, part, next)))
    {
        return false;
    }

    calibration->records[part - 1U] = *next;

    return true;
}

/**
 * @brief  清除指定部位标定：写空记录落盘并清空该部位运行表（不影响其他部位）
 * @param  calibration  标定核心实例
 * @param  part         部位 1~6
 * @retval true  已落盘；false 部位非法或持久化失败（运行表不变）
 * @note   传入的全零记录经 fc_record_to_blob 后仍会写入 magic='HCS2'、
 *         region_count=0、各区域 count=0，故落盘内容是"空槽记录"而非全 0/全 0xFF
 */
static bool force_calibration_clear_part(force_calibration_t *calibration, uint8_t part)
{
    force_calibration_record_t record;

    if (false == fc_valid_part(calibration, part))
    {
        return false;
    }

    memset(&record, 0, sizeof(force_calibration_record_t));

    if ((NULL != calibration->port.save) &&
        (false == calibration->port.save(calibration->port.context, part, &record)))
    {
        return false;
    }

    memset(&calibration->records[part - 1U], 0, sizeof(force_calibration_record_t));

    return true;
}

/**
 * @brief  原地换算 data[y*stride+x]（stride >= cols）：AD 增量 -> 力值
 * @param  calibration  标定核心实例
 * @param  part         部位 1~6
 * @param  data         行优先点位数组（uint16，就地改写为力值）
 * @param  rows         矩形行数
 * @param  cols         矩形列数
 * @param  stride       行跨距（>= cols）
 * @note   流程：每区域求 AD 总和（region_count==1 时自动纳入全部有效点）→ LUT 整数
 *         三段线性（低端自 (0,0) 过渡、高端末段外推、中间插值；单位克重 g）→ 截断除以
 *         APP_FORCE_CALIB_OUT_UNIT_DIV（默认 2 -> 对外输出 2g 单位）→ 按各点 AD
 *         占比分摊（scaled = 点值 * 插值力值 / 区域总和）→ 单点限幅 0~output_max(255)；
 *         不做保底也不回补差额，故区域输出总和可能小于插值力值，点位少时各点
 *         同时取到 255 上限（2g 单位下即 510g）；
 *         未标定部位直接返回、保持 AD 增量输出；
 *         调用上下文：采样任务整轮写临界段内（单实例，禁重入）
 */
static void force_calibration_apply(const force_calibration_t *calibration, uint8_t part,
                                    uint16_t *data, uint16_t rows, uint16_t cols, uint16_t stride)
{
    const force_calibration_record_t *record;
    uint8_t region;

    if ((NULL == calibration) || (part < 1U) || (part > calibration->part_count) ||
        (NULL == data) || (0U == rows) || (0U == cols) || (stride < cols) ||
        (NULL == calibration->port.region_id))
    {
        return;
    }

    record = &calibration->records[part - 1U];
    if (record->region_count == 0U)
    {
        return; /* 未标定：保持原始 AD 增量 */
    }

    for (region = 0U; region < record->region_count; region++)
    {
        const force_calibration_region_t *lut = &record->region[region];
        uint32_t sum    = 0U;
        uint32_t target = 0U;   /* 区域 AD 总和经 LUT 插值所得力值（uint16 域内） */
        uint16_t x, y;
        uint8_t  i;

        if (lut->count < 2U)
        {
            continue;
        }

        for (y = 0U; y < rows; y++)
        {
            for (x = 0U; x < cols; x++)
            {
                uint8_t point_region = calibration->port.region_id(calibration->port.context, part, x, y);

                if ((0U != point_region) &&
                    ((1U == record->region_count) || (point_region == (uint8_t)(region + 1U))))
                {
                    sum += data[((uint32_t)y * stride) + x];
                }
            }
        }

        if (0U == sum)
        {
#if (APP_FORCE_CALIB_SUM_SMOOTH_EN != 0U)
            s_au32SumSmooth[part - 1U][region] = 0U;   /* 无接触：清平滑状态，下次接触直接取瞬时值 */
#endif
            continue; /* 区域无触点 */
        }

#if (APP_FORCE_CALIB_SUM_SMOOTH_EN != 0U)
        /* 区域总和的一阶 IIR 平滑（alpha = 1/2^SHIFT）：只在有接触时更新；
           槽值为 0 表示无历史（首次/刚复位），直接取瞬时值避免爬升 */
        {
            uint32_t u32SumSmooth = s_au32SumSmooth[part - 1U][region];

            if (0U == u32SumSmooth)
            {
                u32SumSmooth = sum;
            }
            else if (sum >= u32SumSmooth)
            {
                u32SumSmooth += ((sum - u32SumSmooth) >> APP_FORCE_CALIB_SUM_SMOOTH_SHIFT);
            }
            else
            {
                u32SumSmooth -= ((u32SumSmooth - sum) >> APP_FORCE_CALIB_SUM_SMOOTH_SHIFT);
            }

            s_au32SumSmooth[part - 1U][region] = u32SumSmooth;
            sum = u32SumSmooth;   /* 后续查表与分摊使用平滑后的区域总和 */
        }
#endif

        /* LUT 三段线性（uint32 整数运算，无浮点）：
         *  - sum < adc[0]         ：自 (0,0) 到 (adc[0],force[0]) 线性过渡（消除硬阶跃）；
         *  - sum >= adc[count-1]  ：沿最后一段斜率线性外推；
         *  - 中间                 ：相邻两点线性插值。
         *  溢出边界：sum = 区域各点 uint8 增量之和（<= 点数*255 < 65536），force <= 65535、
         *  adc 严格递增，各乘积 <= 65535*65535 < 2^32，uint32 内安全；
         *  分母（adc[0]、a1-a0、adc[n-1]-adc[n-2]）因 count>=2 且严格递增恒 >=1，无除零 */
        if (sum < (uint32_t)lut->adc[0])
        {
            /* 低端线性过渡：(0,0) -> (adc[0], force[0]) */
            target = ((uint32_t)lut->force[0] * sum) / (uint32_t)lut->adc[0];
        }
        else if (sum >= (uint32_t)lut->adc[lut->count - 1U])
        {
            /* 高端线性外推：沿 (adc[n-2],force[n-2])->(adc[n-1],force[n-1]) 斜率延续 */
            uint32_t a0 = (uint32_t)lut->adc[lut->count - 2U];
            uint32_t a1 = (uint32_t)lut->adc[lut->count - 1U];
            uint32_t f0 = (uint32_t)lut->force[lut->count - 2U];
            uint32_t f1 = (uint32_t)lut->force[lut->count - 1U];

            target = f1 + ((sum - a1) * (f1 - f0)) / (a1 - a0);
        }
        else
        {
            /* 中间线性插值（原实现不变） */
            for (i = 0U; (uint8_t)(i + 1U) < lut->count; i++)
            {
                uint32_t a0 = (uint32_t)lut->adc[i];
                uint32_t a1 = (uint32_t)lut->adc[i + 1U];

                if ((sum >= a0) && (sum < a1))
                {
                    uint32_t f0 = (uint32_t)lut->force[i];
                    uint32_t f1 = (uint32_t)lut->force[i + 1U];

                    target = f0 + (((sum - a0) * (f1 - f0)) / (a1 - a0));
                    break;
                }
            }
        }

        /* 单位换算：标定表为克重 g，对外输出按 APP_FORCE_CALIB_OUT_UNIT_DIV 缩放
           （默认 2 -> 2g 单位：分辨率 2g、单点上限 255 即 510g）；截断取整，
           故区域总力不足 1 个单位时该区域输出全 0。置 1 即恢复克重输出 */
        target /= APP_FORCE_CALIB_OUT_UNIT_DIV;

        /* 按各点 AD 占比分摊插值力值，单点超 output_max 直接截断：
           点值取自 uint8 子矩阵（<=255）、target <= 65535，乘积 <= 16711425，uint32 内安全；
           又因 *v <= sum，比值必 <= target。不做保底、不回补差额：占比过小的点可输出 0，
           总力大而点位少时各点同时取到 255（区域输出总和小于 target 属预期语义） */
        for (y = 0U; y < rows; y++)
        {
            for (x = 0U; x < cols; x++)
            {
                uint8_t  point_region = calibration->port.region_id(calibration->port.context, part, x, y);
                uint16_t *v;
                uint32_t  scaled;

                if ((0U == point_region) ||
                    ((record->region_count > 1U) && (point_region != (uint8_t)(region + 1U))))
                {
                    continue;
                }

                v = &data[((uint32_t)y * stride) + x];
                if (0U == *v)
                {
                    continue;
                }

                scaled = ((uint32_t)(*v) * target) / sum;

                if (scaled > (uint32_t)calibration->output_max)
                {
                    scaled = (uint32_t)calibration->output_max;
                }

                *v = (uint16_t)scaled;
            }
        }
    }
}

/*******************************************************************************
 * 2. HcFlash 持久化（每部位 1KB 槽位，紧凑 uint16 blob）
 ******************************************************************************/
#define FC_FLASH_PART_SLOT   (1024UL)        /* 每部位槽位，6 槽共 6KB < 8KB 标定区 */
#define FC_FLASH_BLOB_MAGIC  (0x32534348UL)  /* 'HCS2'：紧凑存储格式标识 */

/* ============================================================================
 * 标定表在片内 Flash 的存储格式（落盘真实布局，勿凭注释猜测）
 * ============================================================================
 * 槽位排布（标定区 = 0x0007E000 起 8KB 单扇区，每槽 1024B，blob 仅占前 764B）
 *   0x7E000 +0x000 : part1 槽      0x7EC00 +0x000 : part4 槽
 *   0x7E400 +0x000 : part2 槽      0x7F000 +0x000 : part5 槽
 *   0x7E800 +0x000 : part3 槽      0x7F400 +0x000 : part6 槽
 *   0x7F800 .. 0x7FFFF : 空闲（不写，保持 0xFF）
 *
 * fc_flash_blob_t（764B，全部字段小端）
 *   偏移    长度  字段            说明
 *   0x000    4    magic           0x32534348；Flash 上字节序 48 43 53 32 = ASCII "HCS2"
 *   0x004    1    region_count    1~3；0 表示该部位已擦除/未标定
 *   0x005    3    reserved[3]     对齐填充，写 0
 *   0x008  252    region[0]       区域 1
 *   0x104  252    region[1]       区域 2
 *   0x200  252    region[2]       区域 3
 *   0x2FC .. 0x3FF                不写（保持 0xFF）
 *
 * fc_flash_region_t（252B = 4 + 62*2 + 62*2）
 *   0x00     1    count           本区域点数 2~62（未使用的区域写 0）
 *   0x01     3    reserved[3]     对齐填充，写 0
 *   0x04   124    adc[62]         AD 轴（区域内各点 AD 之和），前 count 个有效、严格递增
 *   0x80   124    force[62]       力值轴，前 count 个有效、单调不降
 *   注：只有前 count 个元素参与校验，count 之后的内容不影响有效性判定。
 *
 * 读写顺序（与下方回调一一对应）
 *   写：fc_record_to_blob -> HcFlash_CalibErase(整扇区) -> HcFlash_CalibWrite(6 槽)
 *       -> HcFlash_CalibVerify(逐槽回读比对)
 *   读：HcFlash_CalibRead(单槽) -> fc_blob_to_record -> 有效性校验
 *   擦：写 magic='HCS2' 且 region_count=0 且各区域 count=0 的 blob（仍重写全扇区）
 *
 * 其他约定
 *   - blob 内没有 CRC/校验和，落盘校验依赖写后回读比对（HcFlash_CalibVerify）；
 *   - 表内 adc/force 的单位与落盘格式**不随对外输出单位变化**：force 恒为克重 g
 *     （PUT 下发也是 g），APP_FORCE_CALIB_OUT_UNIT_DIV 只作用于输出换算结果；
 *   - 运行期 RAM 格式为 force_calibration_record_t（magic 'HCL2'，adc/force 为 uint16
 *     数组，与 blob 同型），与 blob 之间由本节的 record/blob 互转函数完成；
 *   - 容量关系：764B <= 1KB 槽位、6 槽共 4584B <= 8KB 标定区，由
 *     fc_layout_self_check() 的编译期断言守住（改任一容量宏都会在编译期报错）。
 * ========================================================================== */
typedef struct
{
    uint8_t  count;
    uint8_t  reserved[3];
    uint16_t adc[FORCE_CALIBRATION_MAX_POINTS];
    uint16_t force[FORCE_CALIBRATION_MAX_POINTS];
} fc_flash_region_t;

typedef struct
{
    uint32_t          magic;
    uint8_t           region_count;
    uint8_t           reserved[3];
    fc_flash_region_t region[FORCE_CALIBRATION_MAX_REGIONS];
} fc_flash_blob_t;

static force_calibration_t s_stCalib;                                   /* 标定核心单实例 */
static fc_flash_blob_t     s_astFlashBlob[FORCE_CALIBRATION_MAX_PARTS]; /* Flash 常驻镜像 */
/* 回读测试缓冲：直接读 Flash 落盘内容（764B），静态分配避免放在任务栈上
 * （usart 任务栈仅 1024 Word，放不下该结构体） */
static fc_flash_blob_t     s_stReadbackBlob;

/* 标定表完整性判定结果：上电 AppForceCalib_Init() 内判定一次，一个上电周期内固定
 * （PUT 下发/擦除后不重判，改标定表需复位才生效）。
 * true  = 6 个部位的区域数全部达到各自物理节段数且每区域有效 -> 允许 AD->克重换算；
 * false = 不完整 -> 6 个部位都保持"减零点后的 AD 增量"输出（见 AppForceCalib_ApplyPart）。 */
static bool                s_bCalibComplete = false;

static void fc_record_to_blob(const force_calibration_record_t *record, fc_flash_blob_t *blob)
{
    uint8_t i, j;

    blob->magic        = FC_FLASH_BLOB_MAGIC;
    blob->region_count = record->region_count;

    for (i = 0U; i < FORCE_CALIBRATION_MAX_REGIONS; i++)
    {
        fc_flash_region_t *dst = &blob->region[i];

        dst->count = 0U;

        if (i < record->region_count)
        {
            dst->count = record->region[i].count;

            for (j = 0U; j < record->region[i].count; j++)
            {
                dst->adc[j]   = record->region[i].adc[j];
                dst->force[j] = record->region[i].force[j];
            }
        }
    }
}

static void fc_blob_to_record(const fc_flash_blob_t *blob, force_calibration_record_t *record)
{
    uint8_t i, j;

    memset(record, 0, sizeof(force_calibration_record_t));

    if (blob->magic != FC_FLASH_BLOB_MAGIC)
    {
        return; /* 空片（全 0xFF）或未知数据：按未标定处理 */
    }

    record->region_count = blob->region_count;

    if (record->region_count > FORCE_CALIBRATION_MAX_REGIONS)
    {
        record->region_count = 0U;
        return;
    }

    record->magic = FORCE_CALIBRATION_RECORD_MAGIC;

    for (i = 0U; i < record->region_count; i++)
    {
        /* 点数越界（旧固件写入或脏数据）：整记录作废，避免按越界 count 拷贝越界写 */
        if ((blob->region[i].count < 2U) || (blob->region[i].count > FORCE_CALIBRATION_MAX_POINTS))
        {
            memset(record, 0, sizeof(force_calibration_record_t));
            return;
        }

        record->region[i].count = blob->region[i].count;

        for (j = 0U; j < blob->region[i].count; j++)
        {
            record->region[i].adc[j]   = blob->region[i].adc[j];
            record->region[i].force[j] = blob->region[i].force[j];
        }
    }
}

/* 端口回调：加载（上电时只读 Flash，可在调度器启动前调用） */
static bool fc_port_load(void *context, uint8_t part, force_calibration_record_t *record)
{
    fc_flash_blob_t *blob = &s_astFlashBlob[part - 1U];

    (void)context;

    if (LL_OK != HcFlash_CalibRead((uint32_t)(part - 1U) * FC_FLASH_PART_SLOT,
                                   (uint8_t *)blob, (uint32_t)sizeof(fc_flash_blob_t)))
    {
        return false;
    }

    fc_blob_to_record(blob, record);

    return true;
}

/* 端口回调：保存（更新 RAM 镜像 -> 整扇区擦除 -> 回写 6 槽 -> 逐槽校验） */
static bool fc_port_save(void *context, uint8_t part, const force_calibration_record_t *record)
{
    uint8_t i;

    (void)context;

    if ((part < 1U) || (part > FORCE_CALIBRATION_MAX_PARTS))
    {
        return false;
    }

    fc_record_to_blob(record, &s_astFlashBlob[part - 1U]);

    if (LL_OK != HcFlash_CalibErase())
    {
        return false;
    }

    for (i = 0U; i < FORCE_CALIBRATION_MAX_PARTS; i++)
    {
        uint32_t addr = (uint32_t)i * FC_FLASH_PART_SLOT;

        if (LL_OK != HcFlash_CalibWrite(addr, (const uint8_t *)&s_astFlashBlob[i],
                                        (uint32_t)sizeof(fc_flash_blob_t)))
        {
            return false;
        }

        if (LL_OK != HcFlash_CalibVerify(addr, (const uint8_t *)&s_astFlashBlob[i],
                                         (uint32_t)sizeof(fc_flash_blob_t)))
        {
            return false;
        }
    }

    return true;
}

/*******************************************************************************
 * 3. 部位子矩阵/掩码映射 + 对外接口
 ******************************************************************************/
/**
 * @brief  取采集/掩码工作区中部位子区域的首地址
 * @param  cfg   采集工作区或掩码矩阵（两者布局同构，共用本映射）
 * @param  part  部位 1~6
 * @param  idx   子区域序号（0 起；拇指 0=DIP/1=MCP；掌心只用 idx 0）
 * @retval 子矩阵首地址；部位/序号非法返回 NULL
 */
static uint8_t *fc_subarea_data_ptr(tac_data_cfg_t *cfg, uint8_t part, uint8_t idx)
{
    uint8_t *ptr = NULL;

    switch (part)
    {
        case 1U:    /* 拇指 */
            ptr = (0U == idx) ? (uint8_t *)cfg->thumb_dip : (uint8_t *)cfg->thumb_mcp;
            break;
        case 2U:    /* 食指 */
            ptr = (0U == idx) ? (uint8_t *)cfg->index_dip :
                  (1U == idx) ? (uint8_t *)cfg->index_pip : (uint8_t *)cfg->index_mcp;
            break;
        case 3U:    /* 中指 */
            ptr = (0U == idx) ? (uint8_t *)cfg->middle_dip :
                  (1U == idx) ? (uint8_t *)cfg->middle_pip : (uint8_t *)cfg->middle_mcp;
            break;
        case 4U:    /* 无名指 */
            ptr = (0U == idx) ? (uint8_t *)cfg->ring_dip :
                  (1U == idx) ? (uint8_t *)cfg->ring_pip : (uint8_t *)cfg->ring_mcp;
            break;
        case 5U:    /* 小指 */
            ptr = (0U == idx) ? (uint8_t *)cfg->little_dip :
                  (1U == idx) ? (uint8_t *)cfg->little_pip : (uint8_t *)cfg->little_mcp;
            break;
        case 6U:    /* 掌心 */
            ptr = (uint8_t *)cfg->palm;
            break;
        default:
            break;
    }

    return ptr;
}

/**
 * @brief  端口回调：点位所属标定区域（0=无效点位）
 * @param  context  未使用
 * @param  part     部位 1~6
 * @param  x        部位矩形内的列坐标（0 起）
 * @param  y        部位矩形内的行坐标（0 起）
 * @retval 0=掩码为 0 的无效点位；否则为本点位所属**物理节段**的区域号：
 *         拇指 DIP→1、MCP→2；食指/中指/无名指/小指 DIP→1、PIP→2、MCP→3；掌心→1
 * @note   掩码矩阵为 const 只读表，读侧免锁，可在定时器回调上下文调用；
 *         区域号与 s_au8PartSubCnt / s_astPartLayout 的节段序号严格对应（区域 r 的
 *         LUT 只累加本节段点位），也是完整性判定"期望区域数"的来源
 */
static uint8_t fc_port_region_id(void *context, uint8_t part, uint16_t x, uint16_t y)
{
    const tac_data_cfg_t *mask = GetTacMaskData();
    uint8_t sub, cnt;
    uint8_t ret = 0U;

    (void)context;

    if ((part < 1U) || (part > APP_FORCE_CALIB_PART_MAX) || (NULL == mask))
    {
        return 0U;
    }

    cnt = s_au8PartSubCnt[part - 1U];

    for (sub = 0U; sub < cnt; sub++)
    {
        const fc_subarea_t *sa = &s_astPartLayout[part - 1U][sub];

        if ((y >= (uint16_t)sa->row_off) && (y < (uint16_t)(sa->row_off + sa->rows)) &&
            (x < (uint16_t)sa->cols))
        {
            const uint8_t *m = fc_subarea_data_ptr((tac_data_cfg_t *)mask, part, sub);

            if (m[(((uint16_t)y - (uint16_t)sa->row_off) * (uint16_t)sa->cols) + x] != 0U)
            {
                /* 区域号 = 节段序号 + 1：拇指 DIP/MCP=1/2、四指 DIP/PIP/MCP=1/2/3、
                   掌心单矩阵=1（掌心在布局表中只有 sub=0，天然落在区域 1） */
                ret = (uint8_t)(sub + 1U);
            }

            break;
        }
    }

    return ret;
}

/* 换算工作缓存：最大部位矩形（四指 17×13 / 掌心 15×16 → 17×16=272 点），静态分配 */
static uint16_t s_au16PartBuf[FC_PART_RECT_POINTS_MAX];

/**
 * @brief  布局/容量编译期自检（函数体全为 FC_CONST_CHECK，运行期零开销）
 * @note   校验项：子矩阵段高段宽、部位矩形 rows/cols、换算缓存容量
 *         （FC_PART_RECT_POINTS_MAX 与 APP_FORCE_CALIB_RECT_POINTS_MAX 一致）、
 *         LUT 点数上限 62、blob <= 1KB 槽位、6 槽 <= 8KB 标定区；
 *         任一容量宏漂移都会在编译期报错
 */
static void fc_layout_self_check(void)
{
    /* 子矩阵段高/段宽 */
    FC_CONST_CHECK(FC_SUB_ROWS(THUMB_DIP)  == 11U);
    FC_CONST_CHECK(FC_SUB_COLS(THUMB_DIP)  == 14U);
    FC_CONST_CHECK(FC_SUB_ROWS(THUMB_MCP)  == 3U);
    FC_CONST_CHECK(FC_SUB_COLS(THUMB_MCP)  == 14U);
    FC_CONST_CHECK(FC_SUB_COLS(MIDDLE_DIP) == 13U);
    FC_CONST_CHECK(FC_SUB_ROWS(LITTLE_DIP) == 11U);
    FC_CONST_CHECK(FC_SUB_COLS(LITTLE_DIP) == 10U);   /* 小指 DIP 仅 10 列，第 10 列补零 */
    FC_CONST_CHECK(FC_SUB_COLS(LITTLE_PIP) == 11U);
    FC_CONST_CHECK(FC_SUB_COLS(LITTLE_MCP) == 11U);
    FC_CONST_CHECK(FC_SUB_ROWS(PALM)       == 15U);
    FC_CONST_CHECK(FC_SUB_COLS(PALM)       == 16U);

    /* 部位矩形 rows/cols（0x02 上传帧字段）：14/17/17/17/17/15、14/12/13/12/11/16 */
    FC_CONST_CHECK(FC_THUMB_RECT_ROWS       == 14U);
    FC_CONST_CHECK(FC_FINGER_RECT_ROWS(INDEX)  == 17U);
    FC_CONST_CHECK(FC_FINGER_RECT_ROWS(MIDDLE) == 17U);
    FC_CONST_CHECK(FC_FINGER_RECT_ROWS(RING)   == 17U);
    FC_CONST_CHECK(FC_FINGER_RECT_ROWS(LITTLE) == 17U);
    FC_CONST_CHECK(FC_SUB_ROWS(PALM)           == 15U);
    FC_CONST_CHECK(FC_THUMB_RECT_COLS       == 14U);
    FC_CONST_CHECK(FC_FINGER_RECT_COLS(INDEX)  == 12U);
    FC_CONST_CHECK(FC_FINGER_RECT_COLS(MIDDLE) == 13U);
    FC_CONST_CHECK(FC_FINGER_RECT_COLS(RING)   == 12U);
    FC_CONST_CHECK(FC_FINGER_RECT_COLS(LITTLE) == 11U);
    FC_CONST_CHECK(FC_SUB_COLS(PALM)           == 16U);

    /* 换算工作缓存容量（同时作为 0x02 上传帧 data 区容量上界，见
     * APP_FORCE_CALIB_RECT_POINTS_MAX；两处必须一致，防止矩形尺寸宏漂移） */
    FC_CONST_CHECK(FC_PART_RECT_POINTS_MAX == 272U);
    FC_CONST_CHECK(FC_PART_RECT_POINTS_MAX == APP_FORCE_CALIB_RECT_POINTS_MAX);

    /* 上传有效点数上界不得超过部位矩形点数（掩码有效点恒为矩形子集） */
    FC_CONST_CHECK(APP_FORCE_CALIB_POINTS_MAX <= FC_PART_RECT_POINTS_MAX);

    /* LUT/Flash 容量自检：
     *  - LUT 点数上限必须等于协议 4.2 节的 62，且不超过部位有效点数上界；
     *  - Flash blob 必须放得进单部位槽位（否则槽位首尾重叠，回读比对必失败）；
     *  - 全部槽位必须放得进标定区（否则越界写坏相邻扇区） */
    FC_CONST_CHECK(APP_FORCE_CALIB_LUT_POINTS_MAX == 62U);
    FC_CONST_CHECK(APP_FORCE_CALIB_LUT_POINTS_MAX <= APP_FORCE_CALIB_POINTS_MAX);
    FC_CONST_CHECK(sizeof(fc_flash_blob_t) <= FC_FLASH_PART_SLOT);
    FC_CONST_CHECK((FORCE_CALIBRATION_MAX_PARTS * FC_FLASH_PART_SLOT) <= HC_FLASH_CALIB_SIZE);

    /* 完整性判定口径的前置条件（区域按物理节段划分，最多 3 区）：
     * 节段表 s_au8PartSubCnt 的元素值无法用 FC_CONST_CHECK 断言（静态数组元素不是
     * 整型常量表达式），其取值范围改由 fc_calib_is_complete() 在运行期校验 */
    FC_CONST_CHECK(FORCE_CALIBRATION_MAX_PARTS == 6U);
    FC_CONST_CHECK(FORCE_CALIBRATION_MAX_REGIONS >= 3U);
    FC_CONST_CHECK(APP_FORCE_CALIB_REGION_MAX <= FORCE_CALIBRATION_MAX_REGIONS);
}

/**
 * @brief  标定表完整性判定：6 个部位区域数是否全达标且每区域有效
 * @retval true  6 个部位的已存储区域数均等于各自物理节段数，且每个区域都有
 *               效（区域 LUT 点数 >= 2）
 * @retval false 任一部位区域数不符/超范围，或存在无效区域
 * @note   期望区域数的唯一来源是物理节段表 s_au8PartSubCnt（拇指 2 / 四指 3 /
 *         掌心 1），复用而非另建一份表，避免两处漂移；
 *         只读已装载的运行表，不访问 Flash；仅在上电加载后被调用一次。
 */
static bool fc_calib_is_complete(void)
{
    uint8_t  part;
    uint8_t  expect;
    uint8_t  region_count = 0U;
    uint16_t mask = 0U;

    for (part = 1U; part <= FORCE_CALIBRATION_MAX_PARTS; part++)
    {
        expect = s_au8PartSubCnt[part - 1U];

        /* 期望区域数越界（布局表被改坏）：直接判不完整，避免以越界位数构造掩码 */
        if ((expect < 1U) || (expect > FORCE_CALIBRATION_MAX_REGIONS))
        {
            return false;
        }

        if (false == force_calibration_get_status(&s_stCalib, part, &region_count, &mask))
        {
            return false;
        }

        /* 区域数必须恰好等于节段数，且每个区域都有效（mask 低 expect 位全 1） */
        if ((region_count != expect) || (mask != (uint16_t)((1UL << expect) - 1UL)))
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief  初始化标定核心并加载 6 个部位的 Flash 标定表（接口说明见 app_force_calib.h）
 * @note   须在 HcFlash_Init() 之后调用；可在调度器启动前调用（只读 Flash）；
 *         加载完成后立即做一次完整性判定并打印结果（打印通道关闭时无输出）
 */
void AppForceCalib_Init(void)
{
    force_calibration_port_t port;

    fc_layout_self_check();

    port.load      = fc_port_load;
    port.save      = fc_port_save;
    port.region_id = fc_port_region_id;
    port.context   = NULL;

    (void)force_calibration_init(&s_stCalib, &port, FORCE_CALIBRATION_MAX_PARTS);

    /* 加载完成后判定一次标定表完整性（一个上电周期内固定：PUT/擦除后不重判） */
    s_bCalibComplete = fc_calib_is_complete();

    /* 清空区域总和 IIR 平滑状态：新上电/新标定表一律从瞬时值起算，无跨表残留 */
    AppForceCalib_ResetSmoothState();

    /* 判定结果打印：LL_PRINT_ENABLE=DDL_OFF 时 DDL_Printf 为空函数，无输出也无代码 */
    DDL_Printf("[FC] calib %s (parts=%u, expect thumb2/finger3/palm1)\r\n",
               (true == s_bCalibComplete) ? "COMPLETE" : "INCOMPLETE",
               (unsigned int)FORCE_CALIBRATION_MAX_PARTS);
}

/**
 * @brief  查询标定表是否"完整"（接口说明见 app_force_calib.h）
 */
bool AppForceCalib_IsCalibComplete(void)
{
    return s_bCalibComplete;
}

/**
 * @brief  清空"区域 AD 总和"平滑状态（接口说明见 app_force_calib.h）
 */
void AppForceCalib_ResetSmoothState(void)
{
#if (APP_FORCE_CALIB_SUM_SMOOTH_EN != 0U)
    (void)memset((void *)s_au32SumSmooth, 0, sizeof(s_au32SumSmooth));
#endif
}

/**
 * @brief  查询部位标定状态（接口说明见 app_force_calib.h）
 */
bool AppForceCalib_GetStatus(uint8_t part, uint8_t *region_count, uint16_t *mask)
{
    return force_calibration_get_status(&s_stCalib, part, region_count, mask);
}

/**
 * @brief  整体替换部位全部区域 LUT（接口说明见 app_force_calib.h）
 * @note   调用上下文：usart 接收任务（单实例串行），期间须暂停 0x02 周期上传
 */
bool AppForceCalib_SetPart(uint8_t part, const app_force_calib_table_t *tables, uint8_t region_count)
{
    return force_calibration_set_part(&s_stCalib, part, tables, region_count);
}

/**
 * @brief  擦除部位标定表（接口说明见 app_force_calib.h）
 */
bool AppForceCalib_ClearPart(uint8_t part)
{
    return force_calibration_clear_part(&s_stCalib, part);
}

/**
 * @brief  部位上传矩形行数（接口说明见 app_force_calib.h）
 */
uint8_t AppForceCalib_PartRows(uint8_t part)
{
    return ((part >= 1U) && (part <= APP_FORCE_CALIB_PART_MAX)) ? s_au8PartRows[part - 1U] : 0U;
}

/**
 * @brief  部位上传矩形列数（接口说明见 app_force_calib.h）
 */
uint8_t AppForceCalib_PartCols(uint8_t part)
{
    return ((part >= 1U) && (part <= APP_FORCE_CALIB_PART_MAX)) ? s_au8PartCols[part - 1U] : 0U;
}

/**
 * @brief  装配部位整矩形矩阵（行优先，含无效点占位；接口说明见 app_force_calib.h）
 * @note   仅调试/兼容用；0x02 上传帧请用 AppForceCalib_LoadPartStream
 */
void AppForceCalib_LoadPartMatrix(const tac_data_cfg_t *src, uint8_t part, uint8_t *dst)
{
    uint8_t rows, cols, sub, cnt;
    uint8_t r, c;

    if ((part < 1U) || (part > APP_FORCE_CALIB_PART_MAX) || (NULL == src) || (NULL == dst))
    {
        return;
    }

    rows = s_au8PartRows[part - 1U];
    cols = s_au8PartCols[part - 1U];

    memset(dst, 0, (uint32_t)rows * (uint32_t)cols);

    cnt = s_au8PartSubCnt[part - 1U];

    for (sub = 0U; sub < cnt; sub++)
    {
        const fc_subarea_t *sa  = &s_astPartLayout[part - 1U][sub];
        const uint8_t      *p   = fc_subarea_data_ptr((tac_data_cfg_t *)src, part, sub);

        for (r = 0U; r < sa->rows; r++)
        {
            for (c = 0U; c < sa->cols; c++)
            {
                dst[(((uint16_t)sa->row_off + (uint16_t)r) * (uint16_t)cols) + (uint16_t)c] =
                    p[((uint16_t)r * (uint16_t)sa->cols) + (uint16_t)c];
            }
        }
    }
}

/**
 * @brief  按传感点编号升序装配部位上传数据流（0x02 上传帧 data 区）
 * @param  src   采样快照/工作区（只读）
 * @param  part  部位 1~6
 * @param  dst   输出缓冲：dst[i] = 该部位编号 (i+1) 的传感点值，仅前 N 字节有效
 * @param  size  dst 容量（须 ≥ 该部位有效点数，最大 141）
 * @retval 有效点数 N（拇指 101 / 食指 130 / 中指 137 / 无名指 130 / 小指 111 /
 *         掌心 141）；参数非法、掩码不可用或容量不足返回 0
 * @note   data 段发送结构：子矩阵顺序 dip→pip→mcp（拇指 dip→mcp、掌心单矩阵），
 *         段内按掩码局部编号升序连续排列：dst[段基址 + 编号 - 1] = 该格采样值；
 *         掩码为 0 的无效点不占位，故字节序与上位机 mapping 编号一一对应
 *         （如 thumb_dip 编号 1..59 之后紧接 thumb_mcp 编号 1..42）。
 *         data 区只发送这 N 个字节，无效点既不占位也不发送（不补 0），
 *         故 0x02 帧长度随 N 而定，与 rows×cols 无关。
 *         段基址累加"段内最大编号"（当前掩码编号连续，与有效格数相等），
 *         掩码若跳号也能保持字节序与编号一致。
 *         掩码为 const 只读表（同 fc_port_region_id），读侧免锁。
 */
uint16_t AppForceCalib_LoadPartStream(const tac_data_cfg_t *src, uint8_t part,
                                      uint8_t *dst, uint16_t size)
{
    const tac_data_cfg_t *mask = GetTacMaskData();
    uint8_t  sub, cnt, r, c;
    uint16_t base = 0U;

    if ((part < 1U) || (part > APP_FORCE_CALIB_PART_MAX) ||
        (NULL == src) || (NULL == dst) || (NULL == mask) || (0U == size))
    {
        return 0U;
    }

    cnt = s_au8PartSubCnt[part - 1U];

    for (sub = 0U; sub < cnt; sub++)
    {
        const fc_subarea_t *sa = &s_astPartLayout[part - 1U][sub];
        const uint8_t      *m  = fc_subarea_data_ptr((tac_data_cfg_t *)mask, part, sub);
        const uint8_t      *p  = fc_subarea_data_ptr((tac_data_cfg_t *)src, part, sub);
        uint16_t            sub_num = 0U;   /* 本子矩阵编号上限（= 有效点数） */

        if ((NULL == m) || (NULL == p))
        {
            return 0U;
        }

        for (r = 0U; r < sa->rows; r++)
        {
            for (c = 0U; c < sa->cols; c++)
            {
                uint8_t id = m[(((uint16_t)r) * (uint16_t)sa->cols) + (uint16_t)c];

                if (id > sub_num)
                {
                    sub_num = (uint16_t)id;
                }
            }
        }

        if (((uint16_t)(base + sub_num)) > size)
        {
            return 0U;   /* 容量不足：整帧作废，避免短帧被误解析 */
        }

        for (r = 0U; r < sa->rows; r++)
        {
            for (c = 0U; c < sa->cols; c++)
            {
                uint8_t id = m[(((uint16_t)r) * (uint16_t)sa->cols) + (uint16_t)c];

                if (id != 0U)
                {
                    dst[(base + (uint16_t)id) - 1U] =
                        p[(((uint16_t)r) * (uint16_t)sa->cols) + (uint16_t)c];
                }
            }
        }

        base += sub_num;
    }

    return base;
}

/**
 * @brief  对部位子矩阵执行"已标定 -> LUT 换算"（接口说明见 app_force_calib.h）
 * @note   未标定部位直接返回，cfg 保持减零点后的 AD 增量；
 *         调用上下文：采样任务整轮写临界段内（经静态缓存 s_au16PartBuf，单实例禁重入）
 */
void AppForceCalib_ApplyPart(uint8_t part, tac_data_cfg_t *cfg)
{
    uint8_t  region_count = 0U;
    uint16_t mask = 0U;
    uint8_t  rows, cols, sub, cnt;
    uint8_t  r, c;

    if ((part < 1U) || (part > APP_FORCE_CALIB_PART_MAX) || (NULL == cfg))
    {
        return;
    }

    /* 门控：标定表不完整时不做任何换算，6 个部位统一输出减零点后的 AD 增量；
       判定值只在上电加载时算一次，此处仅读静态标志，无额外开销 */
    if (false == s_bCalibComplete)
    {
        return;
    }

    (void)force_calibration_get_status(&s_stCalib, part, &region_count, &mask);

    if (0U == mask)
    {
        return; /* 未标定：保持减零点后的 AD 增量，供标定采集 */
    }

    rows = s_au8PartRows[part - 1U];
    cols = s_au8PartCols[part - 1U];

    memset(s_au16PartBuf, 0, sizeof(s_au16PartBuf));

    /* 部位子矩阵（行优先）拼入矩形缓存；补零列/行保持 0（不影响区域求和） */
    cnt = s_au8PartSubCnt[part - 1U];

    for (sub = 0U; sub < cnt; sub++)
    {
        const fc_subarea_t *sa  = &s_astPartLayout[part - 1U][sub];
        const uint8_t      *src = fc_subarea_data_ptr(cfg, part, sub);

        for (r = 0U; r < sa->rows; r++)
        {
            for (c = 0U; c < sa->cols; c++)
            {
                s_au16PartBuf[(((uint16_t)sa->row_off + (uint16_t)r) * (uint16_t)cols) + (uint16_t)c] =
                    (uint16_t)src[((uint16_t)r * (uint16_t)sa->cols) + (uint16_t)c];
            }
        }
    }

    force_calibration_apply(&s_stCalib, part, s_au16PartBuf, rows, cols, cols);

    /* 换算结果写回子矩阵（0~255 直接回写 uint8） */
    for (sub = 0U; sub < cnt; sub++)
    {
        const fc_subarea_t *sa  = &s_astPartLayout[part - 1U][sub];
        uint8_t            *dst = fc_subarea_data_ptr(cfg, part, sub);

        for (r = 0U; r < sa->rows; r++)
        {
            for (c = 0U; c < sa->cols; c++)
            {
                dst[((uint16_t)r * (uint16_t)sa->cols) + (uint16_t)c] =
                    (uint8_t)s_au16PartBuf[(((uint16_t)sa->row_off + (uint16_t)r) * (uint16_t)cols) + (uint16_t)c];
            }
        }
    }
}

/*******************************************************************************
 * 4. Flash 存取测试：回读校验 + 打印（接口说明见 app_force_calib.h）
 ******************************************************************************/
#if (APP_FORCE_CALIB_FLASH_TEST_EN != 0U)
/* 部位名称（仅打印用；此表也用于区分日志行属于哪个部位） */
static const char * const s_apcPartName[APP_FORCE_CALIB_PART_MAX] =
{
    "thumb", "index", "middle", "ring", "little", "palm"
};

/**
 * @brief  按轴打印 LUT 数值（每 16 点折行，便于与上位机下发帧逐值核对）
 * @param  part    部位 1~6（仅用于打印）
 * @param  region  区域号 1~3（仅用于打印）
 * @param  axis    轴名，"ad" 或 "fr"
 * @param  data    uint16 表值数组
 * @param  count   有效点数
 * @note   仅 APP_FORCE_CALIB_FLASH_TEST_EN != 0 时参与编译
 */
static void fc_print_axis(uint8_t part, uint8_t region, const char *axis,
                          const uint16_t *data, uint8_t count)
{
    uint8_t i;

    for (i = 0U; i < count; i++)
    {
        if (0U == (i % 16U))
        {
            DDL_Printf("\r\n[FC]   p%u r%u %s:", (unsigned int)part, (unsigned int)region, axis);
        }

        DDL_Printf(" %u", (unsigned int)data[i]);
    }

    DDL_Printf("\r\n");
}
#endif /* APP_FORCE_CALIB_FLASH_TEST_EN */

/**
 * @brief  回读 blob 是否为"已标定记录"
 * @param  blob  直接读自 Flash 的紧凑 blob
 * @retval true  已标定；false 空槽（全 0xFF）/脏数据/字段非法
 * @note   判据与 fc_blob_to_record 一致：magic=='HCS2'、region_count 1~3、
 *         每区域 2~LUT_MAX 点、AD 严格递增、力值不降
 */
static bool fc_readback_blob_valid(const fc_flash_blob_t *blob)
{
    uint8_t  r;
    uint16_t i;

    if (blob->magic != FC_FLASH_BLOB_MAGIC)
    {
        return false;
    }

    if ((blob->region_count < 1U) || (blob->region_count > FORCE_CALIBRATION_MAX_REGIONS))
    {
        return false;
    }

    for (r = 0U; r < blob->region_count; r++)
    {
        if ((blob->region[r].count < 2U) || (blob->region[r].count > FORCE_CALIBRATION_MAX_POINTS))
        {
            return false;
        }

        for (i = 0U; i < blob->region[r].count; i++)
        {
            if ((i > 0U) &&
                ((blob->region[r].adc[i] <= blob->region[r].adc[i - 1U]) ||
                 (blob->region[r].force[i] < blob->region[r].force[i - 1U])))
            {
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief  回读 Flash 标定表并与 RAM 镜像/运行表比对（接口说明见 app_force_calib.h）
 * @note   三重判定：① CalibRead 成功且 blob 头部合法；② 与 RAM 常驻镜像逐字节一致；
 *         ③ 与运行表逐点数值一致。回读内容同时留在 s_stReadbackBlob 供 AppForceCalib_DumpPart 使用
 */
bool AppForceCalib_ReadbackVerify(uint8_t part, uint16_t *pu16FirstDiff)
{
    const fc_flash_blob_t           *blob;
    const force_calibration_record_t *record;
    const uint8_t                   *pu8Flash;
    const uint8_t                   *pu8Mirror;
    uint32_t u32Off;
    uint16_t u16Diff = 0U;
    uint8_t  r;
    uint16_t i;
    bool     ok = true;

    if (NULL != pu16FirstDiff)
    {
        *pu16FirstDiff = 0U;
    }

    if ((part < 1U) || (part > FORCE_CALIBRATION_MAX_PARTS))
    {
        return false;
    }

    /* ① 直接读 Flash 落盘内容（不走 RAM 镜像） */
    if (LL_OK != HcFlash_CalibRead((uint32_t)(part - 1U) * FC_FLASH_PART_SLOT,
                                   (uint8_t *)&s_stReadbackBlob,
                                   (uint32_t)sizeof(fc_flash_blob_t)))
    {
        return false;
    }

    blob = &s_stReadbackBlob;

    /* 空槽/脏数据：该部位本就未标定，不算失败（上层用 AppForceCalib_GetStatus 判断） */
    if (false == fc_readback_blob_valid(blob))
    {
        return true;
    }

    /* ② 与 RAM 常驻镜像逐字节比对：验证"写入（擦除+回写+校验）"链路完整 */
    pu8Flash  = (const uint8_t *)blob;
    pu8Mirror = (const uint8_t *)&s_astFlashBlob[part - 1U];

    for (u32Off = 0UL; u32Off < (uint32_t)sizeof(fc_flash_blob_t); u32Off++)
    {
        if (pu8Flash[u32Off] != pu8Mirror[u32Off])
        {
            u16Diff = (uint16_t)u32Off;
            ok      = false;
            break;
        }
    }

    /* ③ 与运行表逐点数值比对：验证"加载->发布"链路（表值以 uint16 域为准） */
    if (true == ok)
    {
        record = &s_stCalib.records[part - 1U];

        if ((record->magic != FORCE_CALIBRATION_RECORD_MAGIC) ||
            (record->region_count != blob->region_count))
        {
            ok = false;
        }
        else
        {
            for (r = 0U; (r < blob->region_count) && (true == ok); r++)
            {
                if (record->region[r].count != blob->region[r].count)
                {
                    ok = false;
                    break;
                }

                for (i = 0U; i < blob->region[r].count; i++)
                {
                    if ((record->region[r].adc[i] != blob->region[r].adc[i]) ||
                        (record->region[r].force[i] != blob->region[r].force[i]))
                    {
                        ok = false;
                        break;
                    }
                }
            }
        }
    }

    if (NULL != pu16FirstDiff)
    {
        *pu16FirstDiff = u16Diff;
    }

    return ok;
}

/**
 * @brief  打印部位标定摘要与完整 LUT 数值（数据取自 Flash 回读内容）
 * @note   内部先调 AppForceCalib_ReadbackVerify 刷新 s_stReadbackBlob 再打印；
 *         仅 APP_FORCE_CALIB_FLASH_TEST_EN != 0 时产生输出
 */
void AppForceCalib_DumpPart(uint8_t part)
{
#if (APP_FORCE_CALIB_FLASH_TEST_EN != 0U)
    const fc_flash_blob_t *blob;
    uint16_t u16FirstDiff = 0U;
    uint16_t u16Mask      = 0U;
    uint8_t  u8RegionCnt  = 0U;
    bool     bReadOk;
    uint8_t  r;

    if ((part < 1U) || (part > FORCE_CALIBRATION_MAX_PARTS))
    {
        return;
    }

    /* ReadbackVerify 内部已把 Flash 内容读入 s_stReadbackBlob，随后从该副本打印 */
    bReadOk = AppForceCalib_ReadbackVerify(part, &u16FirstDiff);
    blob    = &s_stReadbackBlob;

    if (false == fc_readback_blob_valid(blob))
    {
        /* 空槽/无效记录（未标定部位）：只打一行，不刷 LUT */
        DDL_Printf("[FC] %s(part%u) uncalibrated%s\r\n",
                   s_apcPartName[part - 1U], (unsigned int)part,
                   (true == bReadOk) ? "" : " (flash read FAIL)");
        return;
    }

    (void)AppForceCalib_GetStatus(part, &u8RegionCnt, &u16Mask);

    DDL_Printf("[FC] %s(part%u) %s regions=%u mask=0x%X slots=%uB\r\n",
               s_apcPartName[part - 1U], (unsigned int)part,
               (true == bReadOk) ? "READBACK-OK" : "READBACK-FAIL",
               (unsigned int)blob->region_count, (unsigned int)u16Mask,
               (unsigned int)sizeof(fc_flash_blob_t));

    if (false == bReadOk)
    {
        DDL_Printf("[FC]   first mismatch at blob offset %u\r\n", (unsigned int)u16FirstDiff);
    }

    for (r = 0U; r < blob->region_count; r++)
    {
        DDL_Printf("[FC]   region%u: %u points\r\n",
                   (unsigned int)(r + 1U), (unsigned int)blob->region[r].count);
        fc_print_axis(part, (uint8_t)(r + 1U), "ad", blob->region[r].adc,   blob->region[r].count);
        fc_print_axis(part, (uint8_t)(r + 1U), "fr", blob->region[r].force, blob->region[r].count);
    }
#else
    (void)part;
#endif /* APP_FORCE_CALIB_FLASH_TEST_EN */
}

/**
 * @brief  上电自检：遍历 6 个部位做 Flash 回读校验并打印（接口说明见 app_force_calib.h）
 * @note   只读 Flash，不含任何写入，可在调度器启动前调用
 */
void AppForceCalib_FlashSelfTest(void)
{
#if (APP_FORCE_CALIB_FLASH_TEST_EN != 0U)
    uint8_t u8Part;
    uint8_t u8Calibrated = 0U;

    DDL_Printf("\r\n[FC] flash self-test begin (blob=%uB slot=%uB calib_area=%uB)\r\n",
               (unsigned int)sizeof(fc_flash_blob_t), (unsigned int)FC_FLASH_PART_SLOT,
               (unsigned int)HC_FLASH_CALIB_SIZE);

    for (u8Part = 1U; u8Part <= FORCE_CALIBRATION_MAX_PARTS; u8Part++)
    {
        uint8_t  u8RegionCnt = 0U;
        uint16_t u16Mask     = 0U;

        AppForceCalib_DumpPart(u8Part);

        if ((true == AppForceCalib_GetStatus(u8Part, &u8RegionCnt, &u16Mask)) && (0U != u16Mask))
        {
            u8Calibrated++;
        }
    }

    if (0U == u8Calibrated)
    {
        DDL_Printf("[FC] no calibrated part: all 6 parts uncalibrated\r\n");
    }

    DDL_Printf("[FC] flash self-test end\r\n\r\n");
#endif /* APP_FORCE_CALIB_FLASH_TEST_EN */
}

