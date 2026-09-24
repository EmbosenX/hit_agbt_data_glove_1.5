/*******************************************************************************
 * @file    app_tac_config.c
 * @brief   触觉采集阵列配置与采集数据发布快照实现（接口说明见 app_tac_config.h）
 * @note    本文件只持有矩阵数据本体与访问接口，不含扫描/换算逻辑：
 *          采集矩阵由采样任务写，掩码矩阵为 const 只读表，快照供跨任务免锁读取。
 ******************************************************************************/
#include "app_tac_config.h"
#include <string.h>


/* 触觉点编号数据选择宏（左右手固有差异项，禁止照抄 HIT_THUMB_PALM_ZS）：
 *   0: 使用 tac_mask_data_ys（本工程取值，右手手套）
 *   1: 使用 tac_mask_data_zs（左手手套，见 HIT_THUMB_PALM_ZS 工程）
 * 两张掩码表的数据内容两侧一致，仅"选哪张"不同；本工程必须保持 0 */
#define TAC_MASK_DATA_SELECT   (0)


static tac_data_cfg_t tac_data_cfg={0};
static tac_data_cfg_t tac_calib_data={0};

/* ====================== 采集数据发布快照（写侧发布 / 读侧免锁取用） ====================== */
/* 双缓冲 + 单字节活动索引交换：
 *   - 写侧（采样任务）发布时整结构拷入"非活动"缓冲，拷贝完成后再切换活动索引；
 *   - 读侧（官方协议应答）只读一次活动索引再取该缓冲，全程不取 m_hSampleDataMutex。
 * 索引为单字节读写，Cortex-M4 上天然原子，故读侧要么得到旧的完整帧、要么得到
 * 新的完整帧，不存在半新半旧的撕裂帧，同时也消除了应答路径对整轮采集写窗口的等待。 */
static tac_data_cfg_t s_astcSnapshot[2] = {0};       /* 快照双缓冲（静态RAM，不占堆） */
static volatile uint8_t s_u8SnapshotActiveIdx = 0U;  /* 当前可读缓冲下标(0/1) */


#if (TAC_MASK_DATA_SELECT==0)
static const tac_data_cfg_t tac_mask_data_ys =
{
	.thumb_dip =
	{
			{   0,   0,   0,   0,   0,   1,   2,   3,   0,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   4,   5,   6,   0,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   7,   8,   9,   0,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,  10,  11,  12,   0,   0,   0,   0,   0,   0 },
			{   0,  13,   0,   0,   0,  14,  15,  16,   0,   0,   0,   0,   0,   0 },
			{  17,  18,   0,   0,   0,  19,  20,  21,  22,   0,   0,   0,  23,   0 },
			{   0,  24,   0,   0,   0,   0,  25,  26,   0,   0,   0,   0,  27,   0 },
			{   0,  28,   0,   0,   0,   0,  29,  30,   0,   0,   0,   0,  31,  32 },
			{   0,  33,   0,   0,   0,   0,  34,  35,   0,   0,   0,   0,  36,  37 },
			{  38,  39,  40,   0,   0,  41,  42,  43,  44,   0,   0,   0,  45,  46 },
			{   0,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59 },
	},
	.thumb_mcp =
	{
			{   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14 },
			{  15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28 },
			{  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42 },
	},
	.index_dip =
	{
			{   0,   0,   0,   0,   0,   1,   2,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   3,   4,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   5,   6,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   7,   8,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   9,  10,   0,   0,   0,   0,  11 },
			{   0,  12,   0,   0,  13,  14,  15,   0,   0,   0,  16,  17 },
			{   0,  18,   0,   0,   0,  19,  20,   0,   0,   0,  21,  22 },
			{  23,  24,   0,   0,   0,  25,  26,   0,   0,   0,  27,  28 },
			{  29,  30,   0,   0,   0,  31,  32,   0,   0,   0,  33,  34 },
			{  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46 },
			{  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58 },
	},
	.index_pip =
	{
			{   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12 },
			{  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24 },
			{  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36 },
	},
	.index_mcp =
	{
			{   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12 },
			{  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24 },
			{  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36 },
	},
	.middle_dip =
	{
			{   0,   0,   0,   0,   0,   1,   2,   0,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   3,   4,   0,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   5,   6,   0,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   7,   8,   0,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   9,  10,   0,   0,   0,   0,  11,   0 },
			{   0,  12,   0,   0,  13,  14,  15,   0,   0,   0,  16,  17,   0 },
			{   0,  18,   0,   0,   0,  19,  20,   0,   0,   0,  21,  22,   0 },
			{  23,  24,   0,   0,   0,  25,  26,   0,   0,   0,  27,  28,   0 },
			{  29,  30,   0,   0,   0,  31,  32,   0,   0,   0,  33,  34,   0 },
			{  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,   0 },
			{  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59 },
	},
	.middle_pip =
	{
			{   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13 },
			{  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26 },
			{  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39 },
	},
	.middle_mcp =
	{
			{   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13 },
			{  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26 },
			{  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39 },
	},
	.ring_dip =
	{
			{   0,   0,   0,   0,   0,   1,   2,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   3,   4,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   5,   6,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   7,   8,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   9,  10,   0,   0,   0,   0,  11 },
			{   0,  12,   0,   0,  13,  14,  15,   0,   0,   0,  16,  17 },
			{   0,  18,   0,   0,   0,  19,  20,   0,   0,   0,  21,  22 },
			{  23,  24,   0,   0,   0,  25,  26,   0,   0,   0,  27,  28 },
			{  29,  30,   0,   0,   0,  31,  32,   0,   0,   0,  33,  34 },
			{  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46 },
			{  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58 },
	},
	.ring_pip =
	{
			{   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12 },
			{  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24 },
			{  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36 },
	},
	.ring_mcp =
	{
			{   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12 },
			{  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24 },
			{  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36 },
	},
	.little_dip =
	{
			{   0,   0,   0,   0,   1,   2,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   3,   4,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   5,   6,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   7,   8,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   9,  10,   0,   0,   0,   0 },
			{   0,   0,   0,  11,  12,  13,   0,   0,   0,   0 },
			{  14,   0,   0,   0,  15,  16,   0,   0,   0,  17 },
			{  18,   0,   0,   0,  19,  20,   0,   0,   0,  21 },
			{  22,   0,   0,   0,  23,  24,   0,   0,   0,  25 },
			{  26,  27,  28,  29,  30,  31,  32,  33,  34,  35 },
			{  36,  37,  38,  39,  40,  41,  42,  43,  44,  45 },
	},
	.little_pip =
	{
			{   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11 },
			{  12,  13,  14,  15,  16,  17,  18,  19,  20,  21,  22 },
			{  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33 },
	},
	.little_mcp =
	{
			{   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11 },
			{  12,  13,  14,  15,  16,  17,  18,  19,  20,  21,  22 },
			{  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33 },
	},
	.palm =
	{
			{   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15,  16 },
			{  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32 },
			{  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48 },
			{  49,  50,  51,  52,  53,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0 },
			{  54,  55,  56,  57,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0 },
			{  58,  59,  60,  61,   0,   0,   0,   0,   0,   0,   0,   0,  62,   0,   0,   0 },
			{  63,  64,  65,   0,   0,   0,   0,   0,   0,   0,   0,  66,  67,   0,   0,   0 },
			{  68,  69,  70,   0,   0,   0,   0,   0,   0,   0,   0,  71,  72,  73,   0,   0 },
			{  74,  75,  76,   0,   0,   0,   0,   0,   0,   0,  77,  78,  79,  80,  81,   0 },
			{   0,  82,  83,   0,   0,   0,   0,   0,   0,   0,  84,  85,  86,  87,  88,   0 },
			{   0,  89,  90,   0,   0,   0,   0,   0,   0,   0,  91,  92,  93,  94,  95,   0 },
			{   0,  96,  97,  98,  99, 100, 101, 102, 103, 104, 105, 106, 107, 108,   0,   0 },
			{   0, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120,   0,   0,   0 },
			{   0, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131,   0,   0,   0,   0 },
			{   0, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141,   0,   0,   0,   0,   0 },
	}
};
#else
//上传 data 段按掩码矩阵内编号顺序、子矩阵 dip→mcp 依次拼接
static const tac_data_cfg_t tac_mask_data_zs =
{
	.thumb_dip =
	{
			{   0,   0,   0,   0,   0,   0,   3,   2,   1,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   0,   6,   5,   4,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   0,   9,   8,   7,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   0,  12,  11,  10,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   0,  16,  15,  14,   0,   0,   0,  13,   0 },
			{   0,  23,   0,   0,   0,  22,  21,  20,  19,   0,   0,   0,  18,  17 },
			{   0,  27,   0,   0,   0,   0,  26,  25,   0,   0,   0,   0,  24,   0 },
			{  32,  31,   0,   0,   0,   0,  30,  29,   0,   0,   0,   0,  28,   0 },
			{  37,  36,   0,   0,   0,   0,  35,  34,   0,   0,   0,   0,  33,   0 },
			{  46,  45,   0,   0,   0,  44,  43,  42,  41,   0,   0,  40,  39,  38 },
			{  59,  58,  57,  56,  55,  54,  53,  52,  51,  50,  49,  48,  47,   0 },
	},
	.thumb_mcp =
	{
			{  14,  13,  12,  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1 },
			{  28,  27,  26,  25,  24,  23,  22,  21,  20,  19,  18,  17,  16,  15 },
			{  42,  41,  40,  39,  38,  37,  36,  35,  34,  33,  32,  31,  30,  29 },
	},
	.index_dip =
	{
			{   0,   0,   0,   0,   0,   2,   1,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   4,   3,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   6,   5,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   8,   7,   0,   0,   0,   0,   0 },
			{  11,   0,   0,   0,   0,  10,   9,   0,   0,   0,   0,   0 },
			{  17,  16,   0,   0,   0,  15,  14,  13,   0,   0,  12,   0 },
			{  22,  21,   0,   0,   0,  20,  19,   0,   0,   0,  18,   0 },
			{  28,  27,   0,   0,   0,  26,  25,   0,   0,   0,  24,  23 },
			{  34,  33,   0,   0,   0,  32,  31,   0,   0,   0,  30,  29 },
			{  46,  45,  44,  43,  42,  41,  40,  39,  38,  37,  36,  35 },
			{  58,  57,  56,  55,  54,  53,  52,  51,  50,  49,  48,  47 },
	},
	.index_pip =
	{
			{  12,  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1 },
			{  24,  23,  22,  21,  20,  19,  18,  17,  16,  15,  14,  13 },
			{  36,  35,  34,  33,  32,  31,  30,  29,  28,  27,  26,  25 },
	},
	.index_mcp =
	{
			{  12,  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1 },
			{  24,  23,  22,  21,  20,  19,  18,  17,  16,  15,  14,  13 },
			{  36,  35,  34,  33,  32,  31,  30,  29,  28,  27,  26,  25 },
	},
	.middle_dip =
	{
			{   0,   0,   0,   0,   0,   0,   2,   1,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   0,   4,   3,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   0,   6,   5,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   0,   8,   7,   0,   0,   0,   0,   0 },
			{   0,  11,   0,   0,   0,   0,  10,   9,   0,   0,   0,   0,   0 },
			{   0,  17,  16,   0,   0,   0,  15,  14,  13,   0,   0,  12,   0 },
			{   0,  22,  21,   0,   0,   0,  20,  19,   0,   0,   0,  18,   0 },
			{   0,  28,  27,   0,   0,   0,  26,  25,   0,   0,   0,  24,  23 },
			{   0,  34,  33,   0,   0,   0,  32,  31,   0,   0,   0,  30,  29 },
			{   0,  46,  45,  44,  43,  42,  41,  40,  39,  38,  37,  36,  35 },
			{  59,  58,  57,  56,  55,  54,  53,  52,  51,  50,  49,  48,  47 },
	},
	.middle_pip =
	{
			{  13,  12,  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1 },
			{  26,  25,  24,  23,  22,  21,  20,  19,  18,  17,  16,  15,  14 },
			{  39,  38,  37,  36,  35,  34,  33,  32,  31,  30,  29,  28,  27 },
	},
	.middle_mcp =
	{
			{  13,  12,  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1 },
			{  26,  25,  24,  23,  22,  21,  20,  19,  18,  17,  16,  15,  14 },
			{  39,  38,  37,  36,  35,  34,  33,  32,  31,  30,  29,  28,  27 },
	},
	.ring_dip =
	{
			{   0,   0,   0,   0,   0,   2,   1,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   4,   3,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   6,   5,   0,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   0,   8,   7,   0,   0,   0,   0,   0 },
			{  11,   0,   0,   0,   0,  10,   9,   0,   0,   0,   0,   0 },
			{  17,  16,   0,   0,   0,  15,  14,  13,   0,   0,  12,   0 },
			{  22,  21,   0,   0,   0,  20,  19,   0,   0,   0,  18,   0 },
			{  28,  27,   0,   0,   0,  26,  25,   0,   0,   0,  24,  23 },
			{  34,  33,   0,   0,   0,  32,  31,   0,   0,   0,  30,  29 },
			{  46,  45,  44,  43,  42,  41,  40,  39,  38,  37,  36,  35 },
			{  58,  57,  56,  55,  54,  53,  52,  51,  50,  49,  48,  47 },
	},
	.ring_pip =
	{
			{  12,  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1 },
			{  24,  23,  22,  21,  20,  19,  18,  17,  16,  15,  14,  13 },
			{  36,  35,  34,  33,  32,  31,  30,  29,  28,  27,  26,  25 },
	},
	.ring_mcp =
	{
			{  12,  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1 },
			{  24,  23,  22,  21,  20,  19,  18,  17,  16,  15,  14,  13 },
			{  36,  35,  34,  33,  32,  31,  30,  29,  28,  27,  26,  25 },
	},
	.little_dip =
	{
			{   0,   0,   0,   0,   2,   1,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   4,   3,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   6,   5,   0,   0,   0,   0 },
			{   0,   0,   0,   0,   8,   7,   0,   0,   0,   0 },
			{   0,   0,   0,   0,  10,   9,   0,   0,   0,   0 },
			{   0,   0,   0,   0,  13,  12,  11,   0,   0,   0 },
			{  17,   0,   0,   0,  16,  15,   0,   0,   0,  14 },
			{  21,   0,   0,   0,  20,  19,   0,   0,   0,  18 },
			{  25,   0,   0,   0,  24,  23,   0,   0,   0,  22 },
			{  35,  34,  33,  32,  31,  30,  29,  28,  27,  26 },
			{  45,  44,  43,  42,  41,  40,  39,  38,  37,  36 },
	},
	.little_pip =
	{
			{  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1 },
			{  22,  21,  20,  19,  18,  17,  16,  15,  14,  13,  12 },
			{  33,  32,  31,  30,  29,  28,  27,  26,  25,  24,  23 },
	},
	.little_mcp =
	{
			{  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1 },
			{  22,  21,  20,  19,  18,  17,  16,  15,  14,  13,  12 },
			{  33,  32,  31,  30,  29,  28,  27,  26,  25,  24,  23 },
	},
	.palm =
	{
			{  16,  15,  14,  13,  12,  11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1 },
			{  32,  31,  30,  29,  28,  27,  26,  25,  24,  23,  22,  21,  20,  19,  18,  17 },
			{  48,  47,  46,  45,  44,  43,  42,  41,  40,  39,  38,  37,  36,  35,  34,  33 },
			{   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  53,  52,  51,  50,  49 },
			{   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  57,  56,  55,  54 },
			{   0,   0,   0,  62,   0,   0,   0,   0,   0,   0,   0,   0,  61,  60,  59,  58 },
			{   0,   0,   0,  67,  66,   0,   0,   0,   0,   0,   0,   0,   0,  65,  64,  63 },
			{   0,   0,  73,  72,  71,   0,   0,   0,   0,   0,   0,   0,   0,  70,  69,  68 },
			{   0,  81,  80,  79,  78,  77,   0,   0,   0,   0,   0,   0,   0,  76,  75,  74 },
			{   0,  88,  87,  86,  85,  84,   0,   0,   0,   0,   0,   0,   0,  83,  82,   0 },
			{   0,  95,  94,  93,  92,  91,   0,   0,   0,   0,   0,   0,   0,  90,  89,   0 },
			{   0,   0, 108, 107, 106, 105, 104, 103, 102, 101, 100,  99,  98,  97,  96,   0 },
			{   0,   0,   0, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 109,   0 },
			{   0,   0,   0,   0, 131, 130, 129, 128, 127, 126, 125, 124, 123, 122, 121,   0 },
			{   0,   0,   0,   0,   0, 141, 140, 139, 138, 137, 136, 135, 134, 133, 132,   0 },
	}
};
#endif
/* 全局区域范围表 - 可直接通过枚举索引 */
const tac_area_range_t TacAreaRange[TAC_AREA_MAX] = 
{
    /* 大拇指：远节+近节 */
    [TAC_AREA_THUMB_DIP] = {
        .row_start = TAC_ROW_THUMB_DIP_START,
        .row_end   = TAC_ROW_THUMB_DIP_END,
        .col_start = TAC_COL_THUMB_DIP_START,
        .col_end   = TAC_COL_THUMB_DIP_END,
        .point_cnt = (TAC_ROW_THUMB_DIP_END - TAC_ROW_THUMB_DIP_START + 1U) *
                     (TAC_COL_THUMB_DIP_END - TAC_COL_THUMB_DIP_START + 1U)
    },
    [TAC_AREA_THUMB_MCP] = {
        .row_start = TAC_ROW_THUMB_MCP_START,
        .row_end   = TAC_ROW_THUMB_MCP_END,
        .col_start = TAC_COL_THUMB_MCP_START,
        .col_end   = TAC_COL_THUMB_MCP_END,
        .point_cnt = (TAC_ROW_THUMB_MCP_END - TAC_ROW_THUMB_MCP_START + 1U) *
                     (TAC_COL_THUMB_MCP_END - TAC_COL_THUMB_MCP_START + 1U)
    },

    /* 食指：远节+中节+近节 */
		[TAC_AREA_INDEX_DIP] = {
        .row_start = TAC_ROW_INDEX_DIP_START,
        .row_end   = TAC_ROW_INDEX_DIP_END,
        .col_start = TAC_COL_INDEX_DIP_START,
        .col_end   = TAC_COL_INDEX_DIP_END,
        .point_cnt = (TAC_ROW_INDEX_DIP_END - TAC_ROW_INDEX_DIP_START + 1U) *
                     (TAC_COL_INDEX_DIP_END - TAC_COL_INDEX_DIP_START + 1U)
    },
    [TAC_AREA_INDEX_PIP] = {
        .row_start = TAC_ROW_INDEX_PIP_START,
        .row_end   = TAC_ROW_INDEX_PIP_END,
        .col_start = TAC_COL_INDEX_PIP_START,
        .col_end   = TAC_COL_INDEX_PIP_END,
        .point_cnt = (TAC_ROW_INDEX_PIP_END - TAC_ROW_INDEX_PIP_START + 1U) *
                     (TAC_COL_INDEX_PIP_END - TAC_COL_INDEX_PIP_START + 1U)
    },
    [TAC_AREA_INDEX_MCP] = {
        .row_start = TAC_ROW_INDEX_MCP_START,
        .row_end   = TAC_ROW_INDEX_MCP_END,
        .col_start = TAC_COL_INDEX_MCP_START,
        .col_end   = TAC_COL_INDEX_MCP_END,
        .point_cnt = (TAC_ROW_INDEX_MCP_END - TAC_ROW_INDEX_MCP_START + 1U) *
                     (TAC_COL_INDEX_MCP_END - TAC_COL_INDEX_MCP_START + 1U)
    },

    /* 中指：远节+中节+近节 */
		[TAC_AREA_MIDDLE_DIP] = {
        .row_start = TAC_ROW_MIDDLE_DIP_START,
        .row_end   = TAC_ROW_MIDDLE_DIP_END,
        .col_start = TAC_COL_MIDDLE_DIP_START,
        .col_end   = TAC_COL_MIDDLE_DIP_END,
        .point_cnt = (TAC_ROW_MIDDLE_DIP_END - TAC_ROW_MIDDLE_DIP_START + 1U) *
                     (TAC_COL_MIDDLE_DIP_END - TAC_COL_MIDDLE_DIP_START + 1U)
    },
    [TAC_AREA_MIDDLE_PIP] = {
        .row_start = TAC_ROW_MIDDLE_PIP_START,
        .row_end   = TAC_ROW_MIDDLE_PIP_END,
        .col_start = TAC_COL_MIDDLE_PIP_START,
        .col_end   = TAC_COL_MIDDLE_PIP_END,
        .point_cnt = (TAC_ROW_MIDDLE_PIP_END - TAC_ROW_MIDDLE_PIP_START + 1U) *
                     (TAC_COL_MIDDLE_PIP_END - TAC_COL_MIDDLE_PIP_START + 1U)
    },
    [TAC_AREA_MIDDLE_MCP] = {
        .row_start = TAC_ROW_MIDDLE_MCP_START,
        .row_end   = TAC_ROW_MIDDLE_MCP_END,
        .col_start = TAC_COL_MIDDLE_MCP_START,
        .col_end   = TAC_COL_MIDDLE_MCP_END,
        .point_cnt = (TAC_ROW_MIDDLE_MCP_END - TAC_ROW_MIDDLE_MCP_START + 1U) *
                     (TAC_COL_MIDDLE_MCP_END - TAC_COL_MIDDLE_MCP_START + 1U)
    },

    /* 无名指：远节+中节+近节 */
		[TAC_AREA_RING_DIP] = {
        .row_start = TAC_ROW_RING_DIP_START,
        .row_end   = TAC_ROW_RING_DIP_END,
        .col_start = TAC_COL_RING_DIP_START,
        .col_end   = TAC_COL_RING_DIP_END,
        .point_cnt = (TAC_ROW_RING_DIP_END - TAC_ROW_RING_DIP_START + 1U) *
                     (TAC_COL_RING_DIP_END - TAC_COL_RING_DIP_START + 1U)
    },
    [TAC_AREA_RING_PIP] = {
        .row_start = TAC_ROW_RING_PIP_START,
        .row_end   = TAC_ROW_RING_PIP_END,
        .col_start = TAC_COL_RING_PIP_START,
        .col_end   = TAC_COL_RING_PIP_END,
        .point_cnt = (TAC_ROW_RING_PIP_END - TAC_ROW_RING_PIP_START + 1U) *
                     (TAC_COL_RING_PIP_END - TAC_COL_RING_PIP_START + 1U)
    },
    [TAC_AREA_RING_MCP] = {
        .row_start = TAC_ROW_RING_MCP_START,
        .row_end   = TAC_ROW_RING_MCP_END,
        .col_start = TAC_COL_RING_MCP_START,
        .col_end   = TAC_COL_RING_MCP_END,
        .point_cnt = (TAC_ROW_RING_MCP_END - TAC_ROW_RING_MCP_START + 1U) *
                     (TAC_COL_RING_MCP_END - TAC_COL_RING_MCP_START + 1U)
    },

    /* 小指：远节+中节+近节 */
		[TAC_AREA_LITTLE_DIP] = {
        .row_start = TAC_ROW_LITTLE_DIP_START,
        .row_end   = TAC_ROW_LITTLE_DIP_END,
        .col_start = TAC_COL_LITTLE_DIP_START,
        .col_end   = TAC_COL_LITTLE_DIP_END,
        .point_cnt = (TAC_ROW_LITTLE_DIP_END - TAC_ROW_LITTLE_DIP_START + 1U) *
                     (TAC_COL_LITTLE_DIP_END - TAC_COL_LITTLE_DIP_START + 1U)
    },
    [TAC_AREA_LITTLE_PIP] = {
        .row_start = TAC_ROW_LITTLE_PIP_START,
        .row_end   = TAC_ROW_LITTLE_PIP_END,
        .col_start = TAC_COL_LITTLE_PIP_START,
        .col_end   = TAC_COL_LITTLE_PIP_END,
        .point_cnt = (TAC_ROW_LITTLE_PIP_END - TAC_ROW_LITTLE_PIP_START + 1U) *
                     (TAC_COL_LITTLE_PIP_END - TAC_COL_LITTLE_PIP_START + 1U)
    },
    [TAC_AREA_LITTLE_MCP] = {
        .row_start = TAC_ROW_LITTLE_MCP_START,
        .row_end   = TAC_ROW_LITTLE_MCP_END,
        .col_start = TAC_COL_LITTLE_MCP_START,
        .col_end   = TAC_COL_LITTLE_MCP_END,
        .point_cnt = (TAC_ROW_LITTLE_MCP_END - TAC_ROW_LITTLE_MCP_START + 1U) *
                     (TAC_COL_LITTLE_MCP_END - TAC_COL_LITTLE_MCP_START + 1U)
    },

    /* 掌心：全列覆盖 */
    [TAC_AREA_PALM] = {
        .row_start = TAC_ROW_PALM_START,
        .row_end   = TAC_ROW_PALM_END,
        .col_start = TAC_COL_PALM_START,
        .col_end   = TAC_COL_PALM_END,
        .point_cnt = (TAC_ROW_PALM_END - TAC_ROW_PALM_START + 1U) * 
											(TAC_COL_PALM_END - TAC_COL_PALM_START + 1U)
    }
};

/**
 * @brief  节级区域 -> 手指级区域映射（同指 DIP/PIP/MCP 共用一张标定表）
 * @param  area  节级区域枚举
 * @return 手指级区域枚举；非法值返回 TAC_FINGER_MAX
 */
tac_finger_t area_to_finger(tac_area_t area)
{
    switch (area)
    {
        case TAC_AREA_THUMB_DIP:
        case TAC_AREA_THUMB_MCP:
            return TAC_FINGER_THUMB;

        case TAC_AREA_INDEX_DIP:
        case TAC_AREA_INDEX_PIP:
        case TAC_AREA_INDEX_MCP:
            return TAC_FINGER_INDEX;

        case TAC_AREA_MIDDLE_DIP:
        case TAC_AREA_MIDDLE_PIP:
        case TAC_AREA_MIDDLE_MCP:
            return TAC_FINGER_MIDDLE;

        case TAC_AREA_RING_DIP:
        case TAC_AREA_RING_PIP:
        case TAC_AREA_RING_MCP:
            return TAC_FINGER_RING;

        case TAC_AREA_LITTLE_DIP:
        case TAC_AREA_LITTLE_PIP:
        case TAC_AREA_LITTLE_MCP:
            return TAC_FINGER_LITTLE;

        case TAC_AREA_PALM:
            return TAC_FINGER_PALM;

        default:
            return TAC_FINGER_MAX;
    }
}

/**
 * @brief  取采集工作区（接口说明见 app_tac_config.h）
 */
tac_data_cfg_t* GetTacDataCfg(void)
{
		return &tac_data_cfg;
}

/**
 * @brief  取标定数据工作区（接口说明见 app_tac_config.h）
 */
tac_data_cfg_t* GetTacCalibData(void)
{
		return &tac_calib_data;
}

/**
 * @brief  取点位有效性掩码矩阵（接口说明见 app_tac_config.h）
 * @note   由 TAC_MASK_DATA_SELECT 决定返回手心侧（zs）还是手背侧（ys）掩码表
 */
const tac_data_cfg_t* GetTacMaskData(void)
{
#if (TAC_MASK_DATA_SELECT)
		return &tac_mask_data_zs;
#else
		return &tac_mask_data_ys;
#endif
}

/**
 * @brief  从指定源缓冲发布采集数据快照（写侧唯一入口，仅采集任务"轮末"调用）
 * @param  src  源成品帧（须为整轮写完的完整帧；为 NULL 时不发布，读侧仍读到上一帧）
 * @note   把源整结构拷入非活动缓冲，拷贝完成后单字节切换活动索引，读侧全程不取锁
 *         也不会看到半新半旧数据；非可重入。
 *         用于"成品帧 ≠ 采样工作区"的双缓冲结构：工作区恒为减零点 AD 增量，
 *         换算（AD->力值，默认 2g 单位）结果写在独立成品帧里，发布时以成品帧为源。
 *         详见 app_tac_config.h
 */
void TacConfig_PublishSnapshotFrom(const tac_data_cfg_t *src)
{
    uint8_t u8NextIdx;

    if (NULL == src)
    {
        return;   /* 防御：无有效源时不切换活动索引，读侧仍读到上一帧完整数据 */
    }

    u8NextIdx = (uint8_t)(s_u8SnapshotActiveIdx ^ 1U);

    /* tac_data_cfg_t 为 pragma pack(1) 纯字节矩阵结构，整结构 memcpy 安全 */
    (void)memcpy((void *)&s_astcSnapshot[u8NextIdx], (const void *)src,
                 sizeof(tac_data_cfg_t));
    s_u8SnapshotActiveIdx = u8NextIdx;
}

/**
 * @brief  从采样工作区发布采集数据快照（= TacConfig_PublishSnapshotFrom(工作区)）
 * @note   等价于以 GetTacDataCfg() 为源调用 TacConfig_PublishSnapshotFrom()；
 *         适用于"无独立成品帧"的场景（如未启用 AD->克重换算时）。
 *         详见 app_tac_config.h
 */
void TacConfig_PublishSnapshot(void)
{
    TacConfig_PublishSnapshotFrom((const tac_data_cfg_t *)&tac_data_cfg);
}

/**
 * @brief  取最近一次发布的采集数据只读快照（读侧免锁，任意任务可调用）
 * @return 活动快照缓冲指针（静态存储，永不返回 NULL）
 * @note   返回指针指向双缓冲之一，本次流程内用完即可；不得在中断上下文调用。
 *         详见 app_tac_config.h
 */
const tac_data_cfg_t* TacConfig_GetSnapshot(void)
{
    return &s_astcSnapshot[s_u8SnapshotActiveIdx];
}


