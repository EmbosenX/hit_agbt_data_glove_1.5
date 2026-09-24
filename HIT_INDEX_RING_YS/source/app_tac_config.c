#include "app_tac_config.h"


/* 触觉点编号数据选择宏：
 *   0: 使用 tac_mask_data_ys（默认，手背侧）
 *   1: 使用 tac_mask_data_zs（手心侧） */
#define TAC_MASK_DATA_SELECT   (0)


static tac_data_cfg_t tac_data_cfg={0};
static tac_data_cfg_t tac_calib_data={0};


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
/************************
 * 标定数据表（6 手指级区域，每区域 8 个标定点）
 * calib_weight：固定克重序列，单位 g（全区域共用）
 * calib_ad：每区域实测 AD 值（按升序，与 calib_weight 下标一一对应）
 * 说明：默认填入理想线性占位（ad == weight），标定后替换为实测 AD 值
 ************************/
const float calib_weight[CALIB_POINT_NUM] = {
    0.0f, 5.0f, 10.0f, 25.0f, 50.0f, 100.0f, 150.0f, 255.0f
};

const float calib_ad[TAC_FINGER_MAX][CALIB_POINT_NUM] = {
    /* 拇指 */   {0.0f, 5.0f, 10.0f, 25.0f, 50.0f, 100.0f, 150.0f, 255.0f},
    /* 食指 */   {0.0f, 5.0f, 10.0f, 25.0f, 50.0f, 100.0f, 150.0f, 255.0f},
    /* 中指 */   {0.0f, 5.0f, 10.0f, 25.0f, 50.0f, 100.0f, 150.0f, 255.0f},
    /* 无名指 */ {0.0f, 5.0f, 10.0f, 25.0f, 50.0f, 100.0f, 150.0f, 255.0f},
    /* 小指 */   {0.0f, 5.0f, 10.0f, 25.0f, 50.0f, 100.0f, 150.0f, 255.0f},
    /* 掌心 */   {0.0f, 5.0f, 10.0f, 25.0f, 50.0f, 100.0f, 150.0f, 255.0f}
};

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
 * @brief  通过区域总 AD 值（零点校准后各点增量之和）查对应区域标定表插值计算总克重
 * @param  ad_val  输入的区域总 AD（各传感点增量求和，可超 255）
 * @param  finger  手指级区域
 * @return uint8_t 区域总克重（0~255g），超出标定范围时边界截断
 * @note   总和模型：标定表 calib_ad 存的是整区域总 AD（对应砝码总重量），
 *         调用方需先对区域内所有点求 AD 增量之和再传入。
 */
uint8_t ad_to_weight(uint16_t ad_val, tac_finger_t finger)
{
    if (finger >= TAC_FINGER_MAX)
    {
        return 0U;  /* 防御：非法区域返回 0 克重 */
    }

    const float *ad_table = calib_ad[finger];

    /* 边界截断：小于最小标定点 -> 0g */
    if (ad_val <= ad_table[0])
    {
        return (uint8_t)calib_weight[0];
    }

    /* 边界截断：大于最大标定点 -> 255g */
    if (ad_val >= ad_table[CALIB_POINT_NUM - 1])
    {
        return (uint8_t)calib_weight[CALIB_POINT_NUM - 1];
    }

    /* 查找所在分段（ad_val 已大于 ad_table[0]） */
    uint8_t i;
    for (i = 0; i < CALIB_POINT_NUM - 1; i++)
    {
        if (ad_val <= ad_table[i + 1])
        {
            break;
        }
    }

    /* 分段线性插值 */
    float ad_diff = ad_table[i + 1] - ad_table[i];
    float weight;
    if (ad_diff > 0.0f)
    {
        float ratio = ((float)ad_val - ad_table[i]) / ad_diff;
        weight = calib_weight[i] + ratio * (calib_weight[i + 1] - calib_weight[i]);
    }
    else
    {
        weight = calib_weight[i];  /* 防止除零 */
    }

    /* 单字节范围保护 */
    if (weight < 0.0f)
    {
        return 0;
    }
    if (weight > 255.0f)
    {
        return 255;
    }

    return (uint8_t)(weight + 0.5f);  /* 四舍五入 */
}

tac_data_cfg_t* GetTacDataCfg(void)
{
		return &tac_data_cfg;
}
tac_data_cfg_t* GetTacCalibData(void)
{
		return &tac_calib_data;
}

const tac_data_cfg_t* GetTacMaskData(void)
{
#if (TAC_MASK_DATA_SELECT)
		return &tac_mask_data_zs;
#else
		return &tac_mask_data_ys;
#endif
}


