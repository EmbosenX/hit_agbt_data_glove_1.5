#ifndef __APP_TAC_CONFIG_H__
#define __APP_TAC_CONFIG_H__

#include <stdint.h>
#include "hc32_ll_gpio.h"
#include "hc32_ll_adc.h"
#include "main.h"

/******************************************************************************
 * 扫描阵列总数定义
 * 扫描模式：逐行选通通道，对应ADC采集
 ******************************************************************************/
#define TAC_ROW_NUM             (TAC_ROW_PALM_END - TAC_ROW_PALM_START + 1U)   /* 阵列总行数 */
#define TAC_COL_NUM             (TAC_COL_PALM_END - TAC_COL_PALM_START + 1U)   /* 阵列总列数 */
#define TAC_TOTAL_POINTS        (TAC_ROW_NUM * TAC_COL_NUM)

#define DATA_FRAME_BYTE_NUM		0//0:单字节；1：双字节

/* 标定功能开关：1=开启(零点校准+克重换算,默认)；0=关闭(仅零点减除,输出AD增量,3C3C标定命令失效) */
#ifndef TAC_CALIB_EN
#define TAC_CALIB_EN  0U
#endif

/******************************************************************************
 * 手型点位抽象 - 行列范围映射
 * 行分组：远节(DIP)、中节(PIP)、近节(MCP)、掌心(Palm)
 * 列分组：拇指、食指、中指、无名指、小指
 ******************************************************************************/
/* 行号分段定义 */
#define TAC_ROW_THUMB_DIP_START (16U)		/* 大拇指远节指节 - 起始行 */
#define TAC_ROW_THUMB_DIP_END   (26U)		/* 大拇指远节指节 - 结束行 */
#define TAC_ROW_THUMB_MCP_START (27U)		/* 大拇指近节指节 - 起始行 */
#define TAC_ROW_THUMB_MCP_END   (29U)		/* 大拇指近节指节 - 结束行 */

//原理图已确定
#define TAC_ROW_INDEX_DIP_START (0U)		/* 食指远节指节 - 起始行 */
#define TAC_ROW_INDEX_DIP_END   (10U)		/* 食指远节指节 - 结束行 */
#define TAC_ROW_INDEX_PIP_START (11U)		/* 食指中节指节 - 起始行 */
#define TAC_ROW_INDEX_PIP_END   (13U)		/* 食指中节指节 - 结束行 */
#define TAC_ROW_INDEX_MCP_START (14U)		/* 食指近节指节 - 起始行 */
#define TAC_ROW_INDEX_MCP_END   (16U)		/* 食指近节指节 - 结束行 */

#define TAC_ROW_MIDDLE_DIP_START (0U)		/* 中指远节指节 - 起始行 */
#define TAC_ROW_MIDDLE_DIP_END   (10U)		/* 中指远节指节 - 结束行 */
#define TAC_ROW_MIDDLE_PIP_START (11U)		/* 中指中节指节 - 起始行 */
#define TAC_ROW_MIDDLE_PIP_END   (13U)		/* 中指中节指节 - 结束行 */
#define TAC_ROW_MIDDLE_MCP_START (14U)		/* 中指近节指节 - 起始行 */
#define TAC_ROW_MIDDLE_MCP_END   (16U)		/* 中指近节指节 - 结束行 */

#define TAC_ROW_RING_DIP_START (0U)		/* 无名指远节指节 - 起始行 */
#define TAC_ROW_RING_DIP_END   (10U)		/* 无名指远节指节 - 结束行 */
#define TAC_ROW_RING_PIP_START (11U)		/* 无名指中节指节 - 起始行 */
#define TAC_ROW_RING_PIP_END   (13U)		/* 无名指中节指节 - 结束行 */
#define TAC_ROW_RING_MCP_START (14U)		/* 无名指近节指节 - 起始行 */
#define TAC_ROW_RING_MCP_END   (16U)		/* 无名指近节指节 - 结束行 */

#define TAC_ROW_LITTLE_DIP_START (0U)		/* 小指远节指节 - 起始行 */
#define TAC_ROW_LITTLE_DIP_END   (10U)		/* 小指远节指节 - 结束行 */
#define TAC_ROW_LITTLE_PIP_START (11U)		/* 小指中节指节 - 起始行 */
#define TAC_ROW_LITTLE_PIP_END   (13U)		/* 小指中节指节 - 结束行 */
#define TAC_ROW_LITTLE_MCP_START (14U)		/* 小指近节指节 - 起始行 */
#define TAC_ROW_LITTLE_MCP_END   (16U)		/* 小指近节指节 - 结束行 */

#define TAC_ROW_PALM_START    	(0U)		/* 掌心 - 起始行 */
#define TAC_ROW_PALM_END      	(14U)		/* 掌心 - 结束行 */

/* 列号分段定义 */
#define TAC_COL_THUMB_DIP_START     (16U)
#define TAC_COL_THUMB_DIP_END       (29U)
#define TAC_COL_THUMB_MCP_START     (16U)
#define TAC_COL_THUMB_MCP_END       (29U)

#define TAC_COL_INDEX_DIP_START     (12U)
#define TAC_COL_INDEX_DIP_END       (23U)
#define TAC_COL_INDEX_PIP_START     (12U)
#define TAC_COL_INDEX_PIP_END       (23U)
#define TAC_COL_INDEX_MCP_START     (12U)
#define TAC_COL_INDEX_MCP_END       (23U)

#define TAC_COL_MIDDLE_DIP_START     (0U)
#define TAC_COL_MIDDLE_DIP_END       (12U)
#define TAC_COL_MIDDLE_PIP_START     (0U)
#define TAC_COL_MIDDLE_PIP_END       (12U)
#define TAC_COL_MIDDLE_MCP_START     (0U)
#define TAC_COL_MIDDLE_MCP_END       (12U)

#define TAC_COL_RING_DIP_START     (0U)
#define TAC_COL_RING_DIP_END       (11U)
#define TAC_COL_RING_PIP_START     (0U)
#define TAC_COL_RING_PIP_END       (11U)
#define TAC_COL_RING_MCP_START     (0U)
#define TAC_COL_RING_MCP_END       (11U)

#define TAC_COL_LITTLE_DIP_START     (13U)
#define TAC_COL_LITTLE_DIP_END       (22U)
#define TAC_COL_LITTLE_PIP_START     (13U)
#define TAC_COL_LITTLE_PIP_END       (23U)
#define TAC_COL_LITTLE_MCP_START     (13U)
#define TAC_COL_LITTLE_MCP_END       (23U)

#define TAC_COL_PALM_START    	(0U)
#define TAC_COL_PALM_END      	(15U)
 
/* 触觉区域枚举 */
typedef enum {
    TAC_AREA_THUMB_DIP=0,        /* 大拇指 - 远节 */
		TAC_AREA_THUMB_MCP,        /* 大拇指 - 近节 */
	
		TAC_AREA_INDEX_DIP,        /* 食指 - 远节 */
    TAC_AREA_INDEX_PIP=3,        /* 食指 - 中节 */
    TAC_AREA_INDEX_MCP,        /* 食指 - 近节 */
	
		TAC_AREA_MIDDLE_DIP,        /* 中指 - 远节 */
    TAC_AREA_MIDDLE_PIP=6,       /* 中指 - 中节 */
    TAC_AREA_MIDDLE_MCP,       /* 中指 - 近节 */

		TAC_AREA_RING_DIP,        /* 无名指 - 远节 */
    TAC_AREA_RING_PIP=9,         /* 无名指 - 中节 */
    TAC_AREA_RING_MCP,         /* 无名指 - 近节 */
	
		TAC_AREA_LITTLE_DIP,        /* 小指 - 远节 */
    TAC_AREA_LITTLE_PIP=12,       /* 小指 - 中节 */
    TAC_AREA_LITTLE_MCP,       /* 小指 - 近节 */
	
    TAC_AREA_PALM=14,             /* 掌心 */

    TAC_AREA_MAX
} tac_area_t;

/* 手指级区域枚举（标定表粒度：6 个区域各一张表） */
typedef enum {
    TAC_FINGER_THUMB = 0,    /* 拇指（DIP+MCP） */
    TAC_FINGER_INDEX,        /* 食指（DIP+PIP+MCP） */
    TAC_FINGER_MIDDLE,       /* 中指（DIP+PIP+MCP） */
    TAC_FINGER_RING,         /* 无名指（DIP+PIP+MCP） */
    TAC_FINGER_LITTLE,       /* 小指（DIP+PIP+MCP） */
    TAC_FINGER_PALM,         /* 掌心 */
    TAC_FINGER_MAX
} tac_finger_t;

/* 标定点数量（克重序列：0,5,10,25,50,100,150,255） */
#define CALIB_POINT_NUM     8

/* 区域范围结构体 */
typedef struct {
    uint8_t row_start;     /* 区域起始行 */
    uint8_t row_end;       /* 区域结束行 */
    uint8_t col_start;     /* 区域起始列 */
    uint8_t col_end;       /* 区域结束列 */
    uint16_t point_cnt;    /* 区域内传感点总数 */
} tac_area_range_t;

#pragma pack(push, 1)   // 保存原有对齐设置，设置1字节对齐
//目标																		计算方式																说明
//单行总字节数（行大小）									sizeof(palm[0])													第 0 行是一个完整的一维数组，sizeof 返回整行的总字节数
//总行数																	sizeof(palm) / sizeof(palm[0])					数组总字节数 ÷ 单行字节数 = 行数
//总列数（每行元素数）										sizeof(palm[0]) / sizeof(palm[0][0])		单行字节数 ÷ 单个元素字节数 = 每行元素个数
//单个元素字节数													sizeof(palm[0][0])											数组单个元素的大小
typedef struct{
#if(DATA_FRAME_BYTE_NUM)
	uint16_t thumb_dip[TAC_ROW_THUMB_DIP_END - TAC_ROW_THUMB_DIP_START + 1U][TAC_COL_THUMB_DIP_END - TAC_COL_THUMB_DIP_START + 1U];//大拇指fpc型指尖
	uint16_t thumb_mcp[TAC_ROW_THUMB_MCP_END - TAC_ROW_THUMB_MCP_START + 1U][TAC_COL_THUMB_MCP_END - TAC_COL_THUMB_MCP_START + 1U];
	uint16_t index_dip[TAC_ROW_INDEX_DIP_END - TAC_ROW_INDEX_DIP_START + 1U][TAC_COL_INDEX_DIP_END - TAC_COL_INDEX_DIP_START + 1U];//食指fpc型指尖
	uint16_t index_pip[TAC_ROW_INDEX_PIP_END - TAC_ROW_INDEX_PIP_START + 1U][TAC_COL_INDEX_PIP_END - TAC_COL_INDEX_PIP_START + 1U];
	uint16_t index_mcp[TAC_ROW_INDEX_MCP_END - TAC_ROW_INDEX_MCP_START + 1U][TAC_COL_INDEX_MCP_END - TAC_COL_INDEX_MCP_START + 1U];
	uint16_t middle_dip[TAC_ROW_MIDDLE_DIP_END - TAC_ROW_MIDDLE_DIP_START + 1U][TAC_COL_MIDDLE_DIP_END - TAC_COL_MIDDLE_DIP_START + 1U];//中指fpc型指尖
	uint16_t middle_pip[TAC_ROW_MIDDLE_PIP_END - TAC_ROW_MIDDLE_PIP_START + 1U][TAC_COL_MIDDLE_PIP_END - TAC_COL_MIDDLE_PIP_START + 1U];
	uint16_t middle_mcp[TAC_ROW_MIDDLE_MCP_END - TAC_ROW_MIDDLE_MCP_START + 1U][TAC_COL_MIDDLE_MCP_END - TAC_COL_MIDDLE_MCP_START + 1U];
	uint16_t ring_dip[TAC_ROW_RING_DIP_END - TAC_ROW_RING_DIP_START + 1U][TAC_COL_RING_DIP_END - TAC_COL_RING_DIP_START + 1U];//无名指fpc型指尖
	uint16_t ring_pip[TAC_ROW_RING_PIP_END - TAC_ROW_RING_PIP_START + 1U][TAC_COL_RING_PIP_END - TAC_COL_RING_PIP_START + 1U];
	uint16_t ring_mcp[TAC_ROW_RING_MCP_END - TAC_ROW_RING_MCP_START + 1U][TAC_COL_RING_MCP_END - TAC_COL_RING_MCP_START + 1U];
	uint16_t little_dip[TAC_ROW_LITTLE_DIP_END - TAC_ROW_LITTLE_DIP_START + 1U][TAC_COL_LITTLE_DIP_END - TAC_COL_LITTLE_DIP_START + 1U];//小拇指fpc型指尖
	uint16_t little_pip[TAC_ROW_LITTLE_PIP_END - TAC_ROW_LITTLE_PIP_START + 1U][TAC_COL_LITTLE_PIP_END - TAC_COL_LITTLE_PIP_START + 1U];
	uint16_t little_mcp[TAC_ROW_LITTLE_MCP_END - TAC_ROW_LITTLE_MCP_START + 1U][TAC_COL_LITTLE_MCP_END - TAC_COL_LITTLE_MCP_START + 1U];
	
	uint16_t palm[TAC_ROW_PALM_END - TAC_ROW_PALM_START + 1U][TAC_COL_PALM_END - TAC_COL_PALM_START + 1U];
#else
	uint8_t thumb_dip[TAC_ROW_THUMB_DIP_END - TAC_ROW_THUMB_DIP_START + 1U][TAC_COL_THUMB_DIP_END - TAC_COL_THUMB_DIP_START + 1U];//大拇指fpc型指尖
	uint8_t thumb_mcp[TAC_ROW_THUMB_MCP_END - TAC_ROW_THUMB_MCP_START + 1U][TAC_COL_THUMB_MCP_END - TAC_COL_THUMB_MCP_START + 1U];
	uint8_t index_dip[TAC_ROW_INDEX_DIP_END - TAC_ROW_INDEX_DIP_START + 1U][TAC_COL_INDEX_DIP_END - TAC_COL_INDEX_DIP_START + 1U];//食指fpc型指尖
	uint8_t index_pip[TAC_ROW_INDEX_PIP_END - TAC_ROW_INDEX_PIP_START + 1U][TAC_COL_INDEX_PIP_END - TAC_COL_INDEX_PIP_START + 1U];
	uint8_t index_mcp[TAC_ROW_INDEX_MCP_END - TAC_ROW_INDEX_MCP_START + 1U][TAC_COL_INDEX_MCP_END - TAC_COL_INDEX_MCP_START + 1U];
	uint8_t middle_dip[TAC_ROW_MIDDLE_DIP_END - TAC_ROW_MIDDLE_DIP_START + 1U][TAC_COL_MIDDLE_DIP_END - TAC_COL_MIDDLE_DIP_START + 1U];//中指fpc型指尖
	uint8_t middle_pip[TAC_ROW_MIDDLE_PIP_END - TAC_ROW_MIDDLE_PIP_START + 1U][TAC_COL_MIDDLE_PIP_END - TAC_COL_MIDDLE_PIP_START + 1U];
	uint8_t middle_mcp[TAC_ROW_MIDDLE_MCP_END - TAC_ROW_MIDDLE_MCP_START + 1U][TAC_COL_MIDDLE_MCP_END - TAC_COL_MIDDLE_MCP_START + 1U];
	uint8_t ring_dip[TAC_ROW_RING_DIP_END - TAC_ROW_RING_DIP_START + 1U][TAC_COL_RING_DIP_END - TAC_COL_RING_DIP_START + 1U];//无名指fpc型指尖
	uint8_t ring_pip[TAC_ROW_RING_PIP_END - TAC_ROW_RING_PIP_START + 1U][TAC_COL_RING_PIP_END - TAC_COL_RING_PIP_START + 1U];
	uint8_t ring_mcp[TAC_ROW_RING_MCP_END - TAC_ROW_RING_MCP_START + 1U][TAC_COL_RING_MCP_END - TAC_COL_RING_MCP_START + 1U];
	uint8_t little_dip[TAC_ROW_LITTLE_DIP_END - TAC_ROW_LITTLE_DIP_START + 1U][TAC_COL_LITTLE_DIP_END - TAC_COL_LITTLE_DIP_START + 1U];//小拇指fpc型指尖
	uint8_t little_pip[TAC_ROW_LITTLE_PIP_END - TAC_ROW_LITTLE_PIP_START + 1U][TAC_COL_LITTLE_PIP_END - TAC_COL_LITTLE_PIP_START + 1U];
	uint8_t little_mcp[TAC_ROW_LITTLE_MCP_END - TAC_ROW_LITTLE_MCP_START + 1U][TAC_COL_LITTLE_MCP_END - TAC_COL_LITTLE_MCP_START + 1U];
	
	uint8_t palm[TAC_ROW_PALM_END - TAC_ROW_PALM_START + 1U][TAC_COL_PALM_END - TAC_COL_PALM_START + 1U];
#endif
}tac_data_cfg_t;
#pragma pack(pop)       // 恢复原先对齐方式

extern const tac_area_range_t TacAreaRange[TAC_AREA_MAX];
tac_data_cfg_t* GetTacDataCfg(void);
tac_data_cfg_t* GetTacCalibData(void);
const tac_data_cfg_t* GetTacMaskData(void);
extern const float calib_weight[CALIB_POINT_NUM];
extern const float calib_ad[TAC_FINGER_MAX][CALIB_POINT_NUM];
tac_finger_t area_to_finger(tac_area_t area);
uint8_t ad_to_weight(uint16_t ad_val, tac_finger_t finger);

#endif /* __APP_TAC_CONFIG_H__ */


