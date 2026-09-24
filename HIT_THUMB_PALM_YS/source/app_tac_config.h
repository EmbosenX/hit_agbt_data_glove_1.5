/*******************************************************************************
 * @file    app_tac_config.h
 * @brief   触觉采集阵列配置：行列分段宏、tac_data_cfg_t 矩阵布局、点编号掩码表、
 *          采集数据发布快照接口
 * @note    各子矩阵成员按 TAC_ROW_*_START/END 与 TAC_COL_*_START/END 宏声明，
 *          与扫描驱动共用同一批宏，改动须两侧同步。
 ******************************************************************************/
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

/* 标定功能开关：1=开启(整手零点快照 + 按部位标定表 LUT 换算后输出,默认)；
   0=关闭(零点快照与 LUT 换算失效，cfg 保持原始 raw 直接输出，重新零点命令失效) */
#ifndef TAC_CALIB_EN
#define TAC_CALIB_EN  1U
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
#define TAC_ROW_INDEX_PIP_START (0U)		/* 食指中节指节 - 起始行 */
#define TAC_ROW_INDEX_PIP_END   (2U)		/* 食指中节指节 - 结束行 */
#define TAC_ROW_INDEX_MCP_START (0U)		/* 食指近节指节 - 起始行 */
#define TAC_ROW_INDEX_MCP_END   (2U)		/* 食指近节指节 - 结束行 */

#define TAC_ROW_MIDDLE_DIP_START (0U)		/* 中指远节指节 - 起始行 */
#define TAC_ROW_MIDDLE_DIP_END   (10U)		/* 中指远节指节 - 结束行 */
#define TAC_ROW_MIDDLE_PIP_START (0U)		/* 中指中节指节 - 起始行 */
#define TAC_ROW_MIDDLE_PIP_END   (2U)		/* 中指中节指节 - 结束行 */
#define TAC_ROW_MIDDLE_MCP_START (0U)		/* 中指近节指节 - 起始行 */
#define TAC_ROW_MIDDLE_MCP_END   (2U)		/* 中指近节指节 - 结束行 */

#define TAC_ROW_RING_DIP_START (0U)		/* 无名指远节指节 - 起始行 */
#define TAC_ROW_RING_DIP_END   (10U)		/* 无名指远节指节 - 结束行 */
#define TAC_ROW_RING_PIP_START (0U)		/* 无名指中节指节 - 起始行 */
#define TAC_ROW_RING_PIP_END   (2U)		/* 无名指中节指节 - 结束行 */
#define TAC_ROW_RING_MCP_START (0U)		/* 无名指近节指节 - 起始行 */
#define TAC_ROW_RING_MCP_END   (2U)		/* 无名指近节指节 - 结束行 */

#define TAC_ROW_LITTLE_DIP_START (0U)		/* 小指远节指节 - 起始行 */
#define TAC_ROW_LITTLE_DIP_END   (10U)		/* 小指远节指节 - 结束行 */
#define TAC_ROW_LITTLE_PIP_START (0U)		/* 小指中节指节 - 起始行 */
#define TAC_ROW_LITTLE_PIP_END   (2U)		/* 小指中节指节 - 结束行 */
#define TAC_ROW_LITTLE_MCP_START (0U)		/* 小指近节指节 - 起始行 */
#define TAC_ROW_LITTLE_MCP_END   (2U)		/* 小指近节指节 - 结束行 */

#define TAC_ROW_PALM_START    	(0U)		/* 掌心 - 起始行 */
#define TAC_ROW_PALM_END      	(14U)		/* 掌心 - 结束行 */

/* 列号分段定义 */
#define TAC_COL_THUMB_DIP_START     (16U)
#define TAC_COL_THUMB_DIP_END       (29U)
#define TAC_COL_THUMB_MCP_START     (16U)
#define TAC_COL_THUMB_MCP_END       (29U)

#define TAC_COL_INDEX_DIP_START     (0U)
#define TAC_COL_INDEX_DIP_END       (11U)
#define TAC_COL_INDEX_PIP_START     (0U)
#define TAC_COL_INDEX_PIP_END       (11U)
#define TAC_COL_INDEX_MCP_START     (0U)
#define TAC_COL_INDEX_MCP_END       (11U)

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

/* 列宽以 MIDDLE_LITTLE 从机为准：DIP 10 列、PIP/MCP 11 列（与 tac_mask_data 小指行宽一致） */
#define TAC_COL_LITTLE_DIP_START     (0U)
#define TAC_COL_LITTLE_DIP_END       (9U)
#define TAC_COL_LITTLE_PIP_START     (0U)
#define TAC_COL_LITTLE_PIP_END       (10U)
#define TAC_COL_LITTLE_MCP_START     (0U)
#define TAC_COL_LITTLE_MCP_END       (10U)

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

/* 三个矩阵访问接口：返回的都是静态 tac_data_cfg_t，布局同构，故部位映射函数通用 */
/**
 * @brief  取采集工作区（采样任务逐轮写入的实时矩阵）
 * @retval 静态工作区指针，永不返回 NULL
 * @note   写侧须处于 m_hSampleDataMutex 临界段内；若要跨任务读取，请改用
 *         TacConfig_GetSnapshot()，避免读到半更新数据
 */
tac_data_cfg_t* GetTacDataCfg(void);

/**
 * @brief  取标定数据工作区（供上位机采集标定原始值的矩阵）
 * @retval 静态工作区指针，永不返回 NULL
 */
tac_data_cfg_t* GetTacCalibData(void);

/**
 * @brief  取点位有效性掩码矩阵
 * @retval 只读掩码矩阵指针，永不返回 NULL
 * @note   内容为 const 只读表，读侧免锁，可在定时器回调等上下文调用；
 *         格值为 0 表示无效点，非 0 为该子矩阵内的点编号（从 1 起）
 */
const tac_data_cfg_t* GetTacMaskData(void);

/**
 * @brief  节级区域 -> 手指级区域映射（同指 DIP/PIP/MCP 共用一张标定表）
 * @param  area  节级区域枚举
 * @retval 手指级区域枚举；非法值返回 TAC_FINGER_MAX
 */
tac_finger_t area_to_finger(tac_area_t area);

/**
 * @brief  发布采集数据快照（写侧唯一入口，仅采集任务"轮末"调用）
 * @note   调用点须处于 m_hSampleDataMutex 临界段内且工作区 cfg 已整轮写完，
 *         发布出去的一定是完整成品帧。内部把工作区整结构拷贝到非活动缓冲后
 *         切换活动索引，读侧在此期间仍读旧缓冲，不会看到半新半旧状态。
 *         本函数非可重入，仅采集任务单实例调用。
 */
void TacConfig_PublishSnapshot(void);

/**
 * @brief  从指定源缓冲发布采集数据快照（支持"成品帧 ≠ 采样工作区"）
 * @param  src  源成品帧：须为整轮写完的完整一致帧；为 NULL 时不发布
 *              （活动索引不变，读侧仍读到上一帧）
 * @note   典型用途：采样工作区 `tac_data_cfg` 恒保存"减零点后的 AD 增量"，
 *         而力值换算结果（默认 2g 单位，见标定核心输出单位宏）另存独立成品帧缓冲，发布时以该缓冲为源，
 *         从而避免换算结果被写回工作区造成下一轮二次换算（量纲污染）。
 *         调用点须处于整轮写临界段内；内部整结构 memcpy 到非活动缓冲后
 *         单字节切换活动索引，读侧免锁且不会看到半新半旧数据。
 *         本函数非可重入，仅采集任务单实例调用。
 */
void TacConfig_PublishSnapshotFrom(const tac_data_cfg_t *src);

/**
 * @brief  获取最近一次发布的采集数据只读快照（读侧免锁，任意任务可调用）
 * @return 活动快照缓冲指针（静态存储，永不返回 NULL）
 * @note   取用过程不再争抢 m_hSampleDataMutex，故官方协议应答路径不会阻塞在
 *         整轮采集写窗口上。返回指针指向的缓冲为静态双缓冲之一：其被下一次发布
 *         选为非活动缓冲、再次发布时才真正被改写，故调用方只需在"本次处理流程内"
 *         用完即可（无需额外拷贝）；不得在中断上下文调用。
 */
const tac_data_cfg_t* TacConfig_GetSnapshot(void);

#endif /* __APP_TAC_CONFIG_H__ */


