/*******************************************************************************
 * @file    app_force_calib.h
 * @brief   力值标定核心 + HcFlash 掉电持久化 + 部位/区域映射适配层
 * @date    2026-09-16
 *
 * 协议模型（见 temp_file/上传与标定协议使用说明.md）：
 *  - 部位 part_id 1~6：拇指/食指/中指/无名指/小指/掌心，对应 descriptor 0x11~0x61；
 *  - 每部位最多 3 个标定区域（**实际分区按物理节段**：拇指 2 区 DIP/MCP、
 *    食指/中指/无名指/小指各 3 区 DIP/PIP/MCP、掌心 1 区），单区域 LUT 为
 *    rows=2（AD/力值）、2~62 点；上位机须按该分布下发 region_count；
 *  - 标定后上传计算：按区域求 AD 总和 -> LUT 三段线性（低端自 (0,0) 过渡/高端末段外推/中间插值，克重 g）-> 截断除以
 *    APP_FORCE_CALIB_OUT_UNIT_DIV（默认 2 -> 对外输出以 2g 为单位）
 *    -> 按各点 AD 占比分摊 -> 单点限幅 0~255（不做保底、不回补差额，区域总和可小于插值力值）；
 *  - 标定表不完整（6 部位区域数未全部达标）时 6 个部位都不换算，保持「减零点后的
 *    AD 增量」输出，供上位机采集标定数据；判定见 AppForceCalib_IsCalibComplete。
 *
 * 持久化布局（容量/槽位总览；逐字节字段表见 app_force_calib.c 的 fc_flash_blob_t 定义处）：
 *  - 标定区 = HC_FLASH_CALIB_ADDR(0x0007E000) 起 HC_FLASH_CALIB_SIZE(0x2000 = 8KB)
 *    单扇区，本模块独占，与 boot 代码/IAP 标志扇区不重叠；
 *  - 每部位一个 1KB 槽：槽首 = 0x0007E000 + (part-1) * FC_FLASH_PART_SLOT(1024)，
 *    part 取 1~6，6 槽共 6KB <= 8KB 标定区；
 *  - 槽内 blob 764B（fc_flash_blob_t：uint32 magic + uint8 region_count + 3 个区域
 *    的 uint16 AD/力值表），<= 1KB 槽位；尺寸关系由 fc_layout_self_check() 的
 *    编译期断言守住，改任一容量宏都会在编译期报错；
 *  - 有效性判据：magic == 'HCS2' 且 region_count 1~3 且每区域 count 2~62 且
 *    adc 严格递增、force 单调不降；空片（全 0xFF）或脏数据 -> 该部位按未标定处理；
 *  - 持久化策略：RAM 常驻 6 槽镜像 -> 整扇区擦除 -> 回写全部 6 槽 -> 逐槽回读比对；
 *  - 上电 AppForceCalib_Init() 自动加载，无效记录按未标定处理，并立即做一次
 *    完整性判定（结果在一个上电周期内固定，改标定表需复位才生效）。
 ******************************************************************************/
#ifndef __APP_FORCE_CALIB_H__
#define __APP_FORCE_CALIB_H__

#include <stdint.h>
#include <stdbool.h>
#include "app_tac_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 部位/区域/点数容量（与协议文档一致：6 部位、≤3 区域、单区域 2~62 点）。
 * 注意"容量上界"与"实际分区数"不同：运行期按物理节段分区，实际区域数为
 * 拇指 2 / 食指 3 / 中指 3 / 无名指 3 / 小指 3 / 掌心 1（见 app_force_calib.c 的
 * s_au8PartSubCnt），上位机须按该分布下发各部位的 region_count */
#define APP_FORCE_CALIB_PART_MAX     (6U)
#define APP_FORCE_CALIB_REGION_MAX   (3U)
/* 部位"有效传感点数"上界（0x02 上传 data 区长度上界，非 LUT 点数）：拇指 101 /
 * 食指 130 / 中指 137 / 无名指 130 / 小指 111 / 掌心 141，故取 141 */
#define APP_FORCE_CALIB_POINTS_MAX   (141U)
/* 单区域 LUT 点数上界（与协议 4.2 节的 2~62 点一致，须与 app_glove_proto.h 的
 * GLOVE_POINTS_MAX 相等）。**必须与上面的"有效传感点数上界"区分开**：
 * LUT/Flash 存储数组一律按本宏定容，误用 141 会让单个 Flash blob 涨到 1712B，
 * 超过 1KB 槽位并导致 6 个部位槽位首尾重叠（回读比对必失败） */
#define APP_FORCE_CALIB_LUT_POINTS_MAX (62U)

/* 力值换算应用开关：0=暂不把标定表用于力值换算（cfg 保持减零点后的 AD 增量输出）；
 * 1=按标定表 LUT 换算力值（克重表值再按输出单位宏缩放）；取值以本行的实际定义为
 * 准（该开关在调试期为常用调项，注释不再写死当前值）。
 * 置 1 后仍受**完整性子门控**约束：只有上电加载判定为"6 部位区域数全达标且
 * 每区域有效"（AppForceCalib_IsCalibComplete() == true）时才换算；否则整手仍输出
 * 减零点后的 AD 增量。
 *
 * 输出量纲（重要）：换算只作用于**发布快照**（0x02 上传与 0x30/0x10 查询应答的数据源）；
 * 采样工作区 GetTacDataCfg() 永远保存"减零点后的 AD 增量"，不会被换算结果覆盖，
 * 以免下一轮把力值当作 AD 增量二次换算（见 app_sample_task.c 的双缓冲实现）。
 * 发布快照的力值单位 = 克重 / APP_FORCE_CALIB_OUT_UNIT_DIV（默认 2g 单位），
 * 而**标定表与 PUT 下发仍是克重 g**——该非对称关系见下一节的输出单位宏。
 *
 * 重新标定提醒：完整性只在上电判定一次，故开启换算后若要重新采集标定数据
 * （以「分区合值」作为 LUT 的 AD 轴），须先把本开关置 0 或擦表后复位。 */
#ifndef APP_FORCE_CALIB_APPLY_EN
#define APP_FORCE_CALIB_APPLY_EN     (1U)
#endif

/* 对外输出力值单位倍数（标定表始终为克重 g，本宏只作用于输出换算）：
 * 1  = 输出克重（与标定表同单位）；
 * 2  = 输出 2g 单位（默认）：分辨率 2g、单点上限 255 -> 510g/点，量程 x2。
 * 换算点唯一：区域总力（LUT 插值结果，克重）在按 AD 占比分摊前截断除以本值，
 * 故 0x02 上传与官方协议 0x10 点位 / 0x12 总力均为本单位，上位机显示需 x本值；
 * 标定表（PUT 下发 / RAM 运行表 / Flash blob）仍为克重 g，旧表无需重标。
 * 注意：app_skid_detect.c 的 SKID_CONTACT_TH 等"按数据域设定"的阈值语义随之放大
 * 本倍数（该模块当前未启用）。 */
#ifndef APP_FORCE_CALIB_OUT_UNIT_DIV
#define APP_FORCE_CALIB_OUT_UNIT_DIV   (2U)
#endif
#if (APP_FORCE_CALIB_OUT_UNIT_DIV < 1U)
#error "APP_FORCE_CALIB_OUT_UNIT_DIV must be >= 1"
#endif

/* 区域 AD 总和一阶 IIR 平滑开关（抑制读数跳变）：
 * 1=开启（默认）：换算前对"决定查表力值的区域 AD 总和"做一阶低通
 *   `sm += (sum - sm) >> SHIFT`，逐点仍按瞬时占比分摊，故手型/受压分布不滞后；
 *   代价是响应延迟约 3~4 个采集节拍（SAMPLE_PERIOD_MS=10ms ⇒ ≈30~40ms）。
 * 0=关闭：完全旁路，行为与未引入平滑时一致（可用于对照验证）。
 * SHIFT：平滑系数右移位数，alpha = 1/2^SHIFT（2 ⇒ 1/4，越大越稳但越迟钝）。 */
#ifndef APP_FORCE_CALIB_SUM_SMOOTH_EN
#define APP_FORCE_CALIB_SUM_SMOOTH_EN    (1U)
#endif
#ifndef APP_FORCE_CALIB_SUM_SMOOTH_SHIFT
#define APP_FORCE_CALIB_SUM_SMOOTH_SHIFT (2U)
#endif

/* Flash 回读测试开关（上电自检打印 + PUT 写表后回读校验打印）：
 * 产物只有 DDL_Printf，故默认跟随打印通道——LL_PRINT_ENABLE=DDL_OFF 时
 * DDL_Printf 为空函数（无任何输出），置 1 只会白跑一遍回读校验，故自动置 0。
 * 需要测试时把 hc32f4xx_conf.h 的 LL_PRINT_ENABLE 改为 DDL_ON 即可，
 * 也可用编译器 -DAPP_FORCE_CALIB_FLASH_TEST_EN=... 覆盖本默认值 */
#ifndef APP_FORCE_CALIB_FLASH_TEST_EN
#if (LL_PRINT_ENABLE == DDL_ON)
#define APP_FORCE_CALIB_FLASH_TEST_EN (1U)
#else
#define APP_FORCE_CALIB_FLASH_TEST_EN (0U)
#endif
#endif

/* 部位矩形点数上界（含掩码为 0 的无效格占位，与"有效点数上限"语义不同，勿混用）：
 * AppForceCalib_LoadPartMatrix 输出整矩形，按部位物理尺寸 rows×cols 铺放，
 * 上界 = 最大矩形行数 17 × 最大列宽 16 = 272（实际各部位 196~240，均 ≤ 272）。
 * 注意：0x02 上传帧 data 区只发有效点 N（≤ 141），不使用本上界。
 * 与 app_force_calib.c 的 FC_PART_RECT_POINTS_MAX 同义，由 fc_layout_self_check
 * 编译期断言保持一致。 */
#define APP_FORCE_CALIB_RECT_POINTS_MAX (272U)

/* 单区域 LUT 表（标定表下发 PUT 解析产物，表数据在 set_part 调用期间有效） */
typedef struct
{
    const uint16_t *adc;     /* AD 轴（区域内点值之和，严格递增） */
    const uint16_t *force;   /* 力值轴（单调不降）               */
    uint16_t        count;   /* 点数 2~62                        */
} app_force_calib_table_t;

/**
 * @brief  初始化标定核心并从 Flash 加载全部 6 个部位（无效记录按未标定处理）
 * @note   须在 HcFlash_Init() 之后调用；可在调度器启动前调用（只读 Flash）。
 *         加载完成后**立即做一次标定表完整性判定**（结果供
 *         AppForceCalib_IsCalibComplete 查询），并打印一行判定结果
 *         （LL_PRINT_ENABLE=DDL_OFF 时为空函数、无输出）。
 */
void AppForceCalib_Init(void);

/**
 * @brief  查询部位标定状态（供 0x07 GET 状态查询）
 * @param  part          部位 1~6
 * @param  region_count  输出：Flash 中该部位已存储的区域数（按物理节段下发时应等于
 *                       该部位节段数：拇指 2 / 四指 3 / 掌心 1）
 * @param  mask          输出：区域有效位掩码，bit(r-1)=第 r 个节段区域已标定（≥2 点）
 * @retval true 参数有效；false 部位非法
 */
bool AppForceCalib_GetStatus(uint8_t part, uint8_t *region_count, uint16_t *mask);

/**
 * @brief  查询标定表是否"完整"（上电加载时判定一次，一个上电周期内固定）
 * @retval true  6 个部位的已存储区域数均等于各自物理节段数（拇指 2 / 四指 3 /
 *               掌心 1）且每个区域都有效（区域 LUT 点数 ≥ 2）
 * @retval false 任一部位区域数不足或存在无效区域 -> 不做 AD→克重换算
 * @note   判定仅在上电 AppForceCalib_Init() 内进行一次：PUT 下发/擦除标定表后
 *         **不会**重判，需复位（重新上电）才生效。
 *         语义为"全有全无"：false 时 6 个部位都输出减零点后的 AD 增量。
 *         调用上下文：任意任务（只读静态标志，无锁、无 Flash 访问）。
 */
bool AppForceCalib_IsCalibComplete(void);

/**
 * @brief  清空"区域 AD 总和"平滑状态（APP_FORCE_CALIB_SUM_SMOOTH_EN=0 时为空实现）
 * @note   平滑状态会跨轮累计，故在**零点基准重做之后**必须清零，避免把旧基准时期的
 *         总和当成历史值继续平滑；`AppForceCalib_Init()` 内已自动调用一次。
 *         建议调用点：采样任务重新零点快照成功（s_u8CalibReady 置 1）之后。
 *         调用上下文：任意任务（只写本模块静态状态，单实例串行调用即可）。
 */
void AppForceCalib_ResetSmoothState(void);

/**
 * @brief  整体替换指定部位全部区域 LUT（AD 严格递增、力值不降、每区域 ≥2 点），
 *         先写 Flash（失败保留旧表）再发布到运行表
 * @param  part         部位 1~6
 * @param  tables       区域 LUT 数组，按区域序号排列，长度 >= region_count
 * @param  region_count 区域数 1~3
 * @retval true  已落盘并发布（Flash 回读校验通过）
 * @retval false 参数/数值不合法，或 Flash 擦写校验失败（旧表保持不变）
 * @note   会整扇区擦除并回写全部 6 个部位槽位，耗时数十 ms；
 *         调用上下文：usart 接收任务（单实例串行），期间须暂停 0x02 周期上传
 */
bool AppForceCalib_SetPart(uint8_t part, const app_force_calib_table_t *tables, uint8_t region_count);

/**
 * @brief  擦除指定部位全部区域 LUT（写入 region_count=0 的 'HCS2' 空记录并清除运行表），
 *         不影响其他部位
 * @param  part  部位 1~6
 * @retval true  已落盘（该槽位回读判为未标定）
 * @retval false 部位非法或 Flash 擦写校验失败
 * @note   同 AppForceCalib_SetPart，仍会重写整个 8KB 标定区
 */
bool AppForceCalib_ClearPart(uint8_t part);

/*******************************************************************************
 * Flash 存取测试接口（APP_FORCE_CALIB_FLASH_TEST_EN=1 时才会打印，判定逻辑始终可用）
 ******************************************************************************/
/**
 * @brief  回读指定部位 Flash 标定表并与 RAM 镜像/运行表逐项比对（存取链路自检）
 * @param  part            部位 1~6
 * @param  pu16FirstDiff   输出：首个不一致字节在 blob 内的偏移（可为 NULL）
 * @retval true  = Flash 回读成功、与 RAM 镜像逐字节一致、且与运行表数值一致
 * @note   判定三件事：① HcFlash_CalibRead 成功且 blob 头部合法（magic/区域数/
 *         点数）；② 回读内容与 RAM 常驻镜像 memcmp 一致（持久化链路正确）；
 *         ③ blob 与运行表逐点数值一致（加载/发布链路正确）。未标定部位
 *         （blob 无效）返回 true 且偏移置 0，视为"空槽正常"。
 */
bool AppForceCalib_ReadbackVerify(uint8_t part, uint16_t *pu16FirstDiff);

/**
 * @brief  打印指定部位标定状态摘要 + 完整 LUT 数值（数据取自 Flash 回读内容）
 * @note   摘要在同一行内：部位号/是否标定/区域数/区域掩码/每区域点数；
 *         随后逐区域逐点打印 "ad 力值"，便于与上位机下发帧逐字节核对。
 *         未标定部位只打印一行 uncalibrated。仅在 APP_FORCE_CALIB_FLASH_TEST_EN=1
 *         时产生输出（否则函数体为空）。
 */
void AppForceCalib_DumpPart(uint8_t part);

/**
 * @brief  上电自检：遍历 6 个部位做 Flash 回读校验并打印（调度器启动前调用）
 * @note   只读 Flash + 打印，不含任何 Flash 写入，安全用于 main() 早期初始化。
 */
void AppForceCalib_FlashSelfTest(void);

/**
 * @brief  对部位子矩阵执行「已标定 -> LUT 整数插值 + 按 AD 占比分摊」换算（就地写回 cfg）
 * @param  part  部位 1~6
 * @param  cfg   采集工作区（行优先子矩阵，uint8 0~255）；换算后 0~255 代表力值，
 *               单位 = 克重 / APP_FORCE_CALIB_OUT_UNIT_DIV（默认 2g 单位）
 * @note   门控：标定表不完整（AppForceCalib_IsCalibComplete()==false）或该部位
 *         无有效区域（mask==0）时直接返回，cfg 保持减零点后的 AD 增量，
 *         供上位机采集标定数据。仅在采样任务整轮写临界段内调用（单实例）。
 *         区域按物理节段划分：拇指 DIP→区域 1、MCP→区域 2；四指 DIP/PIP/MCP→
 *         区域 1/2/3；掌心→区域 1（各区域 LUT 只作用于本节段点位）。
 */
void AppForceCalib_ApplyPart(uint8_t part, tac_data_cfg_t *cfg);

/**
 * @brief  部位上传矩形行数（0x02 上传帧 rows 字段）
 * @param  part  部位 1~6
 * @retval 行数（拇指 2 / 食指 3 / 中指 3 / 无名指 3 / 小指 3 / 掌心 15）；part 非法返回 0
 */
uint8_t AppForceCalib_PartRows(uint8_t part);

/**
 * @brief  部位上传矩形列数（0x02 上传帧 cols 字段）
 * @param  part  部位 1~6
 * @retval 列数（最大 16，小指含补零列）；part 非法返回 0
 */
uint8_t AppForceCalib_PartCols(uint8_t part);

/**
 * @brief  从采集快照装配部位上传矩形矩阵（行优先 dst[y*cols+x]）
 * @param  src       采集快照/工作区（只读）
 * @param  part      部位 1~6
 * @param  dst       输出缓冲，容量须 ≥ rows×cols（最大部位矩形点数
 *                   APP_FORCE_CALIB_RECT_POINTS_MAX）
 * @note   子区域纵向拼接；小指 DIP 第 10 列等补零位置保持 0。
 *         本接口输出"整矩形含无效点占位"形态，0x02 上传帧已改用
 *         AppForceCalib_LoadPartStream（有效点按编号连续）。
 */
void AppForceCalib_LoadPartMatrix(const tac_data_cfg_t *src, uint8_t part, uint8_t *dst);

/**
 * @brief  按传感点编号升序装配部位上传数据流（0x02 上传帧 data 区）
 * @param  src       采集快照/工作区（只读）
 * @param  part      部位 1~6
 * @param  dst       输出缓冲：dst[i] = 该部位编号 (i+1) 的传感点值，仅前 N 字节有效
 * @param  size      dst 容量（须 ≥ 该部位有效点数，最大 141）
 * @retval 有效点数 N（拇指 101 / 食指 130 / 中指 137 / 无名指 130 / 小指 111 /
 *         掌心 141）；参数非法、掩码不可用或容量不足返回 0
 * @note   data 段发送结构：子矩阵顺序 dip→pip→mcp（拇指 dip→mcp、掌心单矩阵），
 *         段内按 tac_mask_data_zs 的局部编号升序连续排列（如 thumb_dip 编号
 *         1..59 后接 thumb_mcp 编号 1..42）；掩码为 0 的无效点不占位，故 N 个
 *         字节与 mapping 编号一一对应，data 区只发这 N 字节（不补占位）。
 *         掩码为 const 只读表，读侧免锁，可在定时器回调上下文调用。
 */
uint16_t AppForceCalib_LoadPartStream(const tac_data_cfg_t *src, uint8_t part,
                                      uint8_t *dst, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FORCE_CALIB_H__ */
