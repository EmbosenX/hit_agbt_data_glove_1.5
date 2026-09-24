/*******************************************************************************
 * @file        app_skid_detect.h
 * @brief       触觉滑移检测模块头文件
 * @author      
 * @date        2026-09-02
 * @version     V2.0
 * @note        【当前停用】底层已不再调用 Skid_Update（app_sample_task.c 中的每帧驱动
 *              已删除），官方协议 0x14 只返回"无滑移数据"空帧；本模块与头文件保留
 *              以便日后恢复。恢复方法：在采样任务轮末重新调用 Skid_Update（并补回
 *              #include "app_skid_detect.h"）。
 *              原语义：滑移事件后台持续检测（采样任务每帧调用 Skid_Update），
 *              官方协议(RTO 中断)经 Skid_GetSnapshotIsr、任务上下文经
 *              Skid_GetSnapshot 读取快照组帧应答；V2.0 起每区域最多
 *              SKID_TRACKER_PER_AREA 个独立部位并行跟踪（连通域聚类+多跟踪器）。
 ******************************************************************************/
#ifndef __APP_SKID_DETECT_H__
#define __APP_SKID_DETECT_H__

#include <stdint.h>
#include "app_tac_config.h"

/* 每区域最多同时跟踪的独立滑移部位数（掌心同时 3~4 处，统一按 4 支持全区域多点） */
#define SKID_TRACKER_PER_AREA   4U

/* 每手指最大事件快照槽数 = 最大节数(3) × SKID_TRACKER_PER_AREA：
 * 拇指 2 节=8 槽、食指/中指/无名指/小指 3 节=12 槽、掌心 1 节=4 槽 */
#define SKID_EVENT_MAX          (3U * SKID_TRACKER_PER_AREA)

/* 滑移事件快照 */
typedef struct {
    uint8_t  phalanx;      /* 节位：1=远节 2=中节 3=近节（掌心=1），0=无事件 */
    uint16_t matrix_no;    /* 滑移起点矩阵编号（协议附录5 全局编号 1~750，0=无事件） */
    uint16_t cur_no;       /* 滑移中当前质心矩阵编号（协议附录5 全局编号 1~750，
                            * 0=未滑移/无效；滑移期间每帧刷新、事件结束保留末点，
                            * 供上位机轮询采样连成移动轨迹） */
    uint16_t angle_x100;   /* 滑移方向 0~36000（0.01°） */
    uint16_t len_x100;     /* 单次事件累积滑移总长（0.01 step） */
    uint16_t speed_x100;   /* 事件平均滑移速度（0.01 step/采样帧，单位约定待厂商确认） */
} skid_snapshot_t;

/**
 * @brief  滑移检测每帧更新（采样任务采集完成后调用）
 * @note   每区域最多 SKID_TRACKER_PER_AREA 个独立部位并行跟踪：
 *         区域内接触点先做连通域聚类，各簇独立完成滑移状态机；
 *         单点接触时为 1 簇，行为与单部位跟踪一致。
 * @param  p 指向 GetTacDataCfg() 返回的当前帧数据，NULL 时忽略
 */
void Skid_Update(const tac_data_cfg_t *p);

/**
 * @brief  读取某手指全部事件槽快照（任务上下文版）
 * @param  sub_id   手指 sub_id（0x01~0x06，对应拇指~掌心）
 * @param  out      快照输出数组，不可为 NULL
 * @param  max_cnt  输出数组容量（建议 SKID_EVENT_MAX）
 * @retval 拷贝的快照个数（=该手指事件槽数，无事件槽输出全 0）
 * @note   槽位顺序 = 节位基址(拇指 DIP/MCP=0/4，其余手指 DIP/PIP/MCP=0/4/8，
 *         掌心=0) + 节内部位序号；仅 matrix_no 非 0 的槽表示有事件
 * @note   任务上下文版（临界采用任务级宏）；官方协议 RTO 中断应答请调用
 *         Skid_GetSnapshotIsr
 */
uint8_t Skid_GetSnapshot(uint8_t sub_id, skid_snapshot_t *out, uint8_t max_cnt);

/**
 * @brief  读取某手指全部事件槽快照（ISR 上下文版，官方协议 RTO 中断应答用）
 * @param  sub_id   手指 sub_id（0x01~0x06，对应拇指~掌心）
 * @param  out      快照输出数组，不可为 NULL
 * @param  max_cnt  输出数组容量（建议 SKID_EVENT_MAX）
 * @retval 拷贝的快照个数（=该手指事件槽数，无事件槽输出全 0）
 * @note   与 Skid_GetSnapshot 语义一致，内部临界采用 ISR 安全版宏；
 *         官方 0xAA 协议在接收超时(RTO)中断内直接应答时须调用本接口
 */
uint8_t Skid_GetSnapshotIsr(uint8_t sub_id, skid_snapshot_t *out, uint8_t max_cnt);

#endif /* __APP_SKID_DETECT_H__ */
