#ifndef __USART_BSP_H__
#define __USART_BSP_H__

#include <stdint.h>
#include "hc32_ll.h"

/******************************************************************************
 * 配置项：已适配 PE3-RX / PE4-TX / USART3
 ******************************************************************************/
/* 串口号与波特率 */
#define USART_BSP_UNIT                  (CM_USART3)
/* 运行期链路速率 3000000(8N1)：PCLK1=100MHz + USART_CLK_DIV1 + 8 倍过采样，
 * 分频系数 = 100e6/(8*3e6) = 4.1667，可由 DIV_INT=3 加小数分频精确表达，
 * 实际约 2.99~3.003Mbps、误差约 0.1%~0.3%，远小于 UART 容差；
 * 且 3Mbps 仅占 8 倍过采样理论上限 12.5Mbps 的 24%，无采样点余量风险。
 * 3Mbps 满足通讯协议「波特率 >= 2000000」下限，但要求上位机侧 USB-串口桥支持 3Mbps
 * （CH340 上限约 2Mbps、CP2102 约 1Mbps 均不支持；FT232R 上限 3Mbps、CH343 可达 6Mbps）。
 * 改此值必须同步上位机「波特率」；Boot(OTA) 波特率独立，见 app_iap.h */
#define USART_BSP_BAUDRATE              (3000000UL)
/* 收发帧缓存上限：官方协议 get_ori_data 应答拼接大包约 576B，扩至 1024B 留余量；
 * 同时为 RX 缓冲、DMA 接收计数与单帧发送长度的共用上限 */
#define USART_BSP_MAX_FRAME_LEN         (1024U)

/* RX引脚：PE3 */
#define USART_BSP_RX_PORT               (GPIO_PORT_E)
#define USART_BSP_RX_PIN                (GPIO_PIN_03)
#define USART_BSP_RX_FUNC               (GPIO_FUNC_33)   /* USART3_RX */

/* TX引脚：PE4 */
#define USART_BSP_TX_PORT               (GPIO_PORT_E)
#define USART_BSP_TX_PIN                (GPIO_PIN_04)
#define USART_BSP_TX_FUNC               (GPIO_FUNC_32)   /* USART3_TX */

/* 接收超时时长：判帧时间按 TMR0 计数时钟换算（TMR0 计数源为内部 LRC≈32.8kHz，
 * 与系统时钟源无关）：
 * LRC ÷8 ≈ 4096Hz，1 tick ≈ 244µs；compare = ceil(bits/8) - 3（DIV8 硬件延迟补偿 3，
 * 见 usart_bsp.c 中 TMR0_Config 的内部时钟公式）。
 * 当前 25U：compare=1，对应线空闲约 0.24ms 判帧结束（<1ms，适用于 MCU 直连连续发送）；
 * 调整注意：宏值必须 >=25 才可得到 compare>=1（过小会导致 compare<=0 而配置无效），
 * 实测若一帧被拆多段请上调至 48U(≈0.73ms)/56U(≈0.98ms)，若多帧被合并请下调 */
#define USART_BSP_TIMEOUT_BITS          (25U)

/* 发送方式编译期选择（二选一，改 USART_BSP_TX_MODE 即可）：
 *   DMA  ：DMA 通道搬运 + 发送完成任务通知（默认）：USART_BSP_Send 只做"忙检测并启动
 *          一帧 DMA"，返回 LL_OK 即本帧开始发送；线上发送完毕时 TX_CPLT 中断以
 *          vTaskNotifyGiveFromISR(xUsartTxTaskHandle) 唤醒 app_usart_tx_task 发送任务
 *          （发送引擎），由其串行启动下一帧。发送队列/逐帧调度由应用层
 *          app_usart_tx_task.c 的静态 FreeRTOS 队列（AppUsartTxTask_Send）负责。
 *          原 BSP 内部 TX 环形队列与中断自动续发已移除。
 *   POLL ：普通寄存器轮询直发（忙等，不依赖发送 DMA/完成中断；USART_BSP_Send 返回即
 *          线上发送完毕，适合小帧/驱动级调试）。
 * 切换为 POLL 时 usart_bsp.c 会整体裁剪发送 DMA、TX 完成中断相关代码 */
#define USART_BSP_TX_MODE_DMA           (0U)
#define USART_BSP_TX_MODE_POLL          (1U)
#define USART_BSP_TX_MODE               (USART_BSP_TX_MODE_DMA)

/******************************************************************************
 * 对外接口函数
 ******************************************************************************/
/**
 * @brief  初始化USART+DMA(RX)+超时定时器；发送链路按 USART_BSP_TX_MODE 初始化
 */
void USART_BSP_Init(void);

/**
 * @brief  发送一帧数据（底层单帧发送入口，仅 app_usart_tx_task 发送引擎调用）
 * @param  pu8Data 发送数据指针
 * @param  u16Len  数据长度（不得超过 USART_BSP_MAX_FRAME_LEN）
 * @retval LL_OK=本帧 DMA 已启动（或 POLL 模式已线上发送完毕）；
 *         LL_ERR_INVD_PARAM=参数错误/长度超限；LL_ERR_BUSY=发送忙（前一帧未发完）
 * @note   DMA 模式：忙检测(m_u8TxBusy+DMA 硬件状态) -> 清残留 -> 启动一帧 DMA 后即返回
 *         LL_OK；**返回不代表线上发完**，调用方应阻塞等待 TX_CPLT 中断的"发送完成"
 *         任务通知后再复用源缓冲/启动下一帧（逐帧串行，见 app_usart_tx_task.c 发送引擎）。
 *         POLL 模式：降级为寄存器轮询直发，返回即线上发送完毕（仅调试）。
 */
int32_t USART_BSP_Send(const uint8_t *pu8Data, uint16_t u16Len);

/**
 * @brief  [兼容保留] 异步发送一帧数据（等价转调 USART_BSP_Send，语义见其注释）
 * @note   原 BSP 内部 TX 环形队列/中断自动续发机制已移除，发送队列上移至应用层
 *         app_usart_tx_task.c。**新代码请统一走应用层 AppUsartTxTask_Send()**（见
 *         app_usart_tx_task.h）：由发送任务发送引擎逐帧调用 USART_BSP_Send 并等待
 *         发送完成通知，保证帧序与源缓冲生命周期受控。本接口保留仅为兼容旧调用。
 */
int32_t USART_BSP_SendAsync(const uint8_t *pu8Data, uint16_t u16Len);

/**
 * @brief  中断上下文轮询直发一帧数据（ISR 版发送接口，仅 POLL 调试模式可用）
 * @param  pu8Data 发送数据指针
 * @param  u16Len  数据长度
 * @retval LL_OK=本帧已完整发送完毕；LL_ERR_INVD_PARAM=参数错误（含非 POLL 模式调用）；
 *         LL_ERR_TIMEOUT=发送卡死（内部已按当前模式复位发送通道）
 * @note   业务发送已统一走应用层发送队列 AppUsartTxTask_Send()（app_usart_tx_task.h），
 *         本接口不再用于生产路径，仅在 POLL 模式做驱动级直发调试。DMA 模式编译下
 *         不可用（返回 LL_ERR_INVD_PARAM），ISR 内勿再直发。
 */
int32_t USART_BSP_SendFromISR(const uint8_t *pu8Data, uint16_t u16Len);

/**
 * @brief  强制复位发送通道（任务上下文调用）
 * @note   用于发送卡死等异常收尾（DMA：停 DMA+清中断标志；POLL：停发送+清标志）。
 *         复位后调用方必须丢弃当前帧，不得继续引用源缓冲
 */
void USART_BSP_TxAbort(void);

/* RX 接收链路说明（双缓冲）：内部维护 2×1024B 接收缓冲与最多 2 个"已收齐未消费"段的
 * 待处理队列。每次接收空闲(RTO)只把段长度记入当前槽并切换到另一块缓冲重武装，
 * 绝不在消费端取走前写回旧段。消费模型（主路径为任务侧消费）：
 *   1) 任务侧消费（HIT_THUMB_PALM 主路径）：接收超时(RTO)中断登记段后仅释放接收信号量
 *      （应用层重写的 USART_BSP_RxCpltCallback 只 xSemaphoreGiveFromISR），usart 任务
 *      take 信号量后按"IsRxFrameDone 查询→GetRxFrame 拷贝取段→ClearRxFlag 释放"逐段
 *      排空并在任务上下文完成攒帧/协议解析（ClearRxFlag 带 NVIC 屏蔽）。
 *   2) 中断自消费（历史/备用）：USART_BSP_GetRxFramePtr 零拷贝取段 + ClearRxFlagIsr
 *      释放，面向曾在 RTO 中断内直接解析的旧实现，当前主路径已不使用。 */

/**
 * @brief  查询是否有已收齐且未被取走的接收段
 * @retval 1=有待消费段，0=无
 */
uint8_t USART_BSP_IsRxFrameDone(void);

/**
 * @brief  获取最旧的待消费接收段（每段需在消费完成后调用 ClearRxFlag 释放一次）
 * @param  pu8Buf     输出缓冲区
 * @param  u16BufLen  缓冲区大小
 * @retval 实际拷贝长度；0=无待消费段或参数错误
 */
uint16_t USART_BSP_GetRxFrame(uint8_t *pu8Buf, uint16_t u16BufLen);

/**
 * @brief  释放最旧的待消费接收段（每取走一段须对应调用一次）
 * @note   若曾因双缓冲满溢出停收，释放后本函数会自动恢复接收
 */
void USART_BSP_ClearRxFlag(void);

/**
 * @brief  获取最旧的待消费接收段指针（不拷贝数据，中断上下文专用）
 * @param  ppu8Data  输出：段数据指针（指向接收缓冲内部，释放前有效）
 * @param  pu16Len   输出：该段实际接收长度
 * @retval 1=成功取得一段；0=无待消费段或参数错误
 * @note   仅在 USART_BSP_RxCpltCallback（接收超时中断上下文）内调用：回调返回前
 *         接收状态机不会变化（同级 RTO 中断不重入），可安全直接引用完成段，
 *         避免把最长 1KB 段 memcpy 到 ISR 栈上；解析完成后必须调用
 *         USART_BSP_ClearRxFlagIsr 释放该段。
 */
uint8_t USART_BSP_GetRxFramePtr(const uint8_t **ppu8Data, uint16_t *pu16Len);

/**
 * @brief  释放最旧的待消费接收段（中断上下文专用）
 * @note   与 USART_BSP_ClearRxFlag 消费语义一致，但面向"中断自消费"模型：
 *         接收状态机仅由接收超时中断自身读写（同级不重入），故省略 NVIC 屏蔽。
 *         若此前曾因双缓冲满溢出停收，释放后会自动恢复接收。
 */
void USART_BSP_ClearRxFlagIsr(void);

/******************************************************************************
 * 弱回调函数：应用层可重写，在中断上下文执行
 ******************************************************************************/
/**
 * @brief  接收完成回调（弱定义）：每收齐一个接收段调用一次，中断上下文执行
 * @param  u16Len  该接收段长度
 */
void USART_BSP_RxCpltCallback(uint16_t u16Len);


#endif /* __USART_BSP_H__ */


