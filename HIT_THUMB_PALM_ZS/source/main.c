/**
 *******************************************************************************
 * @file    main.c
 * @brief   掌部（TB_PM 板）固件主入口：中断向量表配置、外设/标定初始化、
 *          静态任务创建与调度启动
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */


/*******************************************************************************
 * 系统时钟配置表（当前初始化路径 BSP_RC_CLK_Init：内部 HRC -> MPLL = 200MHz）
 *******************************************************************************
 * 时钟源（本路径全部内部，不启用 XTAL/XTAL32/UPLL）:
 *   HRC    = 16MHz      (内部高速RC, MPLL 输入源, 见 BSP_RC_CLK_Init)
 *   MPLL   = 200MHz     (16/2*50/2, 系统时钟源)
 *
 * 总线时钟 (SYSCLK = 200MHz):
 *   HCLK   = 200MHz  (/1)  内核/SRAM/Flash/DMA/AOS
 *   EXCLK  = 100MHz  (/2)  外部时钟输出
 *   PCLK0  = 200MHz  (/1)  SysTick
 *   PCLK1  = 100MHz  (/2)  USART/SPI/I2C/I2S/USBFS
 *   PCLK2  = 50MHz   (/4)  ADC/CMP/OPA
 *   PCLK3  = 50MHz   (/4)  保留
 *   PCLK4  = 100MHz  (/2)  TMR0/TMR4/TMR6/KEYSCAN
 *
 * 外设时钟:
 *   外设        实例        FCG   总线时钟    计时/参考时钟    用途
 *   ----------  ----------  ----  ----------  ---------------  --------------------
 *   USART3      CM_USART3   FCG1  PCLK1=100M  -                PE3-RX/PE4-TX 普通串口 3Mbps
 *   SPI3        CM_SPI3     FCG1  PCLK1=100M  -                手指触觉传感器
 *   ADC1        CM_ADC1     FCG3  PCLK2=50M   PCLK2            PALM/THUMB触觉+电池电压
 *   TMR0_2      CM_TMR0_2   FCG2  PCLK4=100M  LRC/8≈4096Hz     USART3 接收超时定时(判帧)
 *   DMA         -           FCG0  HCLK=200M   -                SPI/USART数据搬运
 *   AOS         -           FCG0  HCLK=200M   -                事件联动触发DMA
 *
 * 说明:
 *   1. ADC时钟源由 CLK_SetPeriClockSrc(CLK_PERIPHCLK_PCLK) 选择 PCLK2
 *   2. TMR0_2 计数时钟源为内部 LRC(≈32.8kHz), 8分频≈4096Hz, 总线时钟为 PCLK4
 *   3. main() 调用 BSP_RC_CLK_Init()（HRC 路径）；clock_cfg.c 另提供走 12MHz 晶振的
 *      BSP_CLK_Init()，当前未调用；XTAL32/UPLL 未启用。
 *      USB/USART1 等未使用外设的 FCG 时钟未开启。
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"
#include "clock_cfg.h"
#include "app_run_mode.h"
#include "app_fingers_task.h"
#include "app_sample_task.h"
#include "app_usart_task.h"
#include "app_usart_tx_task.h"
#include "app_period_timer.h"
#include "app_force_calib.h"
#include "hc_flash.h"
#include "spi_fingers.h"
#include "exit.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
/**
 * @addtogroup HC32F460_DDL_Applications
 * @{
 */
/* unlock/lock peripheral */
#define EXAMPLE_PERIPH_WE               (LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | \
                                         LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM)
#define EXAMPLE_PERIPH_WP               (LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_SRAM)

/*******************************************************************************
 * 中断向量表基址（唯一来源：链接结果，而非代码里的常量）
 *******************************************************************************
 * Image$$ER_IROM1$$Base 由 armlink 依据 sct 的 ER_IROM1 基址生成：
 *   target app      -> 0x00008000（配合 HIT_IAP_BOOT_UART bootloader）
 *   target app_boot -> 0x00000000（不带 bootloader 的直跑固件）
 * 这样地址只有 sct 一处来源，杜绝"代码写死地址与 sct 不一致"导致中断跑飞。
 ******************************************************************************/
extern uint32_t Image$$ER_IROM1$$Base;

/*******************************************************************************
 * FreeRTOS 静态分配系统回调（必需）
 ******************************************************************************/
/* 空闲任务：静态控制块 + 任务栈 */
static StaticTask_t xIdleTaskTCB;
static StackType_t  uxIdleTaskStack[128];  /* 空闲任务栈很小，128Word足够 */
/* 定时器服务任务：静态控制块 + 任务栈
 * 栈大小(512 Word = 2KB)：周期节拍(125Hz 周期上报 + caliber 10ms 上传)的回调在
 * 守护任务上下文执行，调用链含全手 750 点收集 + 组帧 + 整帧拷贝入队，实测栈深
 * 1.2~1.3KB，1KB 会溢出（与 FreeRTOSConfig.h 的 configTIMER_TASK_STACK_DEPTH
 * = configMINIMAL_STACK_SIZE*4 保持一致） */
static StaticTask_t xTimerTaskTCB;
static StackType_t  uxTimerTaskStack[512]; /* 定时器任务栈，可按需调整 */
/**
 * @brief  为空闲任务提供静态内存（FreeRTOS 内核回调，无需手动调用）
 * @param  ppxIdleTaskTCBBuffer    输出：任务控制块地址
 * @param  ppxIdleTaskStackBuffer  输出：任务栈地址
 * @param  pulIdleTaskStackSize    输出：任务栈深度（单位为 StackType_t，非字节）
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize   = sizeof(uxIdleTaskStack) / sizeof(StackType_t);
}
/**
 * @brief  为定时器服务任务提供静态内存（FreeRTOS 内核回调，无需手动调用）
 * @param  ppxTimerTaskTCBBuffer    输出：任务控制块地址
 * @param  ppxTimerTaskStackBuffer  输出：任务栈地址
 * @param  pulTimerTaskStackSize    输出：任务栈深度（单位为 StackType_t，非字节）
 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize   = sizeof(uxTimerTaskStack) / sizeof(StackType_t);
}

/*******************************************************************************
 * @brief  主函数：上电初始化 + 静态任务创建 + 启动调度器
 * @note   调度器启动前完成：向量表基址设置、时钟/调试口配置、Flash 与标定表
 *         初始化，以及全部任务的静态创建（任务与队列的创建顺序有依赖，见调用处注释）
 ******************************************************************************/
int32_t main(void)
{
    uint32_t u32VectorBase;

    /* Unlock peripherals or registers */
    LL_PERIPH_WE(EXAMPLE_PERIPH_WE);

    /* 中断向量表基址 = 本 target 的链接基址，由 sct 的 ER_IROM1 决定：
         target app      -> 0x00008000（32KB boot 之后，配合 bootloader 使用）
         target app_boot -> 0x00000000（不带 bootloader 的直跑固件）
       保证上电/软复位/MDK 直接调试 App 时中断向量均正确 */
    u32VectorBase = (uint32_t)&Image$$ER_IROM1$$Base;
#if defined (__DEBUG)
    /* Debug 自检：target 的模式宏（APP_RUN_IN_BOOT_MODE）与 sct 链接基址是否一致，
       不一致说明 target 与 sct 配错了（此时中断会跑飞），停机便于定位 */
    if (u32VectorBase != APP_EXPECTED_BASE) {
        for (;;) {
        }
    }
#endif
    SCB->VTOR = u32VectorBase;

    BSP_RC_CLK_Init();//选择内部时钟
	
		/* 释放PB3(PY10/Y10) JTDO，PB4(PY11/Y11) NJTRST，PA15(Y6) JTDI，保留SWD(PA13 PA14)可用 */
		GPIO_SetDebugPort(GPIO_PIN_TDO, DISABLE);   /* PB3  */
		GPIO_SetDebugPort(GPIO_PIN_TRST, DISABLE);  /* PB4  */
		GPIO_SetDebugPort(GPIO_PIN_TDI, DISABLE);   /* PA15  */
		__set_PRIMASK(1);  	
		/************************ 创建静态任务 ************************/
		HcFlash_Init();         /* Flash 驱动初始化（标定区读写依赖，须先于标定加载） */
		AppForceCalib_Init();   /* 从 Flash 加载 6 部位标定表（未标定部位按 AD 增量输出） */
#if (APP_FORCE_CALIB_FLASH_TEST_EN != 0U)
		AppForceCalib_FlashSelfTest();  /* 上电自检：回读 Flash 标定表并打印（存/取链路验证） */
#endif
		APP_Fingers_TaskCreate();
		APP_Sample_TaskCreate();
		APP_UsartTx_TaskCreate();   /* 发送任务与发送队列须先于 usart 任务创建 */
		APP_PeriodTimer_Init();     /* 周期节拍定时器须先于 usart 任务创建（首轮启动节拍） */
		APP_Usart_TaskCreate();
		/* 启动调度，开始执行任务 */
		vTaskStartScheduler();
    for (;;) {
    }
}
