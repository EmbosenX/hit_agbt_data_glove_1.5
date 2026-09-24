/**
 *******************************************************************************
 * @file  usb/usb_dev_cdc/source/main.c
 * @brief Example CDC device.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2022-10-31       CDT             Add USB core ID select function
 @endverbatim
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
 * Include files
 ******************************************************************************/
#include "main.h"
#include "clock_cfg.h"
#include "app_finger_task.h"
#include "app_iap_task.h"
#include "app_sample_task.h"

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

/* ==================== FreeRTOS 系统任务（静态分配必需） ==================== */
static StaticTask_t xIdleTaskTcb;
static StackType_t  uxIdleTaskStack[128];

static StaticTask_t xTimerTaskTcb;
static StackType_t  uxTimerTaskStack[256];
/*******************************************************************************
 * 全局共享变量
 ******************************************************************************/

/*******************************************************************************
 * FreeRTOS 静态分配系统回调（必需，否则链接报错）
 ******************************************************************************/

/**
 * @brief  为空闲任务提供静态内存
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTcb;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize   = sizeof(uxIdleTaskStack) / sizeof(StackType_t);
}

/**
 * @brief  为定时器服务任务提供静态内存
 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &xTimerTaskTcb;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize   = sizeof(uxTimerTaskStack) / sizeof(StackType_t);
}

/**
 * @brief  main function
 * @param  None
 * @retval None
 */
int32_t main(void)
{
    /* Unlock peripherals or registers */
    LL_PERIPH_WE(EXAMPLE_PERIPH_WE);
    BSP_RC_CLK_Init();
		/* 释放PB3(PY10/Y10) JTDO，PB4(PY11/Y11) NJTRST，PA15(Y6) JTDI，保留SWD(PA13 PA14)可用 */
		GPIO_SetDebugPort(GPIO_PIN_TDO, DISABLE);   /* PB3  */
		GPIO_SetDebugPort(GPIO_PIN_TRST, DISABLE);  /* PB4  */
		GPIO_SetDebugPort(GPIO_PIN_TDI, DISABLE);   /* PA15  */
		__set_PRIMASK(1);  	
		/* 创建业务任务：全部静态分配 */
    AppFingerTask_Init();
    AppIapTask_Init();
    AppSampleTask_Init();
		/* 启动调度，开始执行任务 */
		vTaskStartScheduler();
    for (;;)
		{
			
    }
}


/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
