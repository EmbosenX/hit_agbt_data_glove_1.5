/**
 *******************************************************************************
 * @file    clock_cfg.h
 * @brief This file contains all the functions prototypes of the
 *        EV_HC32F460_LQFP100_V2 BSP driver library.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2022-10-31       CDT             Add XTAL/XTAL32 IO define
   2023-09-30       CDT             Add include file named hc32_ll_fcm.h and add declaration of BSP_XTAL32_Init()
   2024-11-08       CDT             Modify XTAL/XTAL32 pins definition
   2026-08-03       Custom          Modify XTAL to 12MHz, add XTAL value define
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
#ifndef __EV_HC32F460_LQFP100_V2_H__
#define __EV_HC32F460_LQFP100_V2_H__

/* 外部高速晶振频率 12MHz，覆盖 system_hc32f460.h 默认的 8MHz */
#define XTAL_VALUE                      (12000000UL)

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll_aos.h"
#include "hc32_ll_clk.h"
#include "hc32_ll_dma.h"
#include "hc32_ll_efm.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_fcm.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_i2c.h"
#include "hc32_ll_i2s.h"
#include "hc32_ll_interrupts.h"
#include "hc32_ll_keyscan.h"
#include "hc32_ll_pwc.h"
#include "hc32_ll_spi.h"
#include "hc32_ll_sram.h"
#include "hc32_ll_usart.h"
#include "hc32_ll_utility.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @addtogroup EV_HC32F460_LQFP100_V2
 * @{
 */

/**
 * @addtogroup EV_HC32F460_LQFP100_V2_BASE
 * @{
 */

#if (BSP_EV_HC32F460_LQFP100_V2 == BSP_EV_HC32F4XX)

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/**
 * @defgroup EV_HC32F460_LQFP100_V2_XTAL_CONFIG EV_HC32F460_LQFP100_V2 XTAL port/pin definition
 * @{
 */
#define BSP_XTAL_PORT                   (GPIO_PORT_H)
#define BSP_XTAL_PIN                    (GPIO_PIN_00 | GPIO_PIN_01)
#define BSP_XTAL_VALUE                  (12000000UL)   /* 外部高速晶振频率：12MHz */
/**
 * @}
 */

/**
 * @defgroup EV_HC32F460_LQFP100_V2_XTAL32_CONFIG EV_HC32F460_LQFP100_V2 XTAL32 port/pin definition
 * @{
 */
#define BSP_XTAL32_PORT                 (GPIO_PORT_C)
#define BSP_XTAL32_PIN                  (GPIO_PIN_14 | GPIO_PIN_15)
/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
/**
 * @addtogroup EV_HC32F460_LQFP100_V2_Global_Functions
 * @{
 */
int32_t BSP_XTAL32_Init(void);
void BSP_CLK_Init(void);
void BSP_RC_CLK_Init(void);

/**
 * @}
 */

#endif /* BSP_EV_HC32F460_LQFP100_V2 */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __EV_HC32F460_LQFP100_V2_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
 

