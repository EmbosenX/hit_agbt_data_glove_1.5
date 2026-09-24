/**
 *******************************************************************************
 * @file  ev_hc32f460_lqfp100_v2.c
 * @brief This file provides firmware functions for EV_HC32F460_LQFP100_V2 BSP
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2022-10-31       CDT             Add configuration of XTAL IO as analog function
   2023-09-30       CDT             Add API BSP_XTAL32_Init()
                                    Optimize function BSP_I2C_Init()
   2024-11-08       CDT             Modify for data overflow in function BSP_I2C_Init()
                                    Enable cache in API BSP_CLK_Init()
                                    MISRAC fix in API BSP_XTAL32_Init()
                                    FCM interface add instance
   2026-08-03       Custom          Modify XTAL from 8MHz to 12MHz, adjust MPLL parameters
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
#include "clock_cfg.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @defgroup EV_HC32F460_LQFP100_V2 EV_HC32F460_LQFP100_V2
 * @{
 */

/**
 * @defgroup EV_HC32F460_LQFP100_V2_BASE EV_HC32F460_LQFP100_V2 Base
 * @{
 */

#if (BSP_EV_HC32F460_LQFP100_V2 == BSP_EV_HC32F4XX)

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/**
 * @}
 */

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @brief  BSP clock initialize.
 *         Set board system clock to MPLL@200MHz (XTAL 12MHz as source)
 *         计算公式: 12MHz / 3 * 100 / 2 = 200MHz
 * @param  None
 * @retval None
 */
__WEAKDEF void BSP_CLK_Init(void)
{
    stc_clock_xtal_init_t     stcXtalInit;
    stc_clock_pll_init_t      stcMpllInit;

    GPIO_AnalogCmd(BSP_XTAL_PORT, BSP_XTAL_PIN, ENABLE);
    (void)CLK_XtalStructInit(&stcXtalInit);
    (void)CLK_PLLStructInit(&stcMpllInit);

    /* 总线分频：与 200MHz 主频匹配（HCLK/EXCLK/PCLK0~4 见 main.c 时钟表） */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | \
                                      CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));

    /* Config Xtal and enable Xtal
       12MHz晶振适配：驱动等级调低功耗，稳定时间加长保证起振 */
    stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
    stcXtalInit.u8Drv = CLK_XTAL_DRV_LOW;       /* 12MHz推荐低驱动，起振困难可调为MID */
    stcXtalInit.u8State = CLK_XTAL_ON;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_4MS; /* 12MHz起振稍慢，加长稳定时间 */
    (void)CLK_XtalInit(&stcXtalInit);

    /* MPLL config (XTAL 12MHz / pllmDiv * plln / PllpDiv = 200M).
       PLLM = 3  → 12/3 = 4MHz 参考时钟
       PLLN = 100 → 4*100 = 400MHz VCO
       PLLP = 2  → 400/2 = 200MHz 系统时钟
       注意：VCO(400MHz) 与参考时钟(4MHz) 必须落在数据手册允许区间内 */
    stcMpllInit.PLLCFGR = 0UL;
    stcMpllInit.PLLCFGR_f.PLLM = 3UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLN = 100UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLP = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLQ = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLR = 2UL - 1UL;
    stcMpllInit.u8PLLState = CLK_PLL_ON;
    stcMpllInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
    (void)CLK_PLLInit(&stcMpllInit);
    /* Wait MPLL ready. */
    while (SET != CLK_GetStableStatus(CLK_STB_FLAG_PLL)) {
        ;
    }

    /* sram init include read/write wait cycle setting */
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    SRAM_SetWaitCycle((SRAM_SRAM12 | SRAM_SRAM3 | SRAM_SRAMR), SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);

    /* flash read wait cycle setting */
    (void)EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    /* 3 cycles for 126MHz ~ 200MHz */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);
    /* Switch driver ability */
    (void)PWC_HighSpeedToHighPerformance();
    /* Switch system clock source to MPLL. */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
    /* Reset cache ram */
    EFM_CacheRamReset(ENABLE);
    EFM_CacheRamReset(DISABLE);
    /* Enable cache */
    EFM_CacheCmd(ENABLE);
}

/**
 * @brief  BSP clock initialize 内部时钟.
 *         Set board system clock to MPLL@200MHz (Internal HRC 16MHz as source)
 * @param  None
 * @retval None
 */
__WEAKDEF void BSP_RC_CLK_Init(void)
{
    stc_clock_pll_init_t      stcMpllInit;

    (void)CLK_PLLStructInit(&stcMpllInit);

    /* 总线分频：与 200MHz 主频匹配（HCLK/EXCLK/PCLK0~4 见 main.c 时钟表） */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | \
                                      CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));

    /* 显式开启内部高速RC HRC（16MHz，默认上电已开启，此处增加可靠性） */
    CLK_HrcCmd(ENABLE);
    while(SET != CLK_GetStableStatus(CLK_STB_FLAG_HRC)) {
        ;
    }

    /* MPLL 配置：HRC 16MHz / 2 * 50 / 2 = 200MHz
       输入分频后得到 8MHz 参考时钟，PLLM/N/P 取值见下行逐项注释 */
    stcMpllInit.PLLCFGR = 0UL;
    stcMpllInit.PLLCFGR_f.PLLM = 2UL - 1UL;  // 分频系数M=2，16MHz/2=8MHz
    stcMpllInit.PLLCFGR_f.PLLN = 50UL - 1UL; // 倍频系数N=50，8MHz*50=400MHz(VCO)
    stcMpllInit.PLLCFGR_f.PLLP = 2UL - 1UL;  // P分频输出200MHz
    stcMpllInit.PLLCFGR_f.PLLQ = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLR = 2UL - 1UL;
    stcMpllInit.u8PLLState = CLK_PLL_ON;
    stcMpllInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_HRC; // 时钟源切换为内部HRC
    (void)CLK_PLLInit(&stcMpllInit);

    /* 等待PLL稳定 */
    while (SET != CLK_GetStableStatus(CLK_STB_FLAG_PLL)) {
        ;
    }

    /* SRAM 等待周期：与 200MHz 主频匹配 */
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    SRAM_SetWaitCycle((SRAM_SRAM12 | SRAM_SRAM3 | SRAM_SRAMR), SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);

    /* Flash 等待周期 5（200MHz 主频要求，禁止下调） */
    (void)EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);

    /* 切换到高性能模式 */
    (void)PWC_HighSpeedToHighPerformance();

    /* 系统时钟源切换为PLL，主频200MHz */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);

    /* Flash 缓存：复位后重新使能（保证取指性能） */
    EFM_CacheRamReset(ENABLE);
    EFM_CacheRamReset(DISABLE);
    EFM_CacheCmd(ENABLE);
}

/**
 * @brief  BSP Xtal32 initialize.
 * @param  None
 * @retval int32_t:
 *         - LL_OK: XTAL32 enable successfully
 *         - LL_ERR_TIMEOUT: XTAL32 enable timeout.
 */
__WEAKDEF int32_t BSP_XTAL32_Init(void)
{
    stc_clock_xtal32_init_t stcXtal32Init;
    stc_fcm_init_t stcFcmInit;
    uint32_t u32TimeOut = 0UL;
    uint32_t u32Time = HCLK_VALUE / 5UL;

    if (CLK_XTAL32_ON == READ_REG8(CM_CMU->XTAL32CR)) {
        /* Disable xtal32 */
        (void)CLK_Xtal32Cmd(DISABLE);
    }

    /* Xtal32 config */
    (void)CLK_Xtal32StructInit(&stcXtal32Init);
    stcXtal32Init.u8State  = CLK_XTAL32_ON;
    stcXtal32Init.u8Drv    = CLK_XTAL32_DRV_MID;
    stcXtal32Init.u8Filter = CLK_XTAL32_FILTER_ALL_MD;
    GPIO_AnalogCmd(BSP_XTAL32_PORT, BSP_XTAL32_PIN, ENABLE);
    (void)CLK_Xtal32Init(&stcXtal32Init);

    /* FCM config */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_FCM, ENABLE);
    (void)FCM_StructInit(&stcFcmInit);
    stcFcmInit.u32RefClock       = FCM_REF_CLK_MRC;
    stcFcmInit.u32RefClockDiv    = FCM_REF_CLK_DIV8192;
    stcFcmInit.u32RefClockEdge   = FCM_REF_CLK_RISING;
    stcFcmInit.u32TargetClock    = FCM_TARGET_CLK_XTAL32;
    stcFcmInit.u32TargetClockDiv = FCM_TARGET_CLK_DIV1;
    stcFcmInit.u16LowerLimit     = (uint16_t)((XTAL32_VALUE / (MRC_VALUE / 8192U)) * 96UL / 100UL);
    stcFcmInit.u16UpperLimit     = (uint16_t)((XTAL32_VALUE / (MRC_VALUE / 8192U)) * 104UL / 100UL);
    (void)FCM_Init(CM_FCM, &stcFcmInit);
    /* Enable FCM, to ensure xtal32 stable */
    FCM_Cmd(CM_FCM, ENABLE);
    for (;;) {
        if (SET == FCM_GetStatus(CM_FCM, FCM_FLAG_END)) {
            FCM_ClearStatus(CM_FCM, FCM_FLAG_END);
            if (SET == FCM_GetStatus(CM_FCM, FCM_FLAG_ERR | FCM_FLAG_OVF)) {
                FCM_ClearStatus(CM_FCM, FCM_FLAG_ERR | FCM_FLAG_OVF);
            } else {
                (void)FCM_DeInit(CM_FCM);
                FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_FCM, DISABLE);
                return LL_OK;
            }
        }
        u32TimeOut++;
        if (u32TimeOut > u32Time) {
            (void)FCM_DeInit(CM_FCM);
            FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_FCM, DISABLE);
            return LL_ERR_TIMEOUT;
        }
    }
}


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

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/

