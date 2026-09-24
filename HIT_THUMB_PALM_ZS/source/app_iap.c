/**
 *******************************************************************************
 * @file    app_iap.c
 * @brief   APP 侧 IAP 升级触发模块实现（标志区布局与波特率说明见 app_iap.h 头部）
 *
 *          实现方式：擦除升级标志所在扇区后写入升级标志
 *          (0xA5B6C7D8 @ 0x0007FF8) 并软复位，由 boot 接管 YMODEM 升级。
 *          擦该扇区会连带清掉 0x7FF4(APP_RUN_ADDR) 与 0x7FFC(APP_EXIST_FLAG)，
 *          boot 升级成功后由 IAP_FinalizeUpdate() 重写，属既定设计。
 *******************************************************************************
 */
#include "app_iap.h"
#include "hc32_ll_efm.h"
#include <string.h>

/*******************************************************************************
 * 局部宏定义
 ******************************************************************************/
/* 升级标志所在扇区基址(HC32F460 扇区大小 EFM_SECTOR_SIZE = 0x2000) */
#define APP_IAP_FLAG_SECTOR_BASE        (APP_IAP_UPGRADE_FLAG_ADDR & ~(EFM_SECTOR_SIZE - 1UL))

/*******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static uint32_t APP_IAP_EnterProgram(void);
static void APP_IAP_ExitProgram(uint32_t u32Primask);

/*******************************************************************************
 * 内部函数实现
 ******************************************************************************/
/**
 * @brief  进入 Flash 编程：关中断、等待 EFM 就绪、解锁并使能编程
 * @retval 保存的 PRIMASK 值(调用前中断使能状态)
 */
static uint32_t APP_IAP_EnterProgram(void)
{
    uint32_t u32Primask = __get_PRIMASK();

    __disable_irq();
    while (SET != EFM_GetStatus(EFM_FLAG_RDY)) {
    }
    EFM_REG_Unlock();
    EFM_FWMC_Cmd(ENABLE);

    return u32Primask;
}

/**
 * @brief  退出 Flash 编程：等待 EFM 就绪、关闭编程并恢复中断
 * @param  u32Primask  进入编程前保存的 PRIMASK 值
 */
static void APP_IAP_ExitProgram(uint32_t u32Primask)
{
    while (SET != EFM_GetStatus(EFM_FLAG_RDY)) {
    }
    EFM_FWMC_Cmd(DISABLE);
    if (0U == u32Primask) {
        __enable_irq();
    }
}

/*******************************************************************************
 * 全局函数实现
 ******************************************************************************/
/**
 * @brief  检查接收数据是否包含升级命令
 * @param  pu8Data  数据指针(不要求以 '\0' 结尾)
 * @param  u16Len   数据长度
 * @retval 1: 命中升级命令；0: 未命中
 */
uint8_t APP_IAP_CheckUpgradeCmd(const uint8_t *pu8Data, uint16_t u16Len)
{
    const uint8_t *pu8Cmd = (const uint8_t *)APP_IAP_UPGRADE_CMD;
    uint16_t u16CmdLen = (uint16_t)strlen(APP_IAP_UPGRADE_CMD);
    uint16_t i;

    if ((NULL == pu8Data) || (u16Len < u16CmdLen)) {
        return 0U;
    }
    for (i = 0U; i <= (u16Len - u16CmdLen); ++i) {
        if (0 == memcmp(&pu8Data[i], pu8Cmd, u16CmdLen)) {
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief  进入升级模式：写入升级标志并软复位
 *
 *         复位后 boot 上电检测到升级标志(0xA5B6C7D8 @ 0x0007FF8)，
 *         即进入 YMODEM 升级流程。
 */
void APP_IAP_EnterUpgradeMode(void)
{
    uint32_t u32Primask;

    /* 1. 关中断 + 解锁 EFM + 使能编程 */
    u32Primask = APP_IAP_EnterProgram();

    /* 2. 擦除升级标志所在扇区 */
    if (LL_OK == EFM_SectorErase(APP_IAP_FLAG_SECTOR_BASE)) {
        /* 3. 写入升级标志 */
        (void)EFM_ProgramWord(APP_IAP_UPGRADE_FLAG_ADDR, APP_IAP_UPGRADE_FLAG);
    }

    /* 4. 关闭编程 + 恢复中断 */
    APP_IAP_ExitProgram(u32Primask);

    /* 5. 软复位，boot 接管 YMODEM 升级 */
    NVIC_SystemReset();
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
