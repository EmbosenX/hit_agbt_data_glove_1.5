/**
 *******************************************************************************
 * @file    app_iap.h
 * @brief   APP 侧 IAP 升级触发模块
 *
 *          与 boot 工程(HIT_IAP_BOOT_UART)配合实现 IAP 升级：
 *          APP 收到升级命令后写入升级标志并软复位，
 *          boot 上电检测到标志后自动进入 YMODEM 升级。
 *
 *          注意：APP 不实现 YMODEM 协议，升级传输完全由 boot 承担。
 *
 *          波特率：两侧已解耦——boot 的 OTA 波特率固定为 921600
 *          (HIT_IAP_BOOT_UART/source/com.h 的 MODEM_USART_BAUD_RATE)，
 *          本 APP 运行期为 USART_BSP_BAUDRATE(3000000)。触发升级并复位后链路会
 *          切到 boot 的 921600，上位机需以「Boot 升级波特率」重连后再发送固件
 *          (web_calib_host 升级向导已自动切换，无需手工改数字)；升级完成后
 *          自动以 App 速率(3000000)连回。
 *******************************************************************************
 */
#ifndef __APP_IAP_H__
#define __APP_IAP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*******************************************************************************
 * 升级标志区定义：以 boot 工程 source/hc32f4xx_conf.h 为唯一权威来源
 *
 *   IAP_BOOT_SIZE          0x8000      Boot 占用 32 KB(见 HC32F460xE.sct)
 *   APP_EXIST_FLAG_ADDR    0x00007FFC  = IAP_BOOT_SIZE - 4
 *   APP_UPGRADE_FLAG_ADDR  0x00007FF8  = IAP_BOOT_SIZE - 8
 *   APP_RUN_ADDR           0x00007FF4  = IAP_BOOT_SIZE - 12
 *   IAP_APP_ADDR           0x00008000  = IAP_BOOT_SIZE，APP 运行/向量表地址
 *
 * 这些字段同处 0x2000 字节的最后一个扇区(基址 0x6000)，本模块只写升级标志，
 * 其余字段由 boot 的 IAP_FinalizeUpdate() 在升级成功后重写，属既定设计。
 * 切勿在本工程内另建一套分区宏：分区与标志地址以 boot 工程为唯一权威来源。
 ******************************************************************************/
#define APP_IAP_UPGRADE_FLAG_ADDR       (0x0007FF8UL)   /*!< = IAP_BOOT_SIZE - 8 */
#define APP_IAP_UPGRADE_FLAG            (0xA5B6C7D8UL)  /*!< boot: APP_UPGRADE_FLAG */

/*******************************************************************************
 * 升级触发命令(ASCII 子串匹配)：与上位机/手机端联调时在此处修改
 * 不含 \r\n：串口助手直接发送 "UPGRADE" 即可触发（兼容带真实/字面 CRLF 的发送）
 ******************************************************************************/
#define APP_IAP_UPGRADE_CMD             "UPGRADE"

/*******************************************************************************
 * 全局函数
 ******************************************************************************/
uint8_t APP_IAP_CheckUpgradeCmd(const uint8_t *pu8Data, uint16_t u16Len);
void APP_IAP_EnterUpgradeMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_IAP_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
