#include "app_iap_task.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

/******************************************************************************
 * 静态任务资源（仅本文件内可见）
 ******************************************************************************/
static StaticTask_t  xIapTaskTcb;
static StackType_t   uxIapTaskStack[512];
static TaskHandle_t  xIapTaskHandle = NULL;

/******************************************************************************
 * 全局IAP变量（对外可访问）
 ******************************************************************************/
uint8_t  g_u8IapFwBuf[IAP_FW_BUF_SIZE];
uint16_t g_u16IapFwLen;
uint32_t g_u32IapFwAddr;

/******************************************************************************
 * 内部任务函数声明
 ******************************************************************************/
static void AppTaskIap(void *pvParameters);

/******************************************************************************
 * 对外接口函数
 ******************************************************************************/

uint8_t AppIapTask_Init(void)
{
    xIapTaskHandle = xTaskCreateStatic(
        AppTaskIap,
        "IapTask",
        512U,
        NULL,
        9U,
        uxIapTaskStack,
        &xIapTaskTcb
    );

    if (xIapTaskHandle != NULL) {
        DDL_Printf("IapTask success!\r\n");
        return 1U;
    }
    return 0U;
}

void AppIapTask_NotifyUpgrade(void)
{
    if (xIapTaskHandle != NULL) {
        xTaskNotifyGive(xIapTaskHandle);
    }
}

/******************************************************************************
 * 任务主体：IAP在线升级
 ******************************************************************************/
static void AppTaskIap(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotifyValue;

    for (;;) {
        /* 阻塞等待升级通知，无通知时不占用CPU */
        ulNotifyValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotifyValue != 0U) {
            /* 用户升级逻辑 */
        }
    }
}


