#include "app_sample_task.h"
#include "main.h"
#include "app_tac_config.h"

/******************************************************************************
 * 配置宏
 ******************************************************************************/
/* 置 1 开启"一轮采集耗时"实测：用 DWT 周期计数器统计六个区域采集的总耗时，
   结果存 g_u32SampleCostCycle（单位 = HCLK 周期，5ns @200MHz）。
   仅在需要评估采集耗时/验证优化效果时开启，发布版本保持 0（测量代码不编译）。 */
#ifndef SAMPLE_TIME_MEASURE
#define SAMPLE_TIME_MEASURE         (0U)
#endif

#if (SAMPLE_TIME_MEASURE != 0U)
/* Cortex-M4 DWT 寄存器（固定地址，不依赖 CMSIS 符号，避免头文件差异） */
#define SAMPLE_DWT_CTRL     (*(volatile uint32_t *)0xE0001000UL)
#define SAMPLE_DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004UL)
#define SAMPLE_DWT_DEMCR    (*(volatile uint32_t *)0xE000EDFCUL)
#define SAMPLE_DWT_TRCENA   (0x01000000UL)   /* DEMCR.TRCENA */
#define SAMPLE_DWT_CNTENA   (0x00000001UL)   /* DWT_CTRL.CYCCNTENA */

volatile uint32_t g_u32SampleCostCycle = 0U;
volatile uint32_t g_u32SampleRoundCnt  = 0U;
#endif

/******************************************************************************
 * 静态任务资源（仅本文件内可见）
 ******************************************************************************/
static StaticTask_t  xSampleTaskTcb;
static StackType_t   uxSampleTaskStack[1024];
TaskHandle_t  xSampleTaskHandle = NULL;

/******************************************************************************
 * 内部任务函数声明
 ******************************************************************************/
static void AppTaskSample(void *pvParameters);
#if(DATA_FRAME_BYTE_NUM==2)
static void Sample_CalibrateZero(uint8_t row, uint8_t col, uint16_t frame[][col], uint16_t calib[][col]);
#else
static void Sample_CalibrateZero(uint8_t row, uint8_t col, uint8_t frame[][col], uint8_t calib[][col]);
#endif
#if TAC_CALIB_EN
static void Sample_ConvertWeight(tac_area_t area, uint8_t *frame);
#endif
/******************************************************************************
 * 对外接口函数
 ******************************************************************************/

uint8_t AppSampleTask_Init(void)
{
		ADC_SAMPLE_Init();

#if (SAMPLE_TIME_MEASURE != 0U)
    /* 使能 DWT 周期计数器，供采集耗时测量使用 */
    SAMPLE_DWT_DEMCR |= SAMPLE_DWT_TRCENA;
    SAMPLE_DWT_CYCCNT = 0U;
    SAMPLE_DWT_CTRL  |= SAMPLE_DWT_CNTENA;
#endif

    /* 优先级取 8，低于 SpiCommTask 的 9：主机 SPI 命令可抢占采样并及时得到处理，
       采样作为后台填充任务，被切碎不影响数据正确性（ADC 为轮询式，任务挂起期间
       模拟开关状态与 ADC 结果寄存器保持不变）。
       注意：本任务依赖 SpiCommTask 在空闲时阻塞让出 CPU 才能运行，二者必须成对配置 */
    xSampleTaskHandle = xTaskCreateStatic(
        AppTaskSample,
        "SampleTask",
        1024U,
        NULL,
        8U,
        uxSampleTaskStack,
        &xSampleTaskTcb
    );

    if (xSampleTaskHandle != NULL) {
        DDL_Printf("SampleTask success!\r\n");
        return 1U;
    }
    return 0U;
}

/******************************************************************************
 * 任务主体：触觉信息采集
 ******************************************************************************/
static void AppTaskSample(void *pvParameters)
{
		tac_data_cfg_t* pTacData = GetTacDataCfg();
		tac_data_cfg_t* pTacCalib = GetTacCalibData();
    (void)pvParameters;

    for (;;) {
        if (pdFALSE != ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
				{
#if (SAMPLE_TIME_MEASURE != 0U)
            uint32_t u32CyStart = SAMPLE_DWT_CYCCNT;
#endif
            /* 后续补充行列扫描、ADC采样、数据滤波逻辑 */
						SampleGetFrame(TAC_AREA_MIDDLE_DIP,
													TacAreaRange[TAC_AREA_MIDDLE_DIP].row_end-TacAreaRange[TAC_AREA_MIDDLE_DIP].row_start+1,
													TacAreaRange[TAC_AREA_MIDDLE_DIP].col_end-TacAreaRange[TAC_AREA_MIDDLE_DIP].col_start+1,
													pTacData->middle_dip);
					
						/* 后续补充行列扫描、ADC采样、数据滤波逻辑 */
						SampleGetFrame(TAC_AREA_MIDDLE_PIP,
													TacAreaRange[TAC_AREA_MIDDLE_PIP].row_end-TacAreaRange[TAC_AREA_MIDDLE_PIP].row_start+1,
													TacAreaRange[TAC_AREA_MIDDLE_PIP].col_end-TacAreaRange[TAC_AREA_MIDDLE_PIP].col_start+1,
													pTacData->middle_pip);

						/* 后续补充行列扫描、ADC采样、数据滤波逻辑 */
						SampleGetFrame(TAC_AREA_MIDDLE_MCP,
													TacAreaRange[TAC_AREA_MIDDLE_MCP].row_end-TacAreaRange[TAC_AREA_MIDDLE_MCP].row_start+1,
													TacAreaRange[TAC_AREA_MIDDLE_MCP].col_end-TacAreaRange[TAC_AREA_MIDDLE_MCP].col_start+1,
													pTacData->middle_mcp);

						/* 后续补充行列扫描、ADC采样、数据滤波逻辑 */
						SampleGetFrame(TAC_AREA_LITTLE_DIP,
													TacAreaRange[TAC_AREA_LITTLE_DIP].row_end-TacAreaRange[TAC_AREA_LITTLE_DIP].row_start+1,
													TacAreaRange[TAC_AREA_LITTLE_DIP].col_end-TacAreaRange[TAC_AREA_LITTLE_DIP].col_start+1,
													pTacData->little_dip);

						/* 后续补充行列扫描、ADC采样、数据滤波逻辑 */
						SampleGetFrame(TAC_AREA_LITTLE_PIP,
													TacAreaRange[TAC_AREA_LITTLE_PIP].row_end-TacAreaRange[TAC_AREA_LITTLE_PIP].row_start+1,
													TacAreaRange[TAC_AREA_LITTLE_PIP].col_end-TacAreaRange[TAC_AREA_LITTLE_PIP].col_start+1,
													pTacData->little_pip);

						/* 后续补充行列扫描、ADC采样、数据滤波逻辑 */
						SampleGetFrame(TAC_AREA_LITTLE_MCP,
													TacAreaRange[TAC_AREA_LITTLE_MCP].row_end-TacAreaRange[TAC_AREA_LITTLE_MCP].row_start+1,
													TacAreaRange[TAC_AREA_LITTLE_MCP].col_end-TacAreaRange[TAC_AREA_LITTLE_MCP].col_start+1,
													pTacData->little_mcp);

#if (SAMPLE_TIME_MEASURE != 0U)
            g_u32SampleCostCycle = SAMPLE_DWT_CYCCNT - u32CyStart;
            g_u32SampleRoundCnt++;
#endif
        }
    }
}
#if TAC_CALIB_EN
/**
 * @brief 将零点校准后的 AD 增量按区域标定表换算为克重（0~255g），写回 frame
 * @param  area   节级区域，内部自动映射到手指级区域
 * @param  frame  传感点数据首地址（内存连续，共 point_cnt 个点）
 */
static void Sample_ConvertWeight(tac_area_t area, uint8_t *frame)
{
    tac_finger_t finger = area_to_finger(area);
    uint16_t point_cnt = TacAreaRange[area].point_cnt;

    for (uint16_t k = 0; k < point_cnt; k++)
    {
        frame[k] = ad_to_weight(frame[k], finger);
    }
}
#endif /* TAC_CALIB_EN */

/**
 * @brief 传感器零点校准流程
 */
#if(DATA_FRAME_BYTE_NUM==2)
static void Sample_CalibrateZero(uint8_t row, uint8_t col, uint16_t frame[][col], uint16_t calib[][col])
{
    for (int i = 0; i < col; i++) 
		{
			for(int j = 0; j < row; j++)
			{
					uint16_t value = frame[j][i];
					value = (value > calib[j][i]) ? (value - calib[j][i]) : 0;
					frame[j][i] = value;
			}
    }
}
#else
static void Sample_CalibrateZero(uint8_t row, uint8_t col, uint8_t frame[][col], uint8_t calib[][col])
{
    for (int i = 0; i < col; i++) 
		{
			for(int j = 0; j < row; j++)
			{
					uint16_t value = frame[j][i];
					value = (value > calib[j][i]) ? (value - calib[j][i]) : 0;
					frame[j][i] = (uint8_t)(value&0xff);
			}
    }
}
#endif

