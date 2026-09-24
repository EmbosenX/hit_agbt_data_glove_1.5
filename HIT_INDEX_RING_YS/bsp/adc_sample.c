/*******************************************************************************
 * @file        adc_sample.c
 * @brief       Y0~Y34输出控制 + AS模拟开关 + PALM单通道ADC采集底层驱动
 * @author      
 * @date        2026-07-21
 * @version     V1.0
 ******************************************************************************/
#include "adc_sample.h"
#include <string.h>
#include "app_sample_task.h"
#include "app_tac_config.h"
/* ADC采集输入 */
#define ADC_UNIT       CM_ADC1
#define ADC_PERIPH_CLK      (FCG3_PERIPH_ADC1)
#define ADC_SEQ        ADC_SEQ_A
#define ADC_EOC_FLAG   ADC_FLAG_EOCA

// ADC PGA配置
#define AD_ADC_PGA_UNIT     (ADC_PGA1)
#define ADC_PGA_GAIN        ADC_PGA_GAIN_8
#define ADC_PGA_VSS         ADC_PGA_VSS_AVSS
// ADC 管脚通道配置
#define X0_ADC_PORT      (GPIO_PORT_A)
#define X0_ADC_PIN       (GPIO_PIN_00)
#define X0_ADC_CHANNEL   (ADC_CH0)
#define X0_ADC_PGA_INPUT (ADC_PGA_PIN_ADC1_PA0)


//ADC 参数宏
#define ADC_VREF            (3.3F)
#define ADC_ACCURACY        (1UL << 12U)
#define ADC_TIMEOUT_VAL     (1000U)
#define ADC_SAMPLE_TIME     (0x20U)    /* 采样周期，加长适配高阻传感器 */

#define SAMPLER_MUX_CHANNELS   (8U)

/* 模拟开关切换后的建立延时（us）：等待 Y 选通与 AS 通道导通稳定后再启动 ADC。
   DDL_DelayUS 为 __NO_OPTIMIZE 的软件空转，该值乘以采集点数直接构成一轮耗时；
   确认硬件实际建立时间后可将其调小以缩短采集周期（置 0 即完全去掉等待）。 */
#ifndef SAMPLE_MUX_SETTLE_US
#define SAMPLE_MUX_SETTLE_US   (1U)
#endif

/* 无引脚处于选中/使能状态的标记 */
#define SAMPLER_NONE           (0xFFU)

/* ====================== 静态内部函数声明 ====================== */
static void GPIO_Y_Init(void);
static void GPIO_AS_Init(void);
static void ADC_PGA_Init(void);
static void SetSinglePin(uint8_t u8Port, uint16_t u16Pin, en_pin_level_t enLevel);
static void sampler_switch_cols(uint8_t AREA, uint8_t N);
static void sampler_select_mux(uint8_t AREA,uint8_t u8XIndex);
static void Y_Channel_AllLow(void);
static void AS_Chip_Enable(en_as_chip_t enChip);
static void AS_Channel_Select(en_as_channel_t enCh);
static void AS_AllChip_Disable(void);
static void AS_Chip_DisableLast(void);
static void Sampler_SelectY(uint8_t u8XIndex);
static void Sampler_ResetYState(void);
static int32_t AdcPollingSample(uint8_t x,uint16_t *pu16AdcVal);
/* ====================== 管脚映射表：Y编号 -> PORT/PIN ====================== */
typedef struct
{
    uint8_t 			pstPort;
    uint16_t      u16Pin;
} stc_y_pin_map_t;

static const stc_y_pin_map_t m_stYPinMap[] =
{
    {PIN_Y0_PORT,  PIN_Y0_PIN},  // Y0
    {PIN_Y1_PORT,  PIN_Y1_PIN},  // Y1
    {PIN_Y2_PORT,  PIN_Y2_PIN},  // Y2
    {PIN_Y3_PORT,  PIN_Y3_PIN},  // Y3
    {PIN_Y4_PORT,  PIN_Y4_PIN},  // Y4
    {PIN_Y5_PORT,  PIN_Y5_PIN},  // Y5
    {PIN_Y6_PORT,  PIN_Y6_PIN},  // Y6
    {PIN_Y7_PORT,  PIN_Y7_PIN},  // Y7
    {PIN_Y8_PORT,  PIN_Y8_PIN},  // Y8
    {PIN_Y9_PORT,  PIN_Y9_PIN},  // Y9
    {PIN_Y10_PORT, PIN_Y10_PIN}, // Y10
    {PIN_Y11_PORT, PIN_Y11_PIN}, // Y11
    {PIN_Y12_PORT, PIN_Y12_PIN}, // Y12
    {PIN_Y13_PORT, PIN_Y13_PIN}, // Y13
    {PIN_Y14_PORT, PIN_Y14_PIN}, // Y14
    {PIN_Y15_PORT, PIN_Y15_PIN}, // Y15
    {PIN_Y16_PORT, PIN_Y16_PIN}, // Y16

};
#define Y_CH_TOTAL_NUM  (sizeof(m_stYPinMap)/sizeof(stc_y_pin_map_t))

/* 采集热路径状态缓存：
   m_u8CurYIndex - 当前被拉高的 Y 引脚索引（全局行号），SAMPLER_NONE 表示无。
                   用于把"每个点遍历该区域所有行各写一次 GPIO"优化为
                   "只拉低旧的、拉高新的两根"，GPIO 写入量由 O(行數)/点 降为 O(1)/点。
                   跨区域采集前必须复位，否则会残留上一区域拉高的引脚造成多点同时导通。
   m_u8LastChipEn - 上次被使能的 AS 芯片索引，用于把"每列固定禁用全部 3 个芯片"
                   优化为"只禁用已使能的那一个"。EN 为高电平禁用。 */
static uint8_t m_u8CurYIndex  = SAMPLER_NONE;
static uint8_t m_u8LastChipEn = SAMPLER_NONE;

/**
 * @brief 底层驱动总初始化：Y输出IO + AS控制IO + PALM ADC
 */
void ADC_SAMPLE_Init(void)
{
    GPIO_Y_Init();
    GPIO_AS_Init();
    ADC_PGA_Init();

    /* 上电默认全部关闭 */
    Y_Channel_AllLow();
    AS_AllChip_Disable();
}

/**
 * @brief Y0~Y34全部推挽输出初始化，默认低电平
 */
static void GPIO_Y_Init(void)
{
    stc_gpio_init_t stcGpioInit;
    uint8_t u8Idx;
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
		stcGpioInit.u16PinOutputType = PIN_OUT_TYPE_CMOS;

    for(u8Idx = 0; u8Idx < Y_CH_TOTAL_NUM; u8Idx++)
    {
        GPIO_Init(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin, &stcGpioInit);
        GPIO_OutputCmd(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin, ENABLE);
				SetSinglePin(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin, PIN_LOW);
    }
}

/**
 * @brief AS模拟开关控制管脚初始化（AS_EN1~4 / AS_S1~3 推挽输出）
 */
static void GPIO_AS_Init(void)
{
    stc_gpio_init_t stcGpioInit;
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
		stcGpioInit.u16PinOutputType = PIN_OUT_TYPE_CMOS;
	
    /* AS_EN */
    GPIO_Init(AS_EN0_PORT, AS_EN0_PIN, &stcGpioInit);
		GPIO_OutputCmd(AS_EN0_PORT, AS_EN0_PIN, ENABLE);
    GPIO_Init(AS_EN1_PORT, AS_EN1_PIN, &stcGpioInit);
		GPIO_OutputCmd(AS_EN1_PORT, AS_EN1_PIN, ENABLE);
    GPIO_Init(AS_EN2_PORT, AS_EN2_PIN, &stcGpioInit);
		GPIO_OutputCmd(AS_EN2_PORT, AS_EN2_PIN, ENABLE);
    /* AS_S 通道选择 */
    GPIO_Init(AS_S1_PORT, AS_S1_PIN, &stcGpioInit);
		GPIO_OutputCmd(AS_S1_PORT, AS_S1_PIN, ENABLE);
    GPIO_Init(AS_S2_PORT, AS_S2_PIN, &stcGpioInit);
		GPIO_OutputCmd(AS_S2_PORT, AS_S2_PIN, ENABLE);
    GPIO_Init(AS_S3_PORT, AS_S3_PIN, &stcGpioInit);
		GPIO_OutputCmd(AS_S3_PORT, AS_S3_PIN, ENABLE);

    /* 默认全部关闭 */
    AS_AllChip_Disable();
}
/**
 * @brief ADC时钟配置（对齐原厂例程规范）
 * @note  适配200MHz系统主频，PCLK2=50MHz(ADC模拟时钟)，PCLK4=100MHz(数字接口)，比例2:1符合规格
 */
static void AdcClockConfig(void)
{
    /* 选择系统PCLK作为ADC时钟源 */
    CLK_SetPeriClockSrc(CLK_PERIPHCLK_PCLK);
}

/**
 * @brief PALM_RP_AD01 ADC1单通道初始化
 */
static void ADC_PGA_Init(void)
{
    stc_adc_init_t stcAdcInit;
    stc_gpio_init_t stcGpioInit;

    /* 模拟输入GPIO */
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
		stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
		GPIO_Init(X0_ADC_PORT, X0_ADC_PIN, &stcGpioInit);
		/*ADC基础初始化，直接复用官方默认配置 */
		AdcClockConfig();
		FCG_Fcg3PeriphClockCmd(ADC_PERIPH_CLK, ENABLE);
    (void)ADC_StructInit(&stcAdcInit);
    (void)ADC_Init(ADC_UNIT, &stcAdcInit);
    /*开启目标通道 */
    ADC_ChCmd(ADC_UNIT, ADC_SEQ, X0_ADC_CHANNEL, DISABLE);
    /*PGA初始化 */
    for (uint16_t u8Ch = X0_ADC_CHANNEL; u8Ch <= X0_ADC_CHANNEL; u8Ch++) {
        ADC_SetSampleTime(ADC_UNIT, u8Ch, ADC_SAMPLE_TIME);
    }
		/* 1. 配置增益、VSS参考 */
    ADC_PGA_Config(ADC_UNIT, AD_ADC_PGA_UNIT, ADC_PGA_GAIN, ADC_PGA_VSS);
    /* 3. 使能PGA */
    ADC_PGA_Cmd(ADC_UNIT, AD_ADC_PGA_UNIT, ENABLE);
}

/* ====================== Y通道操作函数 ====================== */
static void SetSinglePin(uint8_t u8Port, uint16_t u16Pin, en_pin_level_t enLevel)
{
    if(PIN_HIGH == enLevel)
    {
        GPIO_SetPins(u8Port, u16Pin);
    }
    else
    {
        GPIO_ResetPins(u8Port, u16Pin);
    }
}

static void Y_Channel_AllLow(void)
{
    uint8_t u8Idx;
    for(u8Idx = 0; u8Idx < Y_CH_TOTAL_NUM; u8Idx++)
    {
        GPIO_ResetPins(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin);
    }
}

/* ====================== AS模拟开关操作函数 ====================== */
static void AS_Chip_Enable(en_as_chip_t enChip)
{
    en_pin_level_t enPin = PIN_LOW;
    switch(enChip)
    {
        case AS_CHIP_0: SetSinglePin(AS_EN0_PORT, AS_EN0_PIN, enPin); m_u8LastChipEn = 0U; break;
        case AS_CHIP_1: SetSinglePin(AS_EN1_PORT, AS_EN1_PIN, enPin); m_u8LastChipEn = 1U; break;
        case AS_CHIP_2: SetSinglePin(AS_EN2_PORT, AS_EN2_PIN, enPin); m_u8LastChipEn = 2U; break;
        default: break;
    }
}

static void AS_Channel_Select(en_as_channel_t enCh)
{
    /* S3 S2 S1 三位控制位 */
    SetSinglePin(AS_S1_PORT, AS_S1_PIN, (enCh & 0x01U) ? PIN_HIGH : PIN_LOW);
    SetSinglePin(AS_S2_PORT, AS_S2_PIN, (enCh & 0x02U) ? PIN_HIGH : PIN_LOW);
    SetSinglePin(AS_S3_PORT, AS_S3_PIN, (enCh & 0x04U) ? PIN_HIGH : PIN_LOW);
}

static void AS_AllChip_Disable(void)
{
    SetSinglePin(AS_EN0_PORT, AS_EN0_PIN, PIN_HIGH);
		SetSinglePin(AS_EN1_PORT, AS_EN1_PIN, PIN_HIGH);
		SetSinglePin(AS_EN2_PORT, AS_EN2_PIN, PIN_HIGH);
    m_u8LastChipEn = SAMPLER_NONE;
}

/* 只禁用上次使能的那一个 AS 芯片（1 次 GPIO），替代固定写 3 次的 AS_AllChip_Disable。
   未使能的芯片 EN 本来就为高，无需重复拉高；每个区域采集开始时仍会调用
   AS_AllChip_Disable 做一次全量复位，避免状态缓存与硬件不同步造成多片同时导通 */
static void AS_Chip_DisableLast(void)
{
    switch(m_u8LastChipEn)
    {
        case 0U: SetSinglePin(AS_EN0_PORT, AS_EN0_PIN, PIN_HIGH); break;
        case 1U: SetSinglePin(AS_EN1_PORT, AS_EN1_PIN, PIN_HIGH); break;
        case 2U: SetSinglePin(AS_EN2_PORT, AS_EN2_PIN, PIN_HIGH); break;
        default: break;
    }
    m_u8LastChipEn = SAMPLER_NONE;
}

/**
 * @brief Y通道选择
 * @param[out] N 通道0~XX
 */
static void sampler_switch_cols(uint8_t AREA,uint8_t N)
{
    en_as_chip_t mux_index;
    en_as_channel_t channel;

		switch(AREA)
		{
				case TAC_AREA_THUMB_DIP:
				case TAC_AREA_THUMB_MCP:
					break;
				case TAC_AREA_INDEX_DIP:
				case TAC_AREA_INDEX_PIP:
				case TAC_AREA_INDEX_MCP:
					mux_index = (en_as_chip_t)((TAC_COL_INDEX_MCP_END-N+4) / SAMPLER_MUX_CHANNELS);//AS_EN芯片选择
					channel = (en_as_channel_t)((TAC_COL_INDEX_MCP_END-N+4) % SAMPLER_MUX_CHANNELS);//芯片的X通道选择
					AS_Chip_DisableLast();
					AS_Channel_Select(channel);
					AS_Chip_Enable((en_as_chip_t)(mux_index+AS_CHIP_1));
					break;
				case TAC_AREA_MIDDLE_DIP:
				case TAC_AREA_MIDDLE_PIP:
				case TAC_AREA_MIDDLE_MCP:
					break;
				case TAC_AREA_RING_DIP:
				case TAC_AREA_RING_PIP:
				case TAC_AREA_RING_MCP:
					mux_index = (en_as_chip_t)((TAC_COL_RING_MCP_END-N) / SAMPLER_MUX_CHANNELS);//AS_EN芯片选择
					channel = (en_as_channel_t)((TAC_COL_RING_MCP_END-N) % SAMPLER_MUX_CHANNELS);//芯片的X通道选择
					AS_Chip_DisableLast();
					AS_Channel_Select(channel);
					AS_Chip_Enable(mux_index);
					break;
				case TAC_AREA_LITTLE_DIP:
				case TAC_AREA_LITTLE_PIP:
				case TAC_AREA_LITTLE_MCP:
					break;
				case TAC_AREA_PALM:
					break;
				default:break;
		}
}

/**
 * @brief X通道选择
 * @param[out] u8XIndex 通道0~XX
 */
/* 增量切换 Y 选通引脚：仅当目标索引变化时拉低旧的、拉高新的（2 次 GPIO，索引未变则 0 次）。
   原实现遍历该区域所有行各写一次 GPIO，DIP 区达 11 次/点，是本驱动最大的耗时来源。 */
static void Sampler_SelectY(uint8_t u8XIndex)
{
    if (u8XIndex != m_u8CurYIndex) {
        if (m_u8CurYIndex != SAMPLER_NONE) {
            SetSinglePin(m_stYPinMap[m_u8CurYIndex].pstPort,
                         m_stYPinMap[m_u8CurYIndex].u16Pin, PIN_LOW);
        }
        SetSinglePin(m_stYPinMap[u8XIndex].pstPort,
                     m_stYPinMap[u8XIndex].u16Pin, PIN_HIGH);
        m_u8CurYIndex = u8XIndex;
    }
}

/* 复位 Y 选通状态：拉低当前引脚并清除缓存标记。
   跨区域采集前必须调用——不同区域的 row 范围不同，若不复位会残留上一区域
   拉高的引脚，造成多点同时导通、采样串扰。 */
static void Sampler_ResetYState(void)
{
    if (m_u8CurYIndex != SAMPLER_NONE) {
        SetSinglePin(m_stYPinMap[m_u8CurYIndex].pstPort,
                     m_stYPinMap[m_u8CurYIndex].u16Pin, PIN_LOW);
        m_u8CurYIndex = SAMPLER_NONE;
    }
}

static void sampler_select_mux(uint8_t AREA,uint8_t u8XIndex)
{
		switch(AREA)
		{
				case TAC_AREA_THUMB_DIP:
				case TAC_AREA_THUMB_MCP:
					break;
				case TAC_AREA_INDEX_DIP:
				case TAC_AREA_INDEX_PIP:
				case TAC_AREA_INDEX_MCP:
					Sampler_SelectY(u8XIndex);
					break;
				case TAC_AREA_MIDDLE_DIP:
				case TAC_AREA_MIDDLE_PIP:
				case TAC_AREA_MIDDLE_MCP:
					break;
				case TAC_AREA_RING_DIP:
				case TAC_AREA_RING_PIP:
				case TAC_AREA_RING_MCP:
					Sampler_SelectY(u8XIndex);
					break;
				case TAC_AREA_LITTLE_DIP:
				case TAC_AREA_LITTLE_PIP:
				case TAC_AREA_LITTLE_MCP:
					break;
				case TAC_AREA_PALM:
					break;
				default:break;
		}
}

/* ====================== PALM ADC采集函数 ====================== */
/**
 * @brief 轮询ADC采集，带超时保护，复刻官方AdcPolling
 * @param[out] pu16AdcVal 采样结果
 * @retval LL_OK：成功，LL_ERR：超时失败
 */
static int32_t AdcPollingSample(uint8_t x,uint16_t *pu16AdcVal)
{
    int32_t iRet = LL_ERR;
    __IO uint32_t u32TimeCount = 0UL;
    static uint8_t u8CurEnabledCh = 0xFFU; /* 缓存当前已使能的通道，0xFF表示无 */

    /* 通道切换：仅当目标通道与当前使能通道不同时，重新配置 */
    if (x != u8CurEnabledCh) {
				ADC_PGA_SelectInputSrc(ADC_UNIT, (uint32_t)(1<<x));
        /* 关闭所有7个通道 */
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, X0_ADC_CHANNEL, DISABLE);
        /* 仅使能目标通道 */
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, x, ENABLE);
        u8CurEnabledCh = x;
        /* 通道切换后加1us稳定延时，确保多路开关建立完成 */
        DDL_DelayUS(1);
    }
    ADC_Start(ADC_UNIT);
    do
    {
        if(SET == ADC_GetStatus(ADC_UNIT, ADC_EOC_FLAG))
        {
            ADC_ClearStatus(ADC_UNIT, ADC_EOC_FLAG);
						*pu16AdcVal = ADC_GetValue(ADC_UNIT, x);
            iRet = LL_OK;
            break;
        }
    } while (u32TimeCount++ < ADC_TIMEOUT_VAL);

    if(LL_ERR == iRet)
    {
        ADC_Stop(ADC_UNIT);
    }
    return iRet;
}
/* ====================== 对外采集函数 ====================== */
/**
 * @brief 轮询ADC采集所有数据
 * @param[out] pu16AdcVal 采样结果
 * @retval LL_OK：成功，LL_ERR：超时失败
 */
uint16_t SampleGetFrame(uint8_t AREA,uint8_t row, uint8_t col, uint8_t buffer[][col])
{
    int i;
    int j;
		uint16_t length=0;
		int32_t iRet = LL_ERR;
		const tac_data_cfg_t* pTacMaskData = GetTacMaskData();

    /* 进入本区域前复位 Y 选通与 AS 芯片状态缓存：不同区域的 row 范围不同，
       残留上一区域拉高的引脚会造成多点同时导通、采样串扰 */
    Sampler_ResetYState();
    AS_AllChip_Disable();

    for (i = TacAreaRange[AREA].col_start; i <= TacAreaRange[AREA].col_end; i++) {
        sampler_switch_cols(AREA,(uint8_t)i);
        for (j = TacAreaRange[AREA].row_start; j <= TacAreaRange[AREA].row_end; j++) {
            uint16_t value = 0U;
						uint8_t u8R = (uint8_t)(j - TacAreaRange[AREA].row_start);
						uint8_t u8C = (uint8_t)(i - TacAreaRange[AREA].col_start);
						uint8_t u8Valid = 0U;

            /* 先判定该点是否有效（mask 非 0），再决定是否切换模拟开关与采样 */
						switch(AREA)
						{
							case TAC_AREA_THUMB_DIP:
								break;
							case TAC_AREA_THUMB_MCP:
								break;
							case TAC_AREA_INDEX_DIP:
								u8Valid = (pTacMaskData->index_dip[u8R][u8C] == 0U) ? 0U : 1U;
								break;
							case TAC_AREA_INDEX_PIP:
								u8Valid = (pTacMaskData->index_pip[u8R][u8C] == 0U) ? 0U : 1U;
								break;
							case TAC_AREA_INDEX_MCP:
								u8Valid = (pTacMaskData->index_mcp[u8R][u8C] == 0U) ? 0U : 1U;
								break;
							case TAC_AREA_MIDDLE_DIP:
								break;
							case TAC_AREA_MIDDLE_PIP:
								break;
							case TAC_AREA_MIDDLE_MCP:
								break;
							case TAC_AREA_RING_DIP:
								u8Valid = (pTacMaskData->ring_dip[u8R][u8C] == 0U) ? 0U : 1U;
								break;
							case TAC_AREA_RING_PIP:
								u8Valid = (pTacMaskData->ring_pip[u8R][u8C] == 0U) ? 0U : 1U;
								break;
							case TAC_AREA_RING_MCP:
								u8Valid = (pTacMaskData->ring_mcp[u8R][u8C] == 0U) ? 0U : 1U;
								break;
							case TAC_AREA_LITTLE_DIP:
								break;
							case TAC_AREA_LITTLE_PIP:
								break;
							case TAC_AREA_LITTLE_MCP:
								break;
							case TAC_AREA_PALM:
								break;
							default:break;
						}

						/* 无效点（mask 为 0，或本工程不采集的区域）：直接写 0，
						   跳过模拟开关切换、建立延时与 ADC 采样，省掉无效开销 */
						if(u8Valid == 0U)
						{
								buffer[u8R][u8C] = 0U;
								continue;
						}

						sampler_select_mux(AREA,(uint8_t)j);
						DDL_DelayUS(SAMPLE_MUX_SETTLE_US);
						iRet = AdcPollingSample(X0_ADC_CHANNEL,&value);

						if(iRet==LL_ERR)
						{
							buffer[u8R][u8C] = 0U;
						}
						else{
							buffer[u8R][u8C] = (uint8_t)((value >> 4U) & 0xFFU);
						}
        }
    }
		length = col*row;
    Y_Channel_AllLow();
    m_u8CurYIndex = SAMPLER_NONE;
    AS_AllChip_Disable();
		return length;
}
#if 0
/**
 * @brief  从原始帧中提取指定矩形区域
 * @param  src       源二维数组首地址（连续内存）
 * @param  src_col   源数组总列数
 * @param  dst       目标缓冲区
 * @param  row_start 起始行
 * @param  row_cnt   提取行数
 * @param  col_start 起始列
 * @param  col_cnt   提取列数
 */
void tac_extract_area(const uint8_t *src, uint8_t src_col,
                      uint8_t *dst,
                      uint8_t row_start, uint8_t row_cnt,
                      uint8_t col_start, uint8_t col_cnt)
{
    for (uint8_t r = 0; r < row_cnt; r++) {
        const uint8_t *src_row = src + (row_start + r) * src_col + col_start;
        uint8_t *dst_row = dst + r * col_cnt;
        memcpy(dst_row, src_row, col_cnt * sizeof(uint8_t));
    }
}
#endif
/**
 * @brief  实时采集一整帧矩阵ADC数据，按指定格式输出
 * @param  pu8Data     输出数据缓冲区
 * @return uint16_t    实际输出的有效字节数
 * @note  
 */
uint16_t Adc_ReadFrame(uint8_t *pu8Data)
{
    uint16_t u16Offset = 0U;
    const tac_data_cfg_t* pTacData = GetTacDataCfg();

    /* 1. 参数校验 */
    if (pu8Data == NULL) {
        return 0U;
    }

    /*填充帧头 */
    pu8Data[u16Offset++] = 0x01U;                  // 区域标识：食指远节
    pu8Data[u16Offset++] = (uint8_t)(TAC_ROW_INDEX_DIP_END - TAC_ROW_INDEX_DIP_START + 1U);     // 行数
    pu8Data[u16Offset++] = (uint8_t)(TAC_COL_INDEX_DIP_END - TAC_COL_INDEX_DIP_START + 1U);     // 列数
    memcpy(&pu8Data[u16Offset], pTacData->index_dip, sizeof(pTacData->index_dip));
    u16Offset += sizeof(pTacData->index_dip);
		
    pu8Data[u16Offset++] = 0x02U;                  // 区域标识：食指中节
    pu8Data[u16Offset++] = (uint8_t)(TAC_ROW_INDEX_PIP_END - TAC_ROW_INDEX_PIP_START + 1U);     // 行数
    pu8Data[u16Offset++] = (uint8_t)(TAC_COL_INDEX_PIP_END - TAC_COL_INDEX_PIP_START + 1U);     // 列数
    memcpy(&pu8Data[u16Offset], pTacData->index_pip, sizeof(pTacData->index_pip));
    u16Offset += sizeof(pTacData->index_pip);
		
    pu8Data[u16Offset++] = 0x03U;                  // 区域标识：食指近节
    pu8Data[u16Offset++] = (uint8_t)(TAC_ROW_INDEX_MCP_END - TAC_ROW_INDEX_MCP_START + 1U);     // 行数
    pu8Data[u16Offset++] = (uint8_t)(TAC_COL_INDEX_MCP_END - TAC_COL_INDEX_MCP_START + 1U);     // 列数
    memcpy(&pu8Data[u16Offset], pTacData->index_mcp, sizeof(pTacData->index_mcp));
    u16Offset += sizeof(pTacData->index_mcp);
		
    pu8Data[u16Offset++] = 0x04U;                  // 区域标识：无名指远节
    pu8Data[u16Offset++] = (uint8_t)(TAC_ROW_RING_DIP_END - TAC_ROW_RING_DIP_START + 1U);     // 行数
    pu8Data[u16Offset++] = (uint8_t)(TAC_COL_RING_DIP_END - TAC_COL_RING_DIP_START + 1U);     // 列数
    memcpy(&pu8Data[u16Offset], pTacData->ring_dip, sizeof(pTacData->ring_dip));
    u16Offset += sizeof(pTacData->ring_dip);
		
    pu8Data[u16Offset++] = 0x05U;                  // 区域标识：无名指中节
    pu8Data[u16Offset++] = (uint8_t)(TAC_ROW_RING_PIP_END - TAC_ROW_RING_PIP_START + 1U);     // 行数
    pu8Data[u16Offset++] = (uint8_t)(TAC_COL_RING_PIP_END - TAC_COL_RING_PIP_START + 1U);     // 列数
    memcpy(&pu8Data[u16Offset], pTacData->ring_pip, sizeof(pTacData->ring_pip));
    u16Offset += sizeof(pTacData->ring_pip);
		
    pu8Data[u16Offset++] = 0x06U;                  // 区域标识：无名指近节
    pu8Data[u16Offset++] = (uint8_t)(TAC_ROW_RING_MCP_END - TAC_ROW_RING_MCP_START + 1U);     // 行数
    pu8Data[u16Offset++] = (uint8_t)(TAC_COL_RING_MCP_END - TAC_COL_RING_MCP_START + 1U);     // 列数
    memcpy(&pu8Data[u16Offset], pTacData->ring_mcp, sizeof(pTacData->ring_mcp));
    u16Offset += sizeof(pTacData->ring_mcp);
    /* 4. 返回总字节数（帧头 + payload 全部长度） */
    return u16Offset;
}

