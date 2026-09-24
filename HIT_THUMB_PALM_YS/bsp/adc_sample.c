/*******************************************************************************
 * @file        adc_sample.c
 * @brief       Y 通道输出控制（Y0 起，路数由 m_stYPinMap 决定）+ AS/MZ 模拟开关
 *              + PALM/THUMB 单通道 ADC 采集底层驱动
 * @author      
 * @date        2026-07-21
 * @version     V1.0
 ******************************************************************************/
#include "adc_sample.h"
#include <string.h>
#include "app_tac_config.h"

/* ADC采集输入 PALM_RP_AD01 (PA0) */
#define ADC_UNIT       CM_ADC1
#define ADC_PERIPH_CLK      (FCG3_PERIPH_ADC1)
#define ADC_SEQ        ADC_SEQ_A
#define ADC_EOC_FLAG   ADC_FLAG_EOCA

#define PALM_ADC_PORT       GPIO_PORT_A
#define PALM_ADC_PIN        GPIO_PIN_01
#define PALM_ADC_CH         ADC_CH1
#define PALM_ADC_PGA_PIN    ADC_PGA_PIN_ADC1_PA1

#define THUMB_ADC_PORT       GPIO_PORT_A
#define THUMB_ADC_PIN        GPIO_PIN_00
#define THUMB_ADC_CH         ADC_CH0
#define THUMB_ADC_PGA_PIN    ADC_PGA_PIN_ADC1_PA0
// ADC PGA配置
#define AD_ADC_PGA_UNIT     (ADC_PGA1)
#define ADC_PGA_GAIN        ADC_PGA_GAIN_3P2
#define ADC_PGA_VSS         ADC_PGA_VSS_AVSS

#define ADC_VREF            (3.3F)
#define ADC_ACCURACY        (1UL << 12U)
#define ADC_TIMEOUT_VAL     (1000U)
#define ADC_SAMPLE_TIME     (0x20U)    /* 采样周期，加长适配高阻传感器 */

#define SAMPLER_MUX_CHANNELS   (8U)
/* ====================== 静态内部函数声明 ====================== */
static void GPIO_Y_Init(void);
static void GPIO_AS_Init(void);
static void ADC_Palm_Init(void);
static void SetSinglePin(uint8_t u8Port, uint16_t u16Pin, en_pin_level_t enLevel);
static void sampler_switch_cols(uint8_t AREA,uint8_t N);
static void Y_Channel_AllLow(void);
static void AS_Chip_Enable(en_as_chip_t enChip);
static void AS_Channel_Select(en_as_channel_t enCh);
static void MZ_AS_Channel_Select(en_as_channel_t enCh);
static void AS_AllChip_Disable(void);
static int32_t AdcPollingSample(uint8_t adx,uint16_t *pu16AdcVal);
static void Y_PinSelectOnly(uint8_t u8Idx);
static uint8_t SampleAreaMaskValue(const tac_data_cfg_t *p, uint8_t AREA, uint8_t p_idx, uint8_t q_idx);
/* ====================== 管脚映射表：Y编号 -> PORT/PIN ====================== */
typedef struct
{
    uint8_t 			pstPort;
    uint16_t      u16Pin;
} stc_y_pin_map_t;

static const stc_y_pin_map_t m_stYPinMap[] =
{
    {PIN_PY0_PORT,  PIN_PY0_PIN},  // Y0
    {PIN_PY1_PORT,  PIN_PY1_PIN},  // Y1
    {PIN_PY2_PORT,  PIN_PY2_PIN},  // Y2
    {PIN_PY3_PORT,  PIN_PY3_PIN},  // Y3
    {PIN_PY4_PORT,  PIN_PY4_PIN},  // Y4
    {PIN_PY5_PORT,  PIN_PY5_PIN},  // Y5
    {PIN_PY6_PORT,  PIN_PY6_PIN},  // Y6
    {PIN_PY7_PORT,  PIN_PY7_PIN},  // Y7
    {PIN_PY8_PORT,  PIN_PY8_PIN},  // Y8
    {PIN_PY9_PORT,  PIN_PY9_PIN},  // Y9
    {PIN_PY10_PORT, PIN_PY10_PIN}, // Y10
    {PIN_PY11_PORT, PIN_PY11_PIN}, // Y11
    {PIN_PY12_PORT, PIN_PY12_PIN}, // Y12
    {PIN_PY13_PORT, PIN_PY13_PIN}, // Y13
    {PIN_PY14_PORT, PIN_PY14_PIN}, // Y14
    {PIN_PY15_PORT, PIN_PY15_PIN}, // Y15

    /* TY 段（Y16~Y29）为左右手固有差异项，禁止照抄 HIT_THUMB_PALM_ZS：
       右手装配（本工程）Y16->TY0 ... Y29->TY13（升序），
       左手装配（ZS 工程）Y16->TY13 ... Y29->TY0（反序） */
    {PIN_TY0_PORT,  PIN_TY0_PIN},  // Y16
    {PIN_TY1_PORT,  PIN_TY1_PIN},  // Y17
    {PIN_TY2_PORT,  PIN_TY2_PIN},  // Y18
    {PIN_TY3_PORT,  PIN_TY3_PIN},  // Y19
    {PIN_TY4_PORT,  PIN_TY4_PIN},  // Y20
    {PIN_TY5_PORT,  PIN_TY5_PIN},  // Y21
    {PIN_TY6_PORT,  PIN_TY6_PIN},  // Y22
    {PIN_TY7_PORT,  PIN_TY7_PIN},  // Y23
    {PIN_TY8_PORT,  PIN_TY8_PIN},  // Y24
    {PIN_TY9_PORT,  PIN_TY9_PIN},  // Y25
    {PIN_TY10_PORT, PIN_TY10_PIN}, // Y26
    {PIN_TY11_PORT, PIN_TY11_PIN}, // Y27
    {PIN_TY12_PORT, PIN_TY12_PIN}, // Y28
    {PIN_TY13_PORT, PIN_TY13_PIN}, // Y29

};
#define Y_CH_TOTAL_NUM  (sizeof(m_stYPinMap)/sizeof(stc_y_pin_map_t))

/**
 * @brief 底层驱动总初始化：Y输出IO + AS控制IO + PALM ADC
 */
void ADC_SAMPLE_Init(void)
{
    GPIO_Y_Init();
    GPIO_AS_Init();
    ADC_Palm_Init();

    /* 上电默认全部关闭 */
    Y_Channel_AllLow();
    AS_AllChip_Disable();
}

/**
 * @brief  全部 Y 通道推挽输出初始化，默认低电平
 * @note   通道表取自 m_stYPinMap（Y_CH_TOTAL_NUM 路），初始化后即完成一次全低清场
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
				GPIO_OutputCmd(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin,ENABLE);
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
    GPIO_Init(AS_EN1_PORT, AS_EN1_PIN, &stcGpioInit);
    GPIO_Init(AS_EN2_PORT, AS_EN2_PIN, &stcGpioInit);
    GPIO_Init(AS_EN3_PORT, AS_EN3_PIN, &stcGpioInit);
    /* AS_S 通道选择 */
    GPIO_Init(AS_S0_PORT, AS_S0_PIN, &stcGpioInit);
    GPIO_Init(AS_S1_PORT, AS_S1_PIN, &stcGpioInit);
    GPIO_Init(AS_S2_PORT, AS_S2_PIN, &stcGpioInit);
    /* MZ_AS_S 通道选择 */
    GPIO_Init(MZ_AS_S0_PORT, MZ_AS_S0_PIN, &stcGpioInit);
    GPIO_Init(MZ_AS_S1_PORT, MZ_AS_S1_PIN, &stcGpioInit);
    GPIO_Init(MZ_AS_S2_PORT, MZ_AS_S2_PIN, &stcGpioInit);
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
static void ADC_Palm_Init(void)
{
    stc_adc_init_t stcAdcInit;
    stc_gpio_init_t stcGpioInit;

    /* 模拟输入GPIO */
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
		stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
    GPIO_Init(PALM_ADC_PORT, PALM_ADC_PIN, &stcGpioInit);
    GPIO_Init(THUMB_ADC_PORT, THUMB_ADC_PIN, &stcGpioInit);

		/*ADC基础初始化，直接复用官方默认配置 */
    AdcClockConfig();
		FCG_Fcg3PeriphClockCmd(ADC_PERIPH_CLK, ENABLE);
    (void)ADC_StructInit(&stcAdcInit);
    (void)ADC_Init(ADC_UNIT, &stcAdcInit);

    /*开启目标通道 */
    ADC_ChCmd(ADC_UNIT, ADC_SEQ, PALM_ADC_CH, DISABLE);
		ADC_ChCmd(ADC_UNIT, ADC_SEQ, THUMB_ADC_CH, DISABLE);

    /*PGA初始化 */
		ADC_SetSampleTime(ADC_UNIT, PALM_ADC_CH, ADC_SAMPLE_TIME);
		ADC_SetSampleTime(ADC_UNIT, THUMB_ADC_CH, ADC_SAMPLE_TIME);
		/* 1. 配置增益、VSS参考 */
    ADC_PGA_Config(ADC_UNIT, AD_ADC_PGA_UNIT, ADC_PGA_GAIN, ADC_PGA_VSS);
    ADC_PGA_Cmd(ADC_UNIT, AD_ADC_PGA_UNIT, ENABLE);
}

/* ====================== Y通道操作函数 ====================== */
/**
 * @brief  设置单个 GPIO 引脚电平
 * @param  u8Port   端口号（GPIO_PORT_x）
 * @param  u16Pin   引脚掩码（GPIO_PIN_x）
 * @param  enLevel  PIN_HIGH=置高，PIN_LOW=置低
 */
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

/**
 * @brief  把所有 Y 垂直通道拉低（逐列扫描结束后的清场动作）
 * @note   遍历 m_stYPinMap 全部 Y_CH_TOTAL_NUM 个通道，只写 GPIO，无阻塞
 */
static void Y_Channel_AllLow(void)
{
    uint8_t u8Idx;
    for(u8Idx = 0; u8Idx < Y_CH_TOTAL_NUM; u8Idx++)
    {
        GPIO_ResetPins(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin);
    }
}

/* ====================== 采集通道增量切换（缩短每轮采集耗时） ====================== */
/* 背景：原实现每次采样点都"把整段行/列引脚全部重写一遍"（拇指每点最多 14 次 GPIO 写、
 * 掌心每列 15 次写），其中绝大多数写的是"本来就是低电平"的引脚，属纯浪费。
 * 这里改为增量切换：只关了旧引脚、只开新引脚（每点 2 次写）。
 * 状态约定：SampleGetFrame() 入口复位为"未知"，保证每个区域首个有效点强制重配；
 * 区域结束由 Y_Channel_AllLow() 物理清场，故跨区域无残留。 */
static uint8_t s_u8YHighIdx    = 0xFFU;  /* 当前点亮的 Y 引脚在 m_stYPinMap 中的下标，0xFF=全低 */
static uint8_t s_u8ThumbColKey = 0xFFU;  /* 拇指列 AS 通道组合键 (mux*8+ch)，0xFF=未配置 */

/**
 * @brief  增量点亮 Y 引脚：关闭上一次点亮的引脚、点亮目标引脚
 * @param  u8Idx  目标引脚在 m_stYPinMap 中的下标（0xFF=仅关闭，不点亮）
 * @note   与 s_u8YHighIdx 状态配合；与"整段重写"等价，因为同一区域内引脚范围固定，
 *         范围内其余引脚在本区域首个有效点切换时已由状态机保证为低电平。
 *         只写 GPIO，无阻塞。
 */
static void Y_PinSelectOnly(uint8_t u8Idx)
{
    if (u8Idx == s_u8YHighIdx)
    {
        return;   /* 与上次相同：零写操作 */
    }

    if (s_u8YHighIdx < Y_CH_TOTAL_NUM)
    {
        SetSinglePin(m_stYPinMap[s_u8YHighIdx].pstPort, m_stYPinMap[s_u8YHighIdx].u16Pin, PIN_LOW);
    }

    if (u8Idx < Y_CH_TOTAL_NUM)
    {
        SetSinglePin(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin, PIN_HIGH);
    }

    s_u8YHighIdx = u8Idx;
}

/* ====================== AS模拟开关操作函数 ====================== */
/**
 * @brief  使能指定模拟开关芯片（低电平有效）
 * @param  enChip  芯片编号 AS_CHIP_0~AS_CHIP_3
 * @note   非法编号不做任何动作
 */
static void AS_Chip_Enable(en_as_chip_t enChip)
{
    en_pin_level_t enPin = PIN_LOW;
    switch(enChip)
    {
        case AS_CHIP_0: SetSinglePin(AS_EN0_PORT, AS_EN0_PIN, enPin); break;
        case AS_CHIP_1: SetSinglePin(AS_EN1_PORT, AS_EN1_PIN, enPin); break;
        case AS_CHIP_2: SetSinglePin(AS_EN2_PORT, AS_EN2_PIN, enPin); break;
        case AS_CHIP_3: SetSinglePin(AS_EN3_PORT, AS_EN3_PIN, enPin); break;
        default: break;
    }
}

/**
 * @brief  选择模拟开关通道（AS_S0/S1/S2 三位地址，bit0~bit2 对应 S0~S2）
 * @param  enCh  通道号 0~7
 */
static void AS_Channel_Select(en_as_channel_t enCh)
{
    /* S3 S2 S1 三位控制位 */
    SetSinglePin(AS_S0_PORT, AS_S0_PIN, (enCh & 0x01U) ? PIN_HIGH : PIN_LOW);
    SetSinglePin(AS_S1_PORT, AS_S1_PIN, (enCh & 0x02U) ? PIN_HIGH : PIN_LOW);
    SetSinglePin(AS_S2_PORT, AS_S2_PIN, (enCh & 0x04U) ? PIN_HIGH : PIN_LOW);
}
/**
 * @brief  选择 MZ（掌心侧）模拟开关通道（独立一组 S0/S1/S2 引脚）
 * @param  enCh  通道号 0~7
 */
static void MZ_AS_Channel_Select(en_as_channel_t enCh)
{
    /* S3 S2 S1 三位控制位 */
    SetSinglePin(MZ_AS_S0_PORT, MZ_AS_S0_PIN, (enCh & 0x01U) ? PIN_HIGH : PIN_LOW);
    SetSinglePin(MZ_AS_S1_PORT, MZ_AS_S1_PIN, (enCh & 0x02U) ? PIN_HIGH : PIN_LOW);
    SetSinglePin(MZ_AS_S2_PORT, MZ_AS_S2_PIN, (enCh & 0x04U) ? PIN_HIGH : PIN_LOW);
}
/**
 * @brief  关闭全部模拟开关芯片（4 路使能引脚均置高）
 */
static void AS_AllChip_Disable(void)
{
    SetSinglePin(AS_EN0_PORT, AS_EN0_PIN, PIN_HIGH);
    SetSinglePin(AS_EN1_PORT, AS_EN1_PIN, PIN_HIGH);
    SetSinglePin(AS_EN2_PORT, AS_EN2_PIN, PIN_HIGH);
    SetSinglePin(AS_EN3_PORT, AS_EN3_PIN, PIN_HIGH);
}
/**
 * @brief  按区域与列号选通对应 Y 垂直通道（含模拟开关芯片与通道地址选择）
 * @param  AREA  节级区域枚举（决定使用哪组模拟开关与列分段）
 * @param  N     该区域的列号（TAC_COL_* 分段内的绝对列号）
 * @note   内部按 (列终点-列号)/SAMPLER_MUX_CHANNELS 选芯片、按 15-列号 取模选通道；
 *         只写 GPIO，无阻塞
 */
static void sampler_switch_cols(uint8_t AREA,uint8_t N)
{
    en_as_chip_t mux_index;
    en_as_channel_t channel;
		switch(AREA)
		{
				case TAC_AREA_THUMB_DIP:
				case TAC_AREA_THUMB_MCP:
				{
					uint8_t u8Key = (uint8_t)(((uint8_t)((TAC_COL_THUMB_DIP_END-N) / SAMPLER_MUX_CHANNELS) * 8U) +
					                          (uint8_t)((TAC_COL_THUMB_DIP_END-N) % SAMPLER_MUX_CHANNELS));

					/* 与上次配置相同：列通道未变，跳过"先全关再接通"的整套 AS 写 */
					if (u8Key == s_u8ThumbColKey)
					{
						break;
					}

					mux_index = (en_as_chip_t)((TAC_COL_THUMB_DIP_END-N) / SAMPLER_MUX_CHANNELS);//芯片选择
					channel = (en_as_channel_t)((TAC_COL_THUMB_DIP_END-N) % SAMPLER_MUX_CHANNELS);//芯片的X通道选择
					AS_AllChip_Disable();
					MZ_AS_Channel_Select(channel);
					AS_Chip_Enable((en_as_chip_t)(mux_index+AS_CHIP_2));
					s_u8ThumbColKey = u8Key;
					break;
				}
				case TAC_AREA_INDEX_DIP:
				case TAC_AREA_INDEX_PIP:
				case TAC_AREA_INDEX_MCP:
				case TAC_AREA_MIDDLE_DIP:
				case TAC_AREA_MIDDLE_PIP:
				case TAC_AREA_MIDDLE_MCP:
				case TAC_AREA_RING_DIP:
				case TAC_AREA_RING_PIP:
				case TAC_AREA_RING_MCP:
				case TAC_AREA_LITTLE_DIP:
				case TAC_AREA_LITTLE_PIP:
				case TAC_AREA_LITTLE_MCP:
					break;
				case TAC_AREA_PALM:
					/* 增量切换：只关了旧列、只开新列（原来每列全量重写 15 个引脚） */
					Y_PinSelectOnly((uint8_t)N);
					break;
				default:break;
		}
}

/**
 * @brief  按区域与行号选通 X（水平）通道：选择模拟开关芯片与芯片内通道地址
 * @param  AREA      节级区域枚举
 * @param  u8XIndex  该区域的绝对行号（0 起）
 * @note   先关闭全部 AS 芯片再切换通道地址，避免切换瞬间多路导通；
 *         只写 GPIO，无阻塞
 */
static void sampler_select_mux(uint8_t AREA,uint8_t u8XIndex)
{
    en_as_chip_t mux_index;
    en_as_channel_t channel;

		switch(AREA)
		{
			case TAC_AREA_THUMB_DIP:
			case TAC_AREA_THUMB_MCP:
			/* 增量切换：只关了旧行、只开新行（原来每点全量重写整段行引脚，最多 14 次写） */
			Y_PinSelectOnly((uint8_t)u8XIndex);
			break;
			case TAC_AREA_INDEX_DIP:break;
			case TAC_AREA_INDEX_PIP:break;
			case TAC_AREA_INDEX_MCP:break;
			case TAC_AREA_MIDDLE_DIP:break;
			case TAC_AREA_MIDDLE_PIP:break;
			case TAC_AREA_MIDDLE_MCP:break;
			case TAC_AREA_RING_DIP:break;
			case TAC_AREA_RING_PIP:break;
			case TAC_AREA_RING_MCP:break;
			case TAC_AREA_LITTLE_DIP:break;
			case TAC_AREA_LITTLE_PIP:break;
			case TAC_AREA_LITTLE_MCP:break;
			case TAC_AREA_PALM:
			/* 掌心 X 通道为左右手固有差异项，禁止照抄 HIT_THUMB_PALM_ZS：
			   右手（本工程）按 X 序号正序选通；左手（ZS 工程）为 (15-u8XIndex) 反序 */
			mux_index = (en_as_chip_t)((u8XIndex) / SAMPLER_MUX_CHANNELS);//芯片选择
			channel = (en_as_channel_t)((u8XIndex) % SAMPLER_MUX_CHANNELS);//芯片的X通道选择
			AS_AllChip_Disable();
			AS_Channel_Select(channel);
			AS_Chip_Enable(mux_index);
			break;
			default:break;
		}
}

/* ====================== PALM ADC采集函数 ====================== */
/**
 * @brief  单通道轮询 ADC 采集（带超时保护）
 * @param  adx          ADC 通道号（PALM_ADC_CH / THUMB_ADC_CH 等）
 * @param  pu16AdcVal   输出：采样结果
 * @retval LL_OK=成功；LL_ERR=超时（已停止 ADC）
 * @note   内部缓存当前使能通道，仅切换通道时才重配通道与 PGA 并延时 1us 等待建立；
 *         轮询上限 ADC_TIMEOUT_VAL 次，耗尽即判定超时
 */
static int32_t AdcPollingSample(uint8_t adx,uint16_t *pu16AdcVal)
{
    int32_t iRet = LL_ERR;
    __IO uint32_t u32TimeCount = 0UL;
    static uint8_t u8CurEnabledCh = 0xFFU; /* 缓存当前已使能的通道，0xFF表示无 */

    /* 通道切换：仅当目标通道与当前使能通道不同时，重新配置 */
    if (adx != u8CurEnabledCh) {
				ADC_PGA_SelectInputSrc(ADC_UNIT, (uint32_t)(1<<adx));
        /* 关闭所有7个通道 */
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, PALM_ADC_CH, DISABLE);
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, THUMB_ADC_CH, DISABLE);
        /* 仅使能目标通道 */
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, adx, ENABLE);
        u8CurEnabledCh = adx;
				if(adx == THUMB_ADC_CH)
				{
						ADC_PGA_Config(ADC_UNIT, AD_ADC_PGA_UNIT, ADC_PGA_GAIN_8, ADC_PGA_VSS);
						ADC_PGA_Cmd(ADC_UNIT, AD_ADC_PGA_UNIT, ENABLE);
				}
				else{
						ADC_PGA_Config(ADC_UNIT, AD_ADC_PGA_UNIT, ADC_PGA_GAIN, ADC_PGA_VSS);
						ADC_PGA_Cmd(ADC_UNIT, AD_ADC_PGA_UNIT, ENABLE);
				}
        /* 通道切换后加1us稳定延时，确保多路开关建立完成 */
        DDL_DelayUS(1);
    }
    ADC_Start(ADC_UNIT);
    do
    {
        if(SET == ADC_GetStatus(ADC_UNIT, ADC_EOC_FLAG))
        {
            ADC_ClearStatus(ADC_UNIT, ADC_EOC_FLAG);
            *pu16AdcVal = ADC_GetValue(ADC_UNIT, adx);
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
 * @brief  取指定区域内某点的掩码值（区域内部索引，0 起）
 * @param  p       掩码矩阵（GetTacMaskData() 返回值，可为 NULL）
 * @param  AREA    节级区域枚举
 * @param  p_idx   区域内行索引（0 起）
 * @param  q_idx   区域内列索引（0 起）
 * @retval 掩码值；p 为 NULL 或 AREA 非法时返回 0（按无效点处理）
 * @note   仅做索引取值、无阻塞；把"按区域取掩码"从采样循环里抽出来，便于把掩码判断前置
 */
static uint8_t SampleAreaMaskValue(const tac_data_cfg_t *p, uint8_t AREA, uint8_t p_idx, uint8_t q_idx)
{
    uint8_t u8Mask = 0U;

    if (NULL == p)
    {
        return 0U;
    }

    switch(AREA)
    {
        case TAC_AREA_THUMB_DIP:  u8Mask = p->thumb_dip[p_idx][q_idx];  break;
        case TAC_AREA_THUMB_MCP:  u8Mask = p->thumb_mcp[p_idx][q_idx];  break;
        case TAC_AREA_INDEX_DIP:  u8Mask = p->index_dip[p_idx][q_idx];  break;
        case TAC_AREA_INDEX_PIP:  u8Mask = p->index_pip[p_idx][q_idx];  break;
        case TAC_AREA_INDEX_MCP:  u8Mask = p->index_mcp[p_idx][q_idx];  break;
        case TAC_AREA_MIDDLE_DIP: u8Mask = p->middle_dip[p_idx][q_idx]; break;
        case TAC_AREA_MIDDLE_PIP: u8Mask = p->middle_pip[p_idx][q_idx]; break;
        case TAC_AREA_MIDDLE_MCP: u8Mask = p->middle_mcp[p_idx][q_idx]; break;
        case TAC_AREA_RING_DIP:   u8Mask = p->ring_dip[p_idx][q_idx];   break;
        case TAC_AREA_RING_PIP:   u8Mask = p->ring_pip[p_idx][q_idx];   break;
        case TAC_AREA_RING_MCP:   u8Mask = p->ring_mcp[p_idx][q_idx];   break;
        case TAC_AREA_LITTLE_DIP: u8Mask = p->little_dip[p_idx][q_idx]; break;
        case TAC_AREA_LITTLE_PIP: u8Mask = p->little_pip[p_idx][q_idx]; break;
        case TAC_AREA_LITTLE_MCP: u8Mask = p->little_mcp[p_idx][q_idx]; break;
        case TAC_AREA_PALM:       u8Mask = p->palm[p_idx][q_idx];       break;
        default: break;
    }

    return u8Mask;
}

/**
 * @brief  整区域 ADC 采集：按列选通 Y 通道，逐行采样并写入目标矩阵
 * @param  AREA    节级区域枚举（决定行列分段与 Y 选通方式）
 * @param  row     目标矩阵行数（提示用，实际遍历按 TacAreaRange[AREA] 分段）
 * @param  col     目标矩阵列数（作为 buffer 的列跨距）
 * @param  buffer  输出矩阵（行优先，列跨距 = col）
 * @retval 实际写入的有效点数
 * @note   掩码为 0 的无效点保持原值/0 不采样；内部使用阻塞式轮询采样，
 *         仅在采样任务整轮写窗口内调用（单实例）
 */
uint16_t SampleGetFrame(uint8_t AREA, uint8_t row, uint8_t col, uint8_t buffer[][col])
{
    int i;
    int j;
		uint16_t length=0;
		int32_t iRet = LL_ERR;
		const tac_data_cfg_t *pTacMaskData = GetTacMaskData();

    /* 增量切换状态复位：本区域首个有效点强制重配通道
       （区域结束已由 Y_Channel_AllLow()/AS_AllChip_Disable() 清场，跨区域无残留） */
    s_u8YHighIdx    = 0xFFU;
    s_u8ThumbColKey = 0xFFU;

    for (i = TacAreaRange[AREA].col_start; i <= TacAreaRange[AREA].col_end; i++) 
		{
        sampler_switch_cols(AREA, (uint8_t)i);
        for (j = TacAreaRange[AREA].row_start; j <= TacAreaRange[AREA].row_end; j++) 
				{
            uint16_t value;
						uint8_t p=0,q=0;

						p = (j-TacAreaRange[AREA].row_start);
						q = (i-TacAreaRange[AREA].col_start);

						/* 掩码判断前置（原实现为先切通道再判掩码）：无效点直接写 0 并跳过
						   通道切换、1us 建立延时与 ADC 采样，省掉无效点的固定开销 */
						if(0U == SampleAreaMaskValue(pTacMaskData, (uint8_t)AREA, p, q))
						{
							buffer[p][q] = 0;
							continue;
						}

						sampler_select_mux(AREA,(uint8_t)j);
						DDL_DelayUS(1);

						if((TAC_AREA_THUMB_DIP == AREA) || (TAC_AREA_THUMB_MCP == AREA))
						{
							iRet = AdcPollingSample(THUMB_ADC_CH,&value);
						}
						else
						{
							iRet = AdcPollingSample(PALM_ADC_CH,&value);
						}

						if(iRet==LL_ERR)
						{
							buffer[p][q] = 0;
						}
            else{
							buffer[p][q] = (uint8_t)((value >> 4U) & 0xFFU);
						}
        }
		}
		length = col*row;
    Y_Channel_AllLow();
    AS_AllChip_Disable();
    s_u8YHighIdx    = 0xFFU;   /* 与清场后的实际电平保持一致 */
    s_u8ThumbColKey = 0xFFU;
		return length;
}
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

