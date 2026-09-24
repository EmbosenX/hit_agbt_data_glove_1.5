/*******************************************************************************
 * @file        adc_sample.h
 * @brief       手掌 PALM-Y0~Y15 列驱动 + RS2251模拟开关使能/通道选择 + PALM_RP_AD01 ADC采集驱动
 * @author      
 * @date        2026-07-21
 * @version     V1.0
 * @note        适配HC32F460PEHB-VFBGA100原理图(SCH_B06-YCJB-01_4-手掌)，单路ADC轮询多路模拟开关采集
 ******************************************************************************/
#ifndef __ADC_SAMPLE_H__
#define __ADC_SAMPLE_H__

#include "hc32_ll.h"

/* ====================== 硬件管脚定义（匹配 SCH_B06-YCJB-01_4-手掌 原理图） ====================== */
/* 手掌列驱动 Y 管脚 PALM-Y0 ~ PALM-Y15（16根，接主控 HC32F460 U16） */
#define PIN_PY0_PORT     GPIO_PORT_A   /* PALM-Y0 */
#define PIN_PY0_PIN      GPIO_PIN_11

#define PIN_PY1_PORT     GPIO_PORT_A   /* PALM-Y1 */
#define PIN_PY1_PIN      GPIO_PIN_10

#define PIN_PY2_PORT     GPIO_PORT_C   /* PALM-Y2 */
#define PIN_PY2_PIN      GPIO_PIN_09

#define PIN_PY3_PORT     GPIO_PORT_D   /* PALM-Y3 */
#define PIN_PY3_PIN      GPIO_PIN_14

#define PIN_PY4_PORT     GPIO_PORT_D   /* PALM-Y4 */
#define PIN_PY4_PIN      GPIO_PIN_13

#define PIN_PY5_PORT     GPIO_PORT_B   /* PALM-Y5 */
#define PIN_PY5_PIN      GPIO_PIN_12

#define PIN_PY6_PORT     GPIO_PORT_E   /* PALM-Y6 */
#define PIN_PY6_PIN      GPIO_PIN_15

#define PIN_PY7_PORT     GPIO_PORT_E   /* PALM-Y7 */
#define PIN_PY7_PIN      GPIO_PIN_14

#define PIN_PY8_PORT     GPIO_PORT_B   /* PALM-Y8 */
#define PIN_PY8_PIN      GPIO_PIN_10

#define PIN_PY9_PORT     GPIO_PORT_E   /* PALM-Y9 */
#define PIN_PY9_PIN      GPIO_PIN_13

#define PIN_PY10_PORT    GPIO_PORT_E   /* PALM-Y10 */
#define PIN_PY10_PIN     GPIO_PIN_12

#define PIN_PY11_PORT    GPIO_PORT_E   /* PALM-Y11 */
#define PIN_PY11_PIN     GPIO_PIN_11

#define PIN_PY12_PORT    GPIO_PORT_E   /* PALM-Y12 */
#define PIN_PY12_PIN     GPIO_PIN_10

#define PIN_PY13_PORT    GPIO_PORT_E   /* PALM-Y13 */
#define PIN_PY13_PIN     GPIO_PIN_09

#define PIN_PY14_PORT    GPIO_PORT_E   /* PALM-Y14 */
#define PIN_PY14_PIN     GPIO_PIN_08

#define PIN_PY15_PORT    GPIO_PORT_E   /* PALM-Y15 */
#define PIN_PY15_PIN     GPIO_PIN_07

/*大拇指输出管脚*/
#define PIN_TY0_PORT     GPIO_PORT_E   
#define PIN_TY0_PIN      GPIO_PIN_01

#define PIN_TY1_PORT     GPIO_PORT_E   
#define PIN_TY1_PIN      GPIO_PIN_02

#define PIN_TY2_PORT     GPIO_PORT_B   
#define PIN_TY2_PIN      GPIO_PIN_08

#define PIN_TY3_PORT     GPIO_PORT_B   
#define PIN_TY3_PIN      GPIO_PIN_09

#define PIN_TY4_PORT     GPIO_PORT_D   
#define PIN_TY4_PIN      GPIO_PIN_07

#define PIN_TY5_PORT     GPIO_PORT_B   
#define PIN_TY5_PIN      GPIO_PIN_06

#define PIN_TY6_PORT     GPIO_PORT_D  
#define PIN_TY6_PIN      GPIO_PIN_05

#define PIN_TY7_PORT     GPIO_PORT_D   
#define PIN_TY7_PIN      GPIO_PIN_06

#define PIN_TY8_PORT     GPIO_PORT_B   
#define PIN_TY8_PIN      GPIO_PIN_04

#define PIN_TY9_PORT     GPIO_PORT_D   
#define PIN_TY9_PIN      GPIO_PIN_04

#define PIN_TY10_PORT     GPIO_PORT_B   
#define PIN_TY10_PIN      GPIO_PIN_03

#define PIN_TY11_PORT     GPIO_PORT_D   
#define PIN_TY11_PIN      GPIO_PIN_03

#define PIN_TY12_PORT     GPIO_PORT_A  
#define PIN_TY12_PIN      GPIO_PIN_15

#define PIN_TY13_PORT     GPIO_PORT_D 
#define PIN_TY13_PIN      GPIO_PIN_01

/* 模拟开关使能（RS2251 x2，低电平有效；引脚以本处定义为准） */
#define AS_EN0_PORT     GPIO_PORT_D
#define AS_EN0_PIN      GPIO_PIN_09

#define AS_EN1_PORT     GPIO_PORT_D
#define AS_EN1_PIN      GPIO_PIN_12

#define AS_EN2_PORT     GPIO_PORT_C
#define AS_EN2_PIN      GPIO_PIN_13

#define AS_EN3_PORT     GPIO_PORT_E
#define AS_EN3_PIN      GPIO_PIN_05

/* 模拟开关通道地址 A/B/C = AS_S0/AS_S1/AS_S2（引脚以本处定义为准） */
#define AS_S0_PORT      GPIO_PORT_C
#define AS_S0_PIN       GPIO_PIN_08

#define AS_S1_PORT      GPIO_PORT_A
#define AS_S1_PIN       GPIO_PIN_09

#define AS_S2_PORT      GPIO_PORT_D
#define AS_S2_PIN       GPIO_PIN_08

#define MZ_AS_S0_PORT      GPIO_PORT_C
#define MZ_AS_S0_PIN       GPIO_PIN_15

#define MZ_AS_S1_PORT      GPIO_PORT_E
#define MZ_AS_S1_PIN       GPIO_PIN_06

#define MZ_AS_S2_PORT      GPIO_PORT_C
#define MZ_AS_S2_PIN       GPIO_PIN_14

/* 电平枚举 */
typedef enum
{
    PIN_LOW  = 0U,
    PIN_HIGH = 1U
} en_pin_level_t;

/* AS模拟开关通道选择编码 0~7 (S2 S1 S0 三位二进制) */
typedef enum
{
    AS_CH_0 = 0x00U,    // 000
    AS_CH_1 = 0x01U,    // 001
    AS_CH_2 = 0x02U,    // 010
    AS_CH_3 = 0x03U,    // 011
    AS_CH_4 = 0x04U,    // 100
    AS_CH_5 = 0x05U,    // 101
    AS_CH_6 = 0x06U,    // 110
    AS_CH_7 = 0x07U     // 111
} en_as_channel_t;

/* 模拟开关芯片编号（手掌共2片 RS2251） */
typedef enum
{
    AS_CHIP_0 = 0U,//掌心X0-X7     
    AS_CHIP_1 = 1U,//掌心X8-X15    
		AS_CHIP_2 = 2U,//大拇指X0-X7   
		AS_CHIP_3 = 3U,//大拇指X8-X13 
} en_as_chip_t;

/* 对外API */
void ADC_SAMPLE_Init(void);
uint16_t SampleGetFrame(uint8_t AREA, uint8_t row, uint8_t col, uint8_t buffer[][col]);
void tac_extract_area(const uint8_t *src, uint8_t src_col,
                      uint8_t *dst,
                      uint8_t row_start, uint8_t row_cnt,
                      uint8_t col_start, uint8_t col_cnt);
/* ADC单次采集函数 */

#endif /* __ADC_SAMPLE_H__ */
