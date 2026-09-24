/*******************************************************************************
 * @file        adc_sample.h
 * @brief       Y0~Y34 IO输出控制 + AS模拟开关使能/通道选择 + PALM_RP_AD01 ADC采集驱动
 * @author      
 * @date        2026-07-21
 * @version     V1.0
 * @note        适配HC32F460PEHB-VFBGA100原理图，单路ADC轮询多路模拟开关采集
 ******************************************************************************/
#ifndef __ADC_SAMPLE_H__
#define __ADC_SAMPLE_H__

#include "hc32_ll.h"

/* ====================== 硬件管脚定义（严格匹配原理图） ====================== */
/* 通用输出Y管脚 Y0~Y34 */
#define PIN_Y0_PORT     GPIO_PORT_B
#define PIN_Y0_PIN      GPIO_PIN_09

#define PIN_Y1_PORT     GPIO_PORT_B
#define PIN_Y1_PIN      GPIO_PIN_08

#define PIN_Y2_PORT     GPIO_PORT_B
#define PIN_Y2_PIN      GPIO_PIN_07

#define PIN_Y3_PORT     GPIO_PORT_B
#define PIN_Y3_PIN      GPIO_PIN_06

#define PIN_Y4_PORT     GPIO_PORT_B
#define PIN_Y4_PIN      GPIO_PIN_05

#define PIN_Y5_PORT     GPIO_PORT_B
#define PIN_Y5_PIN      GPIO_PIN_04

#define PIN_Y6_PORT     GPIO_PORT_B
#define PIN_Y6_PIN      GPIO_PIN_03

#define PIN_Y7_PORT     GPIO_PORT_A
#define PIN_Y7_PIN      GPIO_PIN_15

#define PIN_Y8_PORT     GPIO_PORT_A
#define PIN_Y8_PIN      GPIO_PIN_08

#define PIN_Y9_PORT     GPIO_PORT_B
#define PIN_Y9_PIN      GPIO_PIN_15

#define PIN_Y10_PORT    GPIO_PORT_B
#define PIN_Y10_PIN     GPIO_PIN_14

#define PIN_Y11_PORT    GPIO_PORT_B
#define PIN_Y11_PIN     GPIO_PIN_13

#define PIN_Y12_PORT    GPIO_PORT_B
#define PIN_Y12_PIN     GPIO_PIN_12

#define PIN_Y13_PORT    GPIO_PORT_B
#define PIN_Y13_PIN     GPIO_PIN_10

#define PIN_Y14_PORT    GPIO_PORT_B
#define PIN_Y14_PIN     GPIO_PIN_02

#define PIN_Y15_PORT    GPIO_PORT_B
#define PIN_Y15_PIN     GPIO_PIN_01

#define PIN_Y16_PORT    GPIO_PORT_B
#define PIN_Y16_PIN     GPIO_PIN_00


/* 模拟开关 AS 使能管脚 AS_EN1 ~ AS_EN4 */
#define AS_EN0_PORT     GPIO_PORT_C
#define AS_EN0_PIN      GPIO_PIN_15

#define AS_EN1_PORT     GPIO_PORT_H
#define AS_EN1_PIN      GPIO_PIN_00

#define AS_EN2_PORT     GPIO_PORT_H
#define AS_EN2_PIN      GPIO_PIN_01
/* 模拟开关通道选择 AS_S1 ~ AS_S3 */
#define AS_S1_PORT      GPIO_PORT_H
#define AS_S1_PIN       GPIO_PIN_02

#define AS_S2_PORT      GPIO_PORT_C
#define AS_S2_PIN       GPIO_PIN_13

#define AS_S3_PORT      GPIO_PORT_C
#define AS_S3_PIN       GPIO_PIN_14

/* 电平枚举 */
typedef enum
{
    PIN_LOW  = 0U,
    PIN_HIGH = 1U
} en_pin_level_t;

/* AS模拟开关通道选择编码 0~7 (S3 S2 S1 三位二进制) */
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

/* 模拟开关芯片编号 1~4 */
typedef enum
{
    AS_CHIP_0 = 0U,
		AS_CHIP_1 = 1U,
		AS_CHIP_2 = 2U,
} en_as_chip_t;

/* 对外API */
void ADC_SAMPLE_Init(void);
uint16_t SampleGetFrame(uint8_t AREA,uint8_t row, uint8_t col, uint8_t buffer[][col]);
uint16_t Adc_ReadFrame(uint8_t *pu8Data);
/* ADC单次采集函数 */

#endif /* __ADC_SAMPLE_H__ */



