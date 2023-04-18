#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"

#define OLED_ADDRERSS 0x78

#define OLED_CMD 0 //写命令
#define OLED_DATA 1 //写数据

/* STM32 I2C 快速模式 */
#define I2C_Speed 100000

/* I2C接口 */
#define OLED_I2C               I2C1
#define OLED_I2C_CLK           RCC_APB1Periph_I2C1

#define OLED_I2C_SCL_PIN       GPIO_Pin_6
#define OLED_I2C_SCL_GPIO_PORT GPIOB
#define OLED_I2C_SCL_GPIO_CLK  RCC_APB2Periph_GPIOB
#define OLED_I2C_SCL_SOURCE    GPIO_PinSource6
#define OLED_I2C_SCL_AF        GPIO_AF_I2C1

#define OLED_I2C_SDA_PIN       GPIO_Pin_7
#define OLED_I2C_SDA_GPIO_PORT GPIOB
#define OLED_I2C_SDA_GPIO_CLK  RCC_APB2Periph_GPIOB
#define OLED_I2C_SDA_SOURCE    GPIO_PinSource7
#define OLED_I2C_SDA_AF        GPIO_AF_I2C1

void I2C_Configuration(void);
void I2C_WriteByte(uint8_t addr, uint8_t data);

//OLED控制用函数
void OLED_WriteByte(uint8_t data, uint8_t cmd);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size);
void OLED_Show_Welcome(unsigned char x, unsigned char y, unsigned char N);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode);
void OLED_DrawHeartRate(s32 *data);

#endif /* __OLED_H */
