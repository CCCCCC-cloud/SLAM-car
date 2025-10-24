/*
 * OLED.h
 *
 *  Created on: Aug 22, 2024
 *      Author: 王滋行
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "main.h"
extern I2C_HandleTypeDef hi2c3;

#define OLED_DEV_ADDRESS 	0x78
#define OLED_CMD_ADDRESS 	0x00
#define OLED_DATA_ADDRESS	0x40

// 内部函数
void _writeCMD(uint8_t cmd);
void _writeDATA(uint8_t data);
void _setPosition(uint8_t x, uint8_t page);
void _getNumPost(unsigned int num, uint8_t length, uint8_t* array);
unsigned int _pow(uint8_t m,uint8_t n);

// 外部函数
// 初始化
void OLED_Init(void);
// 清屏
void OLED_Clear(void);
// 显示字符 占据8列2页
void OLED_ShowChar8X16(uint8_t x, uint8_t page, char charIndex);
// 显示字符 占据6列1页
void OLED_ShowChar6X8(uint8_t x, uint8_t page, char charIndex);
// 显示汉字
void OLED_ShowChinese16X16(uint8_t x, uint8_t page, uint8_t chineseIndex);
// 显示字符串
void OLED_ShowString8X16(uint8_t x, uint8_t page, char* string);
void OLED_ShowString6X8(uint8_t x, uint8_t page, char* string);
// 显示图片
void OLED_ShowIMG(uint8_t x, uint8_t page, uint8_t imgWeight, uint8_t imgHeight);
// 显示数字
void OLED_ShowNum8X16(uint8_t x, uint8_t page, unsigned int num, uint8_t length);
void OLED_ShowNum6X8(uint8_t x, uint8_t page, unsigned int num, uint8_t length);

// 显示float
void OLED_Showdecimal8X16(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len);

#endif /* INC_OLED_H_ */
