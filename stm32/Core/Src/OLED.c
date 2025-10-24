
#include "OLED.h"
#include "OLED_FONT.h"


void _writeCMD(uint8_t cmd)
{
	HAL_I2C_Mem_Write(&hi2c3, OLED_DEV_ADDRESS, OLED_CMD_ADDRESS, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0x100);
}
void _writeDATA(uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c3, OLED_DEV_ADDRESS, OLED_DATA_ADDRESS, I2C_MEMADD_SIZE_8BIT, &data, 1, 0x100);
}

void _setPosition(uint8_t x, uint8_t page)
{
	_writeCMD(0x00 | (x & 0x0F));
	_writeCMD(0x10 | ((x & 0xF0) >> 4));
	_writeCMD(0xB0 | page);
}

void _getNumPost(unsigned int num, uint8_t length, uint8_t* array)
{
	for(uint8_t i = 0; i < length; i++)
	{
		array[length - i - 1] = num % 10;
		num = num / 10;
	}
}
unsigned int _pow(uint8_t m,uint8_t n)
{
	unsigned int result=1;
	while(n--)result*=m;
	return result;
}


void OLED_Init(void)
{
	uint8_t CMD_Data[]={
		0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,0xA1, 0xC8, 0xDA,
		0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,0x8D, 0x14,
		0xAF};
	for(uint8_t i = 0; i < 23; i++)
	{
		_writeCMD(CMD_Data[i]);
	}
	OLED_Clear();
}

void OLED_Clear(void)
{
	for(uint8_t j = 0; j < 8; j++)
	{
		_setPosition(0, j);
		for(uint8_t i = 0; i < 128; i++) _writeDATA(0x00);
	}
}

void OLED_ShowChar8X16(uint8_t x, uint8_t page, char charIndex)
{
	uint8_t index = charIndex - ' ';
	_setPosition(x, page);
	for(uint8_t i = 0; i < 8; i++)
	{
		_writeDATA(char8X16[index * 2][i]);
	}
	_setPosition(x, page + 1);
	for(uint8_t i = 0; i < 8; i++)
	{
		_writeDATA(char8X16[index * 2 + 1][i]);
	}
}
void OLED_ShowChar6X8(uint8_t x, uint8_t page, char charIndex)
{
	uint8_t index = charIndex - ' ';
	_setPosition(x, page);
	for(uint8_t i = 0; i < 6; i++)
	{
		_writeDATA(char6X8[index][i]);
	}
}

void OLED_ShowChinese16X16(uint8_t x, uint8_t page, uint8_t chineseIndex)
{
	_setPosition(x, page);
	for(uint8_t i = 0; i < 16; i++)
	{
		_writeDATA(font16X16[chineseIndex * 2][i]);
	}
	_setPosition(x, page + 1);
	for(uint8_t i = 0; i < 16; i++)
	{
		_writeDATA(font16X16[chineseIndex * 2 + 1][i]);
	}
}

void OLED_ShowString8X16(uint8_t x, uint8_t page, char* string)
{
	for(uint8_t index = 0; string[index] != '\0'; index++)
	{
		OLED_ShowChar8X16(x + index * 8, page, string[index]);
	}
}
void OLED_ShowString6X8(uint8_t x, uint8_t page, char* string)
{
	for(uint8_t index = 0; string[index] != '\0'; index++)
	{
		OLED_ShowChar6X8(x + index * 6, page, string[index]);
	}
}

void OLED_ShowIMG(uint8_t x, uint8_t page, uint8_t imgWeight, uint8_t imgHeight)
{
	for(uint8_t i = 0; i < imgHeight; i++)
	{
		_setPosition(x, page + i);
		for(uint8_t j = 0; j < imgWeight; j++)
		{
			_writeDATA(image[i * imgWeight + j]);
		}
	}
}

void OLED_ShowNum8X16(uint8_t x, uint8_t page, unsigned int num, uint8_t length)
{
	uint8_t numArray[length];
	_getNumPost(num, length, &numArray[0]);
	for(uint8_t i = 0; i < length; i++)
	{
		OLED_ShowChar8X16(x + i * 8, page, numArray[i] + '0');
	}
}
void OLED_ShowNum6X8(uint8_t x, uint8_t page, unsigned int num, uint8_t length)
{
	uint8_t numArray[length];
	_getNumPost(num, length, &numArray[0]);
	for(uint8_t i = 0; i < length; i++)
	{
		OLED_ShowChar6X8(x + i * 6, page, numArray[i] + '0');
	}
}


void OLED_Showdecimal8X16(uint8_t x,uint8_t page,float num,uint8_t z_len,uint8_t f_len)
{
	uint8_t t,temp,i=0;
	uint8_t enshow;
 	unsigned int z_temp,f_temp;
	if(num<0)
	{
		
		OLED_ShowChar8X16(x, page, '-');

		x = x + 8;
		num = -num;
	}
	
	z_temp=(unsigned int)num;
	OLED_ShowNum8X16(x, page, z_temp, z_len);
	
	x = x + z_len * 8;
	OLED_ShowChar8X16(x, page, '.');
	
	f_temp=(unsigned int)((num-z_temp)*(_pow(10,f_len)));
	x = x + 8;
	OLED_ShowNum8X16(x, page, f_temp, f_len);
}
