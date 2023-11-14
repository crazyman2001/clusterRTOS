/*
 * Display.c
 *
 *  Created on: Oct 14, 2023
 *      Author: AutoWires
 */
#include "main.h"
#include "cmsis_os.h"
#include "Display.h"
#include "event_groups.h"

//  declare a event grounp handler variable
extern EventGroupHandle_t  xEventGroupDisplay;

unsigned char segChar[] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90, 0x8c, 0x84, 0x86, 0xaf, 0x83, 0x89, 0x87, 0xc2, 0x88, 0xff, 0xbf, 0xa1, 0xfb, 0xc1};
//                                                                                        P/10  A/11  E/12  r/13  b/14  H/15 t/16   G/17 A/18 ' '/19  /20  d/21  i/22  U/23

unsigned char dispBuf[18];
uint16_t speedVarF;
extern uint32_t odoVar, tripVar, volt, soc;
extern uint8_t gGear, parking, gr, gInfo, wnSymbol, bitMotor, bitECU, reGen;
extern int16_t amp2, Soh;
extern volatile int32_t amp;

void delayUs(unsigned int itime)
{
	for(; itime>0; itime--);
}

void HT162x_SendBits(unsigned int dat, unsigned char bits, _Bool abc)
{
  unsigned int mask;
	unsigned char i;
	mask = (abc ? 1 : 1 << (bits-1 ));
  for ( i= bits; i > 0; i--)
  {
    HAL_GPIO_WritePin(SEG_WR_GPIO_Port, SEG_WR_Pin, GPIO_PIN_RESET);//LCD_WR=0;//GPIO_WriteLow(LCD_PORT, LCD_WR);
	//osDelay(1);
    delayUs(10);
    if(dat & mask)
        HAL_GPIO_WritePin(SEG_DATA_GPIO_Port, SEG_DATA_Pin, GPIO_PIN_SET);//LCD_DATA=1;
    else
        HAL_GPIO_WritePin(SEG_DATA_GPIO_Port, SEG_DATA_Pin, GPIO_PIN_RESET);//LCD_DATA=0;
    //osDelay(1);
	HAL_GPIO_WritePin(SEG_WR_GPIO_Port, SEG_WR_Pin, GPIO_PIN_SET);//LCD_WR=1;//GPIO_WriteHigh(LCD_PORT, LCD_WR);
    //osDelay(1);
		if(abc)
			dat >>= 1;
		else
			dat <<= 1;
  }
  //osDelay(1);
  delayUs(10);
}

unsigned char HT162xx_ReadBits(void)
{
	unsigned char retVal, i;
	retVal=0;
	//GPIO_Init(PORTC, (LCD_DATA), GPIO_MODE_IN_PU_NO_IT);
	for(i=0; i<4; i++)
	{
		retVal>>=1;
		HAL_GPIO_WritePin(SEG_RD_GPIO_Port, SEG_RD_Pin, GPIO_PIN_RESET);//LCD_RD=0;//GPIO_WriteLow(LCD_PORT, LCD_RD);
		osDelay(10);
		HAL_GPIO_WritePin(SEG_RD_GPIO_Port, SEG_RD_Pin, GPIO_PIN_SET);//LCD_RD=1;//GPIO_WriteHigh(LCD_PORT, LCD_RD);
		osDelay(10);
		if(HAL_GPIO_ReadPin(SEG_DATA_GPIO_Port, SEG_DATA_Pin))//GPIO_ReadInputPin(LCD_PORT, LCD_DATA))
			retVal|=0x08;
	}
	//GPIO_Init(PORTC, (LCD_DATA), GPIO_MODE_OUT_PP_HIGH_FAST);
	return retVal;
}

void HT162x_Command(unsigned char cmd)
{
	HAL_GPIO_WritePin(SEG_CS_GPIO_Port, SEG_CS_Pin, GPIO_PIN_RESET);//LCD_CS=0;//GPIO_WriteLow(LCD_PORT, LCD_CS);
	//osDelay(1);
	delayUs(10);
  HT162x_SendBits(4, 3, MSB_FORMAT);
  HT162x_SendBits(cmd, 8, MSB_FORMAT);
  HT162x_SendBits(1, 1, MSB_FORMAT);
	HAL_GPIO_WritePin(SEG_CS_GPIO_Port, SEG_CS_Pin, GPIO_PIN_SET);//LCD_CS=1;//GPIO_WriteHigh(LCD_PORT, LCD_CS);
	//osDelay(2);
	delayUs(10);
}

void HT162x_WriteData(unsigned char addr, unsigned int sdata, unsigned char bits)
{
	HAL_GPIO_WritePin(SEG_CS_GPIO_Port, SEG_CS_Pin, GPIO_PIN_RESET);//LCD_CS=0;//GPIO_WriteLow(LCD_PORT, LCD_CS);
	//osDelay(1);
  HT162x_SendBits(5, 3, MSB_FORMAT);
  HT162x_SendBits(addr, 6, MSB_FORMAT);
  HT162x_SendBits(sdata, bits, MSB_FORMAT);//LSB_FORMAT);
	HAL_GPIO_WritePin(SEG_CS_GPIO_Port, SEG_CS_Pin, GPIO_PIN_SET);//LCD_CS=1;//GPIO_WriteHigh(LCD_PORT, LCD_CS);
	//osDelay(1);
	delayUs(10);
}

unsigned char HT162xx_ReadData(unsigned char adr)
{
	unsigned char chr;
	HAL_GPIO_WritePin(SEG_CS_GPIO_Port, SEG_CS_Pin, GPIO_PIN_RESET);//LCD_CS=0;//GPIO_WriteLow(LCD_PORT, LCD_CS);
	osDelay(10);
  HT162x_SendBits(6, 3, MSB_FORMAT);
  HT162x_SendBits(adr, 6, MSB_FORMAT);
	chr = HT162xx_ReadBits();
	osDelay(10);
	HAL_GPIO_WritePin(SEG_CS_GPIO_Port, SEG_CS_Pin, GPIO_PIN_SET);//LCD_CS=1;//GPIO_WriteHigh(LCD_PORT, LCD_CS);
	return chr;
}

void clrDisp(void)
{
	int h;

	HAL_GPIO_WritePin(SEG_CS_GPIO_Port, SEG_CS_Pin, GPIO_PIN_RESET);//LCD_CS=0;//GPIO_WriteLow(LCD_PORT, LCD_CS);
	osDelay(10);
  HT162x_SendBits(5, 3, MSB_FORMAT);
  HT162x_SendBits(0, 6, MSB_FORMAT);
	for(h=12;h<60;h++)
		HT162x_SendBits(0, 4, MSB_FORMAT);//LSB_FORMAT);
	osDelay(10);
	for(h=0; h<18; h++)
		dispBuf[h]=0;

	HAL_GPIO_WritePin(SEG_CS_GPIO_Port, SEG_CS_Pin, GPIO_PIN_SET);//LCD_CS=1;//GPIO_WriteHigh(LCD_PORT, LCD_CS);
}

void fillDisp(void)
{
	int h;

	HAL_GPIO_WritePin(SEG_CS_GPIO_Port, SEG_CS_Pin, GPIO_PIN_RESET);//LCD_CS=0;//GPIO_WriteLow(LCD_PORT, LCD_CS);
	osDelay(10);
  HT162x_SendBits(5, 3, MSB_FORMAT);
  HT162x_SendBits(0, 6, MSB_FORMAT);
	for(h=12;h<60;h++)
	{
		HT162x_SendBits(0xff, 4, MSB_FORMAT);//LSB_FORMAT);
	}
	osDelay(10);

	HAL_GPIO_WritePin(SEG_CS_GPIO_Port, SEG_CS_Pin, GPIO_PIN_SET);//LCD_CS=1;//GPIO_WriteHigh(LCD_PORT, LCD_CS);
}

void updateDisp(unsigned char dno, unsigned char val)
{
	unsigned char  x, b1, b2;
	b1=0;
	b2=0;
	//b3=0;
	x=~segChar[val];
	if(dno==1)	//13,14
	{
		b1=x&0x0f;
		if(x&0x10)
			b2|=0x04;
		if(x&0x20)
			b2|=0x01;
		if(x&0x40)
			b2|=0x02;
		x=dispBuf[6];
		x&=0xf8;
		x|=b2;
		dispBuf[6]=x;
		x=dispBuf[7];
		x&=0xf0;
		x|=b1;
		dispBuf[7]=x;
		HT162x_WriteData(25, dispBuf[6], 4);
		HT162x_WriteData(27, dispBuf[7], 4);
	}
	else if(dno==2)
	{
		if(x&0x01)
			b1|=0x01;
		if(x&0x02)
			b2|=0x01;
		if(x&0x04)
			b2|=0x04;
		if(x&0x08)
			b1|=0x08;
		if(x&0x10)
			b1|=0x04;
		if(x&0x20)
			b1|=0x02;
		if(x&0x40)
			b2|=0x02;
		x=dispBuf[8];
		x&=0xf0;
		x|=b1;
		dispBuf[8]=x;
		x=dispBuf[9];
		x&=0xf8;
		x|=b2;
		dispBuf[9]=x;
		HT162x_WriteData(29, dispBuf[8], 4);
		HT162x_WriteData(31, dispBuf[9], 4);
	}
	else if(dno==3)
	{
		if(x&0x01)
			b1|=0x01;
		if(x&0x02)
			b2|=0x01;
		if(x&0x04)
			b2|=0x08;
		if(x&0x08)
			b1|=0x08;
		if(x&0x10)
			b1|=0x04;
		if(x&0x20)
			b1|=0x02;
		if(x&0x40)
			b2|=0x02;
		x=dispBuf[11];
		x&=0xf0;
		x|=b1;
		dispBuf[11]=x;
		x=dispBuf[12];
		x&=0xf4;
		x|=b2;
		dispBuf[12]=x;
		HT162x_WriteData(35, dispBuf[11], 4);
		HT162x_WriteData(37, dispBuf[12], 4);
	}
	else if(dno==4)
	{
		b2=x&0x0f;
		b2<<=4;
		if(x&0x10)
			b1|=0x80;
		if(x&0x20)
			b1|=0x20;
		if(x&0x40)
			b1|=0x40;
		x=dispBuf[1];
		x&=0x1f;
		x|=b1;
		dispBuf[1]=x;
		x=dispBuf[2];
		x&=0x0f;
		x|=b2;
		dispBuf[2]=x;
		HT162x_WriteData(14, dispBuf[1]>>4, 4);
		HT162x_WriteData(16, dispBuf[2]>>4, 4);
	}
	else if(dno==5)
	{
		b2=x&0x0f;
		b2<<=4;
		if(x&0x10)
			b1|=0x80;
		if(x&0x20)
			b1|=0x20;
		if(x&0x40)
			b1|=0x40;
		x=dispBuf[3];
		x&=0x1f;
		x|=b1;
		dispBuf[3]=x;
		x=dispBuf[4];
		x&=0x0f;
		x|=b2;
		dispBuf[4]=x;
		HT162x_WriteData(18, dispBuf[3]>>4, 4);
		HT162x_WriteData(20, dispBuf[4]>>4, 4);
	}
	else if(dno==6)
	{
		b2=x&0x0f;
		b2<<=4;
		if(x&0x10)
			b1|=0x80;
		if(x&0x20)
			b1|=0x20;
		if(x&0x40)
			b1|=0x40;
		b1>>=1;
		x=dispBuf[5];
		x&=0x8f;
		x|=b1;
		dispBuf[5]=x;
		x=dispBuf[6];
		x&=0x0f;
		x|=b2;
		dispBuf[6]=x;
		HT162x_WriteData(22, dispBuf[5]>>4, 4);
		HT162x_WriteData(24, dispBuf[6]>>4, 4);
	}
	else if(dno==7)
	{
		b2=x&0x0f;
		b2<<=4;
		if(x&0x10)
			b1|=0x80;
		if(x&0x20)
			b1|=0x20;
		if(x&0x40)
			b1|=0x40;
		b1>>=1;
		x=dispBuf[7];
		x&=0x8f;
		x|=b1;
		dispBuf[7]=x;
		x=dispBuf[8];
		x&=0x0f;
		x|=b2;
		dispBuf[8]=x;
		HT162x_WriteData(26, dispBuf[7]>>4, 4);
		HT162x_WriteData(28, dispBuf[8]>>4, 4);
	}
	else if(dno==8)
	{
		b2=x&0x0f;
		b2<<=4;
		if(x&0x10)
			b1|=0x80;
		if(x&0x20)
			b1|=0x20;
		if(x&0x40)
			b1|=0x40;
		b1>>=1;
		x=dispBuf[9];
		x&=0x8f;
		x|=b1;
		dispBuf[9]=x;
		x=dispBuf[10];
		x&=0x0f;
		x|=b2;
		dispBuf[10]=x;
		HT162x_WriteData(30, dispBuf[9]>>4, 4);
		HT162x_WriteData(32, dispBuf[10]>>4, 4);
	}
	else if(dno==11)
	{
		if(x&0x01)
			b2|=0x10;
		if(x&0x02)
			b1|=0x20;
		if(x&0x04)
			b1|=0x80;
		if(x&0x08)
			b2|=0x80;
		if(x&0x10)
			b2|=0x40;
		if(x&0x20)
			b2|=0x20;
		if(x&0x40)
			b1|=0x40;
		x=dispBuf[16];
		x&=0x1f;
		x|=b1;
		dispBuf[16]=x;
		x=dispBuf[17];
		x&=0x0f;
		x|=b2;
		dispBuf[17]=x;
		HT162x_WriteData(44, dispBuf[16]>>4, 4);
		HT162x_WriteData(46, dispBuf[17]>>4, 4);
	}
	else if(dno==12)
	{
		if(x&0x01)
			b2|=0x10;
		if(x&0x02)
			b1|=0x10;
		if(x&0x04)
			b1|=0x40;
		if(x&0x08)
			b2|=0x80;
		if(x&0x10)
			b2|=0x40;
		if(x&0x20)
			b2|=0x20;
		if(x&0x40)
			b1|=0x20;
		x=dispBuf[14];
		x&=0x8f;
		x|=b1;
		dispBuf[14]=x;
		x=dispBuf[15];
		x&=0x0f;
		x|=b2;
		dispBuf[15]=x;
		HT162x_WriteData(40, dispBuf[14]>>4, 4);
		HT162x_WriteData(42, dispBuf[15]>>4, 4);
	}
	else if(dno==13)
	{
		if(x&0x01)
			b2|=0x10;
		if(x&0x02)
			b1|=0x10;
		if(x&0x04)
			b1|=0x40;
		if(x&0x08)
			b2|=0x80;
		if(x&0x10)
			b2|=0x40;
		if(x&0x20)
			b2|=0x20;
		if(x&0x40)
			b1|=0x20;
		x=dispBuf[12];
		x&=0x8f;
		x|=b1;
		dispBuf[12]=x;
		x=dispBuf[13];
		x&=0x0f;
		x|=b2;
		dispBuf[13]=x;
		HT162x_WriteData(36, dispBuf[12]>>4, 4);
		HT162x_WriteData(38, dispBuf[13]>>4, 4);
	}
	else if(dno==14)
	{
		if(x&0x01)
			b2|=0x01;
		if(x&0x02)
			b1|=0x02;
		if(x&0x04)
			b1|=0x08;
		if(x&0x08)
			b2|=0x08;
		if(x&0x10)
			b2|=0x04;
		if(x&0x20)
			b2|=0x02;
		if(x&0x40)
			b1|=0x04;
		x=dispBuf[16];
		x&=0xf1;
		x|=b1;
		dispBuf[16]=x;
		x=dispBuf[17];
		x&=0xf0;
		x|=b2;
		dispBuf[17]=x;
		HT162x_WriteData(45, dispBuf[16], 4);
		HT162x_WriteData(47, dispBuf[17], 4);
	}
	else if(dno==15)
	{
		if(x&0x01)
			b2|=0x01;
		if(x&0x02)
			b1|=0x02;
		if(x&0x04)
			b1|=0x08;
		if(x&0x08)
			b2|=0x08;
		if(x&0x10)
			b2|=0x04;
		if(x&0x20)
			b2|=0x02;
		if(x&0x40)
			b1|=0x04;
		x=dispBuf[14];
		x&=0xf1;
		x|=b1;
		dispBuf[14]=x;
		x=dispBuf[15];
		x&=0xf0;
		x|=b2;
		dispBuf[15]=x;
		HT162x_WriteData(41, dispBuf[14], 4);
		HT162x_WriteData(43, dispBuf[15], 4);
	}
	else if(dno==16)
	{
		if(x&0x01)
			b2|=0x01;
		if(x&0x02)
			b1|=0x80;
		if(x&0x04)
			b1|=0x20;
		if(x&0x08)
			b2|=0x08;
		if(x&0x10)
			b2|=0x04;
		if(x&0x20)
			b2|=0x02;
		if(x&0x40)
			b1|=0x40;
		x=dispBuf[11];
		x&=0x1f;
		x|=b1;
		dispBuf[11]=x;
		x=dispBuf[13];
		x&=0xf0;
		x|=b2;
		dispBuf[13]=x;
		HT162x_WriteData(34, dispBuf[11]>>4, 4);
		HT162x_WriteData(39, dispBuf[13], 4);
	}
}

void drawSymbol(unsigned char sy, unsigned char stat)
{
	unsigned char x,y;
	if(sy==1)												//T1 is replaced with T25
	{
		x=dispBuf[0];
		x&=0x7F;
		if(stat)
			x|=0x80;
		dispBuf[0]=x;
		HT162x_WriteData(12, dispBuf[0]>>4, 4);
	}
	else if(sy==2)
	{
		x=dispBuf[16];
		x&=0xFE;
		if(stat)
			x|=0x01;
		dispBuf[16]=x;
		HT162x_WriteData(45, dispBuf[16], 4);
	}
	else if(sy==3)
	{
		x=dispBuf[3];
		x&=0xF7;
		if(stat)
			x|=0x08;
		dispBuf[3]=x;
		HT162x_WriteData(19, dispBuf[3], 4);
	}
	else if(sy==4)
	{
		x=dispBuf[3];
		x&=0xEF;
		if(stat)
			x|=0x10;
		dispBuf[3]=x;
		HT162x_WriteData(18, dispBuf[3]>>4, 4);
	}
	else if(sy==5)
	{
		x=dispBuf[1];
		x&=0xEF;
		if(stat)
			x|=0x10;
		dispBuf[1]=x;
		HT162x_WriteData(14, dispBuf[1]>>4, 4);
	}
	else if(sy==6)
	{
		x=dispBuf[0];
		x&=0xFE;
		if(stat)
			x|=0x01;
		dispBuf[0]=x;
		HT162x_WriteData(13, dispBuf[0], 4);
	}
	else if(sy==7)
	{
		x=dispBuf[0];
		x&=0xFD;
		if(stat)
			x|=0x02;
		dispBuf[0]=x;
		HT162x_WriteData(13, dispBuf[0], 4);
	}
	else if(sy==8)
	{
		x=dispBuf[0];
		x&=0xEF;
		if(stat)
			x|=0x10;
		dispBuf[0]=x;
		HT162x_WriteData(12, dispBuf[0]>>4, 4);
	}
	else if(sy==9)
	{
		x=dispBuf[12];
		x&=0x7F;
		if(stat)
			x|=0x80;
		dispBuf[12]=x;
		HT162x_WriteData(36, dispBuf[12]>>4, 4);
	}
	else if(sy==10)
	{
		x=dispBuf[11];
		x&=0xEF;
		if(stat)
			x|=0x10;
		dispBuf[11]=x;
		HT162x_WriteData(34, dispBuf[11]>>4, 4);
	}
	else if(sy==11)
	{
		x=dispBuf[16];
		x&=0xEF;
		if(stat)
			x|=0x10;
		dispBuf[16]=x;
		HT162x_WriteData(44, dispBuf[16]>>4, 4);
	}
	else if(sy==12)
	{
		x=dispBuf[0];
		x&=0xBF;
		if(stat)
			x|=0x40;
		dispBuf[0]=x;
		HT162x_WriteData(12, dispBuf[0]>>4, 4);
	}
	else if(sy==13)
	{
		x=dispBuf[0];
		x&=0xDF;
		if(stat)
			x|=0x20;
		dispBuf[0]=x;
		HT162x_WriteData(12, dispBuf[0]>>4, 4);
	}
	else if(sy==14)
	{
		x=dispBuf[9];
		x&=0x7F;
		if(stat)
			x|=0x80;
		dispBuf[9]=x;
		HT162x_WriteData(30, dispBuf[9]>>4, 4);
	}
	else if(sy==15)
	{
		x=dispBuf[6];
		x&=0xF7;
		if(stat)
			x|=0x08;
		dispBuf[6]=x;
		HT162x_WriteData(25, dispBuf[6], 4);
	}
	else if(sy==16)
	{
		x=dispBuf[5];
		x&=0x7F;
		if(stat)
			x|=0x80;
		dispBuf[5]=x;
		HT162x_WriteData(22, dispBuf[5]>>4, 4);
	}
	else if(sy==17)
	{
		x=dispBuf[7];
		x&=0x7F;
		if(stat)
			x|=0x80;
		dispBuf[7]=x;
		HT162x_WriteData(26, dispBuf[7]>>4, 4);
	}
	else if(sy==18)
	{
		x=dispBuf[10];
		x&=0xFE;
		if(stat)
			x|=0x01;
		dispBuf[10]=x;
		HT162x_WriteData(33, dispBuf[10], 4);
	}
	else if(sy==19)
	{
		x=dispBuf[10];
		x&=0xFD;
		if(stat)
			x|=0x02;
		dispBuf[10]=x;
		HT162x_WriteData(33, dispBuf[10], 4);
	}
	else if(sy==20)
	{
		x=dispBuf[14];
		x&=0x7F;
		if(stat)
			x|=0x80;
		dispBuf[14]=x;
		HT162x_WriteData(40, dispBuf[14]>>4, 4);
	}
	else if(sy==21)
	{
		x=dispBuf[9];
		x&=0xF7;
		if(stat)
			x|=0x08;
		dispBuf[9]=x;
		HT162x_WriteData(31, dispBuf[9], 4);
	}
	else if(sy==22)
	{
		x=dispBuf[10];
		x&=0xF7;
		if(stat)
			x|=0x08;
		dispBuf[10]=x;
		HT162x_WriteData(33, dispBuf[10], 4);
	}
	else if(sy==23)
	{
		x=dispBuf[10];
		x&=0xFB;
		if(stat)
			x|=0x04;
		dispBuf[10]=x;
		HT162x_WriteData(33, dispBuf[10], 4);
	}
	else if(sy==24)
	{
		x=dispBuf[4];
		x&=0xFE;
		if(stat)
			x|=0x01;
		dispBuf[4]=x;
		HT162x_WriteData(21, dispBuf[4], 4);
	}
	else if(sy==26)
	{
		x=dispBuf[0];
		x&=0xF7;
		if(stat)
			x|=0x08;
		dispBuf[0]=x;
		HT162x_WriteData(13, dispBuf[0], 4);
	}
	else if(sy==27)
	{
		x=dispBuf[0];
		x&=0xFB;
		if(stat)
			x|=0x04;
		dispBuf[0]=x;
		HT162x_WriteData(13, dispBuf[0], 4);
	}
	else if(sy==28)
	{
		x=dispBuf[1];
		x&=0xFE;
		if(stat)
			x|=0x01;
		dispBuf[1]=x;
		HT162x_WriteData(15, dispBuf[1], 4);
	}
	else if(sy==29)
	{
		x=dispBuf[1];
		x&=0xFD;
		if(stat)
			x|=0x02;
		dispBuf[1]=x;
		HT162x_WriteData(15, dispBuf[1], 4);
	}
	else if(sy==30)
	{
		x=dispBuf[1];
		x&=0xFB;
		if(stat)
			x|=0x04;
		dispBuf[1]=x;
		HT162x_WriteData(15, dispBuf[1], 4);
	}
	else if(sy==31)
	{
		x=dispBuf[1];
		x&=0xF7;
		if(stat)
			x|=0x08;
		dispBuf[1]=x;
		HT162x_WriteData(15, dispBuf[1], 4);
	}
	else if(sy==32)
	{
		x=dispBuf[2];
		x&=0xF7;
		if(stat)
			x|=0x08;
		dispBuf[2]=x;
		HT162x_WriteData(17, dispBuf[2], 4);
	}
	else if(sy==33)
	{
		x=dispBuf[2];
		x&=0xFB;
		if(stat)
			x|=0x04;
		dispBuf[2]=x;
		HT162x_WriteData(17, dispBuf[2], 4);
	}
	else if(sy==34)
	{
		x=dispBuf[2];
		x&=0xFD;
		if(stat)
			x|=0x02;
		dispBuf[2]=x;
		HT162x_WriteData(17, dispBuf[2], 4);
	}
	else if(sy==35)
	{
		x=dispBuf[2];
		x&=0xFE;
		if(stat)
			x|=0x01;
		dispBuf[2]=x;
		HT162x_WriteData(17, dispBuf[2], 4);
	}
	else if(sy==36)
	{
		x=dispBuf[3];
		x&=0xFB;
		if(stat)
			x|=0x04;
		dispBuf[3]=x;
		HT162x_WriteData(19, dispBuf[3], 4);
	}
	else if(sy==37)
	{
		x=dispBuf[3];
		x&=0xFD;
		if(stat)
			x|=0x02;
		dispBuf[3]=x;
		HT162x_WriteData(19, dispBuf[3], 4);
	}
	else if(sy==38)
	{
		x=dispBuf[3];
		x&=0xFE;
		if(stat)
			x|=0x01;
		dispBuf[3]=x;
		HT162x_WriteData(19, dispBuf[3], 4);
	}
	else if(sy==39)
	{
		x=dispBuf[4];
		x&=0xFD;
		if(stat)
			x|=0x02;
		dispBuf[4]=x;
		HT162x_WriteData(21, dispBuf[4], 4);
	}
	else if(sy==40)
	{
		x=dispBuf[4];
		x&=0xFB;
		if(stat)
			x|=0x04;
		dispBuf[4]=x;
		HT162x_WriteData(21, dispBuf[4], 4);
	}
	else if(sy==41)
	{
		x=dispBuf[4];
		x&=0xF7;
		if(stat)
			x|=0x08;
		dispBuf[4]=x;
		HT162x_WriteData(21, dispBuf[4], 4);
	}
	else if(sy==42)
	{
		x=dispBuf[5];
		x&=0xF7;
		if(stat)
			x|=0x08;
		dispBuf[5]=x;
		HT162x_WriteData(23, dispBuf[5], 4);
	}
	else if(sy==43)
	{
		x=dispBuf[5];
		x&=0xFB;
		if(stat)
			x|=0x04;
		dispBuf[5]=x;
		HT162x_WriteData(23, dispBuf[5], 4);
	}
	else if(sy==44)
	{
		x=dispBuf[5];
		x&=0xFD;
		if(stat)
			x|=0x02;
		dispBuf[5]=x;
		HT162x_WriteData(23, dispBuf[5], 4);
	}
	else if(sy==45)				//reverse
	{
		x=dispBuf[12];
		x&=0xFB;
		if(stat)
			x|=0x04;
		dispBuf[12]=x;
		HT162x_WriteData(37, dispBuf[12], 4);
	}
	else if(sy==46)				//nutral for zerro
	{
		x=dispBuf[12];
		y=dispBuf[11];
		x&=0xF0;
		y&=0xF0;
		if(stat)
		{
			x|=0x0C;
			y|=0x04;
		}
		dispBuf[12]=x;
		dispBuf[11]=y;
		HT162x_WriteData(37, dispBuf[12], 4);
		HT162x_WriteData(35, dispBuf[11], 4);
	}
	else if(sy==47)				//E for one
	{
		x=dispBuf[12];
		x&=0xF0;
		if(stat)
			x|=0x02;
		dispBuf[12]=x;
		HT162x_WriteData(37, dispBuf[12], 4);
		x=dispBuf[11];
		x&=0xF0;
		if(stat)
			x|=0x0F;
		dispBuf[11]=x;
		HT162x_WriteData(35, dispBuf[11], 4);
	}
	else if(sy==48)				//F for two
	{
		x=dispBuf[12];
		x&=0xF0;
		if(stat)
			x|=0x02;
		dispBuf[12]=x;
		HT162x_WriteData(37, dispBuf[12], 4);
		x=dispBuf[11];
		x&=0xF0;
		if(stat)
			x|=0x07;
		dispBuf[11]=x;
		HT162x_WriteData(35, dispBuf[11], 4);
	}
	else if(sy==49)				//CRUSE for two
	{
		x=dispBuf[12];
		x&=0xF4;
		/*if(stat)
			x|=0x02;*/
		dispBuf[12]=x;
		HT162x_WriteData(37, dispBuf[12], 4);
		x=dispBuf[11];
		x&=0xF0;
		if(stat)
			x|=0x0F;
		dispBuf[11]=x;
		HT162x_WriteData(35, dispBuf[11], 4);
	}
	else if(sy==50)				//reverse
		{
			x=dispBuf[14];
			x&=0xFE;
			if(stat)
				x|=0x01;
			dispBuf[14]=x;
			HT162x_WriteData(41, dispBuf[14], 4);
		}
	else if(sy==51)				//Boost
			{
				x=dispBuf[14];
				x&=0xFE;
				//if(stat)
					x|=0x01;
				dispBuf[14]=x;
				HT162x_WriteData(41, dispBuf[14], 4);
				updateDisp(3, 11);
			}
	else if(sy==52)				//boost
			{
				drawSymbol(REVERSE,0);
				updateDisp(3, 14);
			}
	else if(sy==53)				//HILL
			{
				drawSymbol(REVERSE,0);
				updateDisp(3, 16);
			}
	else if(sy==54)				//MOTOR ERROR
	{
		x=dispBuf[5];
		x&=0xFE;
		if(stat)
			x|=0x01;
		dispBuf[5]=x;
		HT162x_WriteData(23, dispBuf[5], 4);
	}
	else if(sy==55)				//Parking
	{
		x=dispBuf[12];
		x&=0xFB;
		if(stat)
			x|=0x04;
		dispBuf[12]=x;
		HT162x_WriteData(37, dispBuf[12], 4);
	}
}

void ht1622Init(void)
{
	int l;
	int k;
	HT162x_Command(CMD_SYS_EN);
  HT162x_Command(CMD_RC_INT);
  HT162x_Command(CMD_LCD_OFF);
  HT162xx_ReadData(39);
  clrDisp();
	//fillDisp();
  HT162x_Command(CMD_LCD_ON); // Should turn it back on
  for(k =0; k<25; k++)
	{
	  if(k!=3)
	  {
		drawSymbol(k, 1);
		osDelay(10);
	  }
	}
	for(l=0; l<10; l++)
	{
		if(l==0)
			drawSymbol(3, 1);
		else
		drawSymbol(35+l, 1);
		drawSymbol(26+l, 1);
		updateDisp(0, l);
		updateDisp(1, l);
		updateDisp(2, l);
		updateDisp(3, l);
		updateDisp(4, l);
		updateDisp(5, l);
		updateDisp(6, l);
		updateDisp(7, l);
		updateDisp(8, l);
		updateDisp(11, l);
		updateDisp(12, l);
		updateDisp(13, l);
		updateDisp(14, l);
		updateDisp(15, l);
		updateDisp(16, l);
		osDelay(70);
	}
	osDelay(50);
	for(k =0; k<25; k++)
	{
		if(k!=3)
		{
			drawSymbol(k, 0);
			osDelay(1);
		}
	}
	for(l=0; l<10; l++)
	{
		if(l==0)
		{
			drawSymbol(46, 1);
			updateDisp(3, 10);
		}
		else if((l>0) && (l<5))
		{
			drawSymbol(45+l, 1);
			//osDelay(300);
		}
		else if(l==6)
		{
			updateDisp(3,10);
		}
		else if(l==7)
		{
			drawSymbol(51,1);
			drawSymbol(REVERSE,1);
		}
		else if(l==8)
		{
			drawSymbol(52,1);
		}
		else if(l==9)
		{
			drawSymbol(53,1);
		}
		drawSymbol(26+l, 0);
		if(l == 0)
			drawSymbol(3, 0);
		else
			drawSymbol(35+l, 0);
		updateDisp(0, 0);
		updateDisp(1, 0);
		updateDisp(2, 0);
		//updateDisp(3, 0);
		updateDisp(4, 0);
		updateDisp(5, 0);
		updateDisp(6, 0);
		updateDisp(7, 0);
		updateDisp(8, 0);
		updateDisp(11, 0);
		updateDisp(12, 0);
		updateDisp(13, 0);
		updateDisp(14, 0);
		updateDisp(15, 0);
		updateDisp(16, 0);
		osDelay(70);
	}
  osDelay(200);
  for(uint8_t h=0; h<18; h++)
		dispBuf[h]=0;
}

void gearDisp(char g)
{
	/*if(g==5)									//parking
	{
		drawSymbol(REVERSE,0);
		updateDisp(3,10);
	}
	else*/ if(g==6)							//reverse
	{
		updateDisp(3, 10);
		drawSymbol(REVERSE, 1);
	}
	else
	{
		drawSymbol(REVERSE,0);
		updateDisp(3, g);
	}
}

void speedDisp(unsigned char spd)
{
	unsigned char tmx;
	if(spd<=199)
	{
		if(spd>99)
		{
			drawSymbol(ONE, 1);
			tmx=spd%100;
		}
		else
		{
			drawSymbol(ONE, 0);
			tmx=spd;
		}
		updateDisp(2, tmx%10);
		updateDisp(1, tmx/10);
	}
}

void dispOdo(unsigned long var, _Bool bi)
{
	if(bi)
	{
		drawSymbol(ODO_DECIMAL,1);
		drawSymbol(ODO, 0);
		drawSymbol(TRIP, 1);
		drawSymbol(KM, 1);
	}
	else
	{
		drawSymbol(ODO_DECIMAL,0);
		drawSymbol(TRIP, 0);
		drawSymbol(ODO, 1);
		drawSymbol(KM, 1);
		var/=10;
	}
	if(var>99999)
	{
		uint16_t abc=var/100000;
		var = var-(abc*100000);
			//var--;
	}
	updateDisp(4, var/10000);
	var%=10000;
	updateDisp(5, var/1000);
	var%=1000;
	updateDisp(6, var/100);
	var%=100;
	updateDisp(7, var/10);
	updateDisp(8, var%10);
}

void dispVolt(unsigned int va)
{
    if(va>999)
    {
        va=999;
    }
	updateDisp(11, va/100);
	va%=100;
	updateDisp(12, va/10);
	updateDisp(13, va%10);
}

void dispMm(unsigned int va)
{
    if(va>999)
    {
        va=999;
    }
	updateDisp(14, va/100);
	va%=100;
	updateDisp(15, va/10);
	updateDisp(16, va%10);
}

void battDisp(unsigned char per)
{
	int g;
	if(per==0)
	{
		for(g=26;g<=35;g++)
			drawSymbol(g, 0);
	}
	else if((per>0)&&(per<=10))
	{
		drawSymbol(26, 1);
		for(g=27;g<=35;g++)
			drawSymbol(g, 0);
	}
	else if((per>10)&&(per<=20))//(per==20)
	{
		for(g=26; g<=27; g++)
			drawSymbol(g, 1);
		for(g=28;g<=35;g++)
			drawSymbol(g, 0);
	}
	else if((per>20)&&(per<=30))//(per==30)
	{
		for(g=26; g<=28; g++)
			drawSymbol(g, 1);
		for(g=29;g<=35;g++)
			drawSymbol(g, 0);
	}
	else if((per>30)&&(per<=40))//(per==40)
	{
		for(g=26; g<=29; g++)
			drawSymbol(g, 1);
		for(g=30;g<=35;g++)
			drawSymbol(g, 0);
	}
	else if((per>40)&&(per<=50))//(per==50)
	{
		for(g=26; g<=30; g++)
			drawSymbol(g, 1);
		for(g=31;g<=35;g++)
			drawSymbol(g, 0);
	}
	else if((per>50)&&(per<=60))//(per==60)
	{
		for(g=26; g<=31; g++)
			drawSymbol(g, 1);
		for(g=32;g<=35;g++)
			drawSymbol(g, 0);
	}
	else if((per>60)&&(per<=70))//(per==70)
	{
		for(g=26; g<=32; g++)
			drawSymbol(g, 1);
		for(g=33;g<=35;g++)
			drawSymbol(g, 0);
	}
	else if((per>70)&&(per<=80))//(per==80)
	{
		for(g=26; g<=33; g++)
			drawSymbol(g, 1);
		for(g=34;g<=35;g++)
			drawSymbol(g, 0);
	}
	else if((per>80)&&(per<=90))//(per==90)
	{
		for(g=26; g<=34; g++)
			drawSymbol(g, 1);
		//for(g=31;g<=35;g++)
			drawSymbol(35, 0);
	}
	else if(per>90)
	{
		for(g=26; g<=35; g++)
			drawSymbol(g, 1);
		//for(g=31;g<=35;g++)
			//drawSymbol(g, 0);
	}
}

void dispAcc(unsigned int acc)
{
	unsigned int accStp;
	int g;
	if(acc>0)
	{
		accStp=acc/5;//10  //5//15
		accStp++;
	}
	else
		accStp=acc;
	if(accStp==0)
	{
		drawSymbol(3, 0);
		for(g=36;g<=44;g++)
			drawSymbol(g, 0);
	}
	else if(accStp==1)
	{
		drawSymbol(3, 1);
		for(g=36;g<=44;g++)
			drawSymbol(g, 0);
	}
	else if(accStp==2)
	{
		drawSymbol(3, 1);
		drawSymbol(36, 1);
		for(g=37;g<=44;g++)
			drawSymbol(g, 0);
	}
	else if(accStp==3)
	{
		drawSymbol(3, 1);
		for(g=36; g<=37; g++)
			drawSymbol(g, 1);
		for(g=38;g<=44;g++)
			drawSymbol(g, 0);
	}
	else if(accStp==4)
	{
		drawSymbol(3, 1);
		for(g=36; g<=38; g++)
			drawSymbol(g, 1);
		for(g=39;g<=44;g++)
			drawSymbol(g, 0);
	}
	else if(accStp==5)
	{
		drawSymbol(3, 1);
		for(g=36; g<=39; g++)
			drawSymbol(g, 1);
		for(g=40;g<=44;g++)
			drawSymbol(g, 0);
	}
	else if(accStp==6)
	{
		drawSymbol(3, 1);
		for(g=36; g<=40; g++)
			drawSymbol(g, 1);
		for(g=41;g<=44;g++)
			drawSymbol(g, 0);
	}
	else if(accStp==7)
	{
		drawSymbol(3, 1);
		for(g=36; g<=41; g++)
			drawSymbol(g, 1);
		for(g=42;g<=44;g++)
			drawSymbol(g, 0);
	}
	else if(accStp==8)
	{
		drawSymbol(3, 1);
		for(g=36; g<=42; g++)
			drawSymbol(g, 1);
		for(g=43;g<=44;g++)
			drawSymbol(g, 0);
	}
	else if(accStp==9)
	{
		drawSymbol(3, 1);
		for(g=36; g<=43; g++)
			drawSymbol(g, 1);
		drawSymbol(44, 0);
	}
	else if(accStp>=10)
	{
		drawSymbol(3, 1);
		for(g=36; g<=44; g++)
			drawSymbol(g, 1);
	}
}

void dispInit(void)
{
	clrDisp();
	//HAL_Delay(3000);
	drawSymbol(LOGO, 1);
	drawSymbol(BATERY, 1);
	drawSymbol(PERCENTAGE, 1);
	drawSymbol(KMPH, 1);
	//drawSymbol(KM, 1);
	drawSymbol(GEAR, 1);
	drawSymbol(VOLT, 1);
	drawSymbol(VDOT, 1);
	//drawSymbol(AMP, 1);
	//drawSymbol(MM, 1);
	//drawSymbol(RNG, 1);
}

void StartDisplayTask( void *pvParameters )
{
	EventBits_t xEventGroupValue, bitsSet;
	ht1622Init();
	dispInit();
	bitsSet = (SPEED_BIT | ODO_BIT | TRIP_BIT | RANGE_BIT | GEAR_BIT | VOLT_BIT);
	while(1)
	{
		xEventGroupValue  = xEventGroupWaitBits( xEventGroupDisplay, bitsSet, pdTRUE, pdFALSE, portMAX_DELAY );
		if(xEventGroupValue & SPEED_BIT)
		{
			speedDisp(speedVarF);
			xEventGroupClearBits(xEventGroupDisplay, SPEED_BIT);
		}
		if(xEventGroupValue & ODO_BIT)
		{
			dispOdo(odoVar, 0);
			xEventGroupClearBits(xEventGroupDisplay, ODO_BIT);
		}
		if(xEventGroupValue & TRIP_BIT)
		{
			dispOdo(tripVar, 1);
			xEventGroupClearBits(xEventGroupDisplay, TRIP_BIT);
		}
		if(xEventGroupValue & RANGE_BIT)
		{
			dispMm(soc);
			xEventGroupClearBits(xEventGroupDisplay, RANGE_BIT);
		}
		if(xEventGroupValue & VOLT_BIT)
		{
			dispVolt(volt);
			xEventGroupClearBits(xEventGroupDisplay, VOLT_BIT);
		}
		if(xEventGroupValue & PARK_BIT)
		{
			drawSymbol(REVERSE,0);
			updateDisp(2,10);
			updateDisp(1,20);
			gInfo|=0x01;
			xEventGroupClearBits(xEventGroupDisplay, PARK_BIT);
		}
		if(xEventGroupValue & REV_BIT)
		{
			updateDisp(3, 10);
			drawSymbol(REVERSE, 1);
			gInfo|=0x10;
			xEventGroupClearBits(xEventGroupDisplay, REV_BIT);
		}
		if(xEventGroupValue & CURISE_BIT)
		{
			drawSymbol(CRUISE,1);
			gInfo|=0x20;
			xEventGroupClearBits(xEventGroupDisplay, CURISE_BIT);
		}
		if(xEventGroupValue & ECHO_BIT)
		{
			drawSymbol(ECHO, 1);
			gInfo|=0x40;
			xEventGroupClearBits(xEventGroupDisplay, ECHO_BIT);
		}
		if(xEventGroupValue & GEAR_BIT)
		{
			gearDisp(gr);
			xEventGroupClearBits(xEventGroupDisplay, GEAR_BIT);
		}
		if(xEventGroupValue & WARN_BIT)
		{
			drawSymbol(WARN, 1);
			gInfo|=0x08;
			xEventGroupClearBits(xEventGroupDisplay, WARN_BIT);
		}
		if(xEventGroupValue & THROTLE_BIT)
		{
			drawSymbol(THROTLE, 1);
			wnSymbol|=0x01;
			xEventGroupClearBits(xEventGroupDisplay, THROTLE_BIT);
		}
		else
		{
			drawSymbol(THROTLE, 0);
			wnSymbol&=0xFE;
		}
		if(xEventGroupValue & MOTOR_BIT)
		{
			bitMotor = 1;
			drawSymbol(MOTOR, 1);
			wnSymbol|=0x02;
			xEventGroupClearBits(xEventGroupDisplay, MOTOR_BIT);
		}
		else
		{
			bitMotor = 0;
			drawSymbol(MOTOR, 0);
			wnSymbol&=0xFD;
		}
		if(xEventGroupValue & SIDE_BIT)
		{
			drawSymbol(WARN, 1);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			xEventGroupClearBits(xEventGroupDisplay, SIDE_BIT);
		}
		else
		{
			drawSymbol(WARN, 0);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

		}
		if(xEventGroupValue & BAT_DEAD_BIT)
		{
			drawSymbol(BAT_DEAD, 1);
			//drawSymbol(ECU, 1);
			wnSymbol|=0x08;
			xEventGroupClearBits(xEventGroupDisplay, BAT_DEAD_BIT);
		}
		else
		{
			drawSymbol(BAT_DEAD, 0);
			wnSymbol&=0xF7;
			//drawSymbol(ECU, 0);
		}
		if(xEventGroupValue & ECU_BIT)
		{
			//drawSymbol(ECU, 1);
			wnSymbol|=0x10;
			xEventGroupClearBits(xEventGroupDisplay, ECU_BIT);
		}
		else
		{
			wnSymbol&=0xEF;
				//drawSymbol(ECU, 0);
		}
		if(xEventGroupValue & SOC_DISP_BIT1)
		{
    		dispMm(soc);
    		soc/=10;
    	   	soc++;
    	   	battDisp(soc*10);
    	   	if(soc<=2)
			{
    	  		drawSymbol(BAT_DEAD, 1);
			}
    		else
			{
    			drawSymbol(BAT_DEAD, 0);
			}
			xEventGroupClearBits(xEventGroupDisplay, SOC_DISP_BIT1);
		}
		if(xEventGroupValue & SOC_DISP_BIT2)
		{
    		dispMm(soc);
    		soc/=10;
    	   	soc++;
    	   	battDisp(soc*10);
    		if(soc<=2)
    		{
    			drawSymbol(BAT_DEAD, 1);
    			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
    		}
    		else
    		{
    			drawSymbol(BAT_DEAD, 0);
    			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
    		}
			xEventGroupClearBits(xEventGroupDisplay, SOC_DISP_BIT2);
		}
		if(xEventGroupValue & SOC_DISP_BIT3)
		{
			if (soc<=10)
			{
				updateDisp(14, 20);
				updateDisp(15, 20);
				updateDisp(16, 20);
			}
			else
				dispMm(soc);
    		soc/=10;
    	   	soc++;
    	   	battDisp(soc*10);
    		if(soc<=2)
    		{
    			drawSymbol(BAT_DEAD, 1);
    		}
    		else
    		{
    			drawSymbol(BAT_DEAD, 0);
    		}
			xEventGroupClearBits(xEventGroupDisplay, SOC_DISP_BIT2);
		}
		if(xEventGroupValue & AMP_BIT1)
		{
			if(amp2<=0)    //amp<//0
			{
			   	amp2=-amp2;
			   	dispAcc(amp2/10);
			   	reGen=0;
			}
			else
			{
				reGen=1;
				drawSymbol(POWER, 1);
				dispAcc(amp2/10);
		   	}
			if(reGen)
				drawSymbol(POWER, 1);
			else
				drawSymbol(POWER, 0);
			xEventGroupClearBits(xEventGroupDisplay, AMP_BIT1);
		}
		if(xEventGroupValue & AMP_BIT2)
		{
	    	if(amp2<=0)
	   		{
	    		amp2=-amp2;
	    		drawSymbol(POWER, 0);
	   		}
	    	else
	    	{
	    		drawSymbol(POWER, 1);
	    	}
			amp2/=100;
			dispAcc(amp2);
			xEventGroupClearBits(xEventGroupDisplay, AMP_BIT2);
		}
		/*if(xEventGroupValue & AMP_BIT3)
		{
			if(amp2<=0)    //amp<//0
	   		{
	    		amp2=-amp2;
	    		dispAcc(amp2/10);
	    		drawSymbol(POWER, 0);
	   		}
	   		else
	   		{
	    		drawSymbol(POWER, 1);
	    		dispAcc(amp2/10);
	   		}
			xEventGroupClearBits(xEventGroupDisplay, AMP_BIT3);
		}
		if(xEventGroupValue & AMP_BIT4)
		{
	    	if(amp2<=0)    //amp<//0
	   		{
	    		amp2=-amp2;
	    		dispAcc(amp2/100);
	    		drawSymbol(POWER, 0);
	   		}
	   		else
	   		{
	    		dispAcc(amp2/100);
	   		}
			xEventGroupClearBits(xEventGroupDisplay, AMP_BIT4);
		}
		if(xEventGroupValue & AMP_BIT5)
		{
			if(amp2<0)
			{
				amp2=-amp2;
				dispAcc(amp2/10);
				drawSymbol(POWER, 1);
			}
			else
			{
				drawSymbol(POWER, 0);
				dispAcc(amp2/10);
			}
			xEventGroupClearBits(xEventGroupDisplay, AMP_BIT5);
		}
		if(xEventGroupValue & AMP_BIT6)
		{
			dispAcc(amp2/10);
			xEventGroupClearBits(xEventGroupDisplay, AMP_BIT6);
		}
		if(xEventGroupValue & VOLT_BIT1)
		{
			dispVolt(volt);
			xEventGroupClearBits(xEventGroupDisplay, VOLT_BIT1);
		}
		if(xEventGroupValue & AMP_BIT7)
		{
	    	if(amp>0)
	    	{
	    		dispAcc(amp/100);
	    	}
	  		else
	  		{
	  			amp=-amp;
	  			dispAcc(amp/100);
	  		}
			xEventGroupClearBits(xEventGroupDisplay, AMP_BIT7);
		}
		if(xEventGroupValue & AMP_BIT8)
		{
			if(amp<=0)
	   		{
	    		amp=-amp;
	    		drawSymbol(POWER, 0);
	   		}
	    	else
	    	{
	    		drawSymbol(POWER, 1);
	    	}
			amp/=1000;
			dispAcc(amp);
			xEventGroupClearBits(xEventGroupDisplay, AMP_BIT8);
		}
		if(xEventGroupValue & AMP_BIT9)
		{
	    	if(amp<=0)    //amp<//0
			{
				amp=-amp;
				drawSymbol(POWER, 0);
				dispAcc(amp/100);
				//dispMm(amp);
			}
			else
			{
				drawSymbol(POWER, 1);
				dispAcc(amp/100);
			}
			xEventGroupClearBits(xEventGroupDisplay, AMP_BIT9);
		}*/
		xEventGroupValue = 0;
	}
}
