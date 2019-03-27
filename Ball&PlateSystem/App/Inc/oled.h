/*****************************************
文 件 名：LED.h
创 建 者：信意电子科技
创建日期：2012/11/08
描    述：LED128x64显示屏底层驱动
------------------------------------------
调用头文件：


 void LED_Init(void);
 void LED_CLS(void); 
 void LED_Set_Pos(uint8 x, uint8 y);//设置坐标函数
 void LED_WrDat(uchar data);   //写数据函数
 void LED_Fill(uint8 dat);
 void LED_P6x8Char(uint8 x,uint8 y,uint8 ch);
 void LED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
 void LED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
 void LED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
 void LED_PXx16MixStr(uint8 x, uint8 y, uint8 ch[]);
 
void LED_PrintBMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]); 
void LED_PrintValueC(uchar x, uchar y,char data);
void LED_PrintValueI(uchar x, uchar y, int data);
void LED_PrintValueF(uchar x, uchar y, float data, uchar num);
void LED_PrintPIC(uchar table[ROW][COLUMN]);
void LED_PrintPIC2(uchar table[ROW][COLUMN]);
void LED_PrintEdge(void);
void LED_Cursor(uchar cursor_column, uchar cursor_row);

*****************************************/

#ifndef __OLED_H__
#define __OLED_H__

//#include "short_w.h"
#include "common.h"
#include "MK60_gpio.h"
#include "include.h"

/*****************************设置数据类型*****************************/


typedef char *pchar;
typedef unsigned char *puchar;
typedef int  *pint;
typedef unsigned int *puint;
/*#define LED_SCL       PORTA_PA0  
#define LED_SDA       PORTA_PA1
#define LED_RST       PORTA_PA2 
#define LED_DC         PORTA_PA3 
#define LED_PORT    DDRA*/

/*****************************************
【功 能 】:初始化引脚定义
【说 明 】:直接在这里改IO就可以了*/

#define SCL_PIN PTC15
#define SDA_PIN PTC19
#define RST_PIN PTD0
#define DC_PIN  PTD1//A0 A3为下载线

#define CS_PIN PTE27

#define INIT 0
#define LED_SCL_Init  gpio_init(SCL_PIN,GPO,INIT) // 时钟初始化定义
#define LED_SDA_Init  gpio_init(SDA_PIN,GPO,INIT)//数据口D1
#define LED_RST_Init  gpio_init(RST_PIN,GPO,INIT)//复位低能电平
#define LED_DC_Init   gpio_init(DC_PIN,GPO,INIT)//偏置常低
#define LED_CS_Init   gpio_init(CS_PIN,GPO,INIT)//偏置常低


#define LED_SCLH  GPIO_PDOR_REG(GPIOX_BASE(SCL_PIN))  |= (1 << PTn(SCL_PIN));// 时钟定义 
#define LED_SCLL  GPIO_PDOR_REG(GPIOX_BASE(SCL_PIN)) &= ~(1 << PTn(SCL_PIN));

#define LED_SDAH  GPIO_PDOR_REG(GPIOX_BASE(SDA_PIN))  |= (1 << PTn(SDA_PIN));//数据口D0
#define LED_SDAL  GPIO_PDOR_REG(GPIOX_BASE(SDA_PIN)) &= ~(1 << PTn(SDA_PIN));

#define LED_RSTH  GPIO_PDOR_REG(GPIOX_BASE(RST_PIN))  |= (1 << PTn(RST_PIN));//复位低能电平
#define LED_RSTL  GPIO_PDOR_REG(GPIOX_BASE(RST_PIN)) &= ~(1 << PTn(RST_PIN));

#define LED_DCH   GPIO_PDOR_REG(GPIOX_BASE(DC_PIN))  |= (1 << PTn(DC_PIN));
#define LED_DCL   GPIO_PDOR_REG(GPIOX_BASE(DC_PIN)) &= ~(1 << PTn(DC_PIN));//偏置常低

extern int img_center[50];				        //提取中点后的位置
extern uint8 left_black[50]; 
extern uint8 right_black[50]; 
/************************************************/

void  LEDPIN_Init(void);   //LED控制引脚初始化
void LED_Init(void);
void LED_CLS(void);
void LED_Set_Pos(uint8 x, uint8 y);//设置坐标函数
void LED_WrDat(uint8 data);   //写数据函数
void LED_P6x8Char(uint8 x,uint8 y,uint8 ch);
void LED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
void LED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
void LED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
void LED_PXx16MixStr(uint8 x, uint8 y, uint8 ch[]);
void LED_PrintBMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]); 
void LED_Fill(uint8 dat);
void LED_PrintValueC(uint8 x, uint8 y,int8 data);
void LED_PrintValueI(uint8 x, uint8 y, int16 data);
void LED_PrintValueF(uint8 x, uint8 y, float data, uint8 num);
void LED_PrintValueFP(uint8 x, uint8 y, uint16 data, uint8 num);
void LED_PrintEdge(void);
void LED_Cursor(uint8 cursor_column, uint8 cursor_row);
void LED_PrintLine(void);
 
 
 
 void img_show(void);
 void team_name(void);
 void look_look(void);

#endif