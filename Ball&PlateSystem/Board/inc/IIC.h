#include "common.h"

#define L3G_SDA_IN()  {gpio_ddr (PTC11, GPI);}     //上拉输入
#define L3G_SDA_OUT() {gpio_ddr (PTC11, GPO);}     //输出

#define L3G_SCL_H      gpio_set (PTC10, 1); //SCL//D8
#define L3G_SCL_L      gpio_set (PTC10, 0); //SCL
#define L3G_SDA_H      gpio_set (PTC11, 1); //SDA输出//D10	 
#define L3G_SDA_L      gpio_set (PTC11, 0); //SDA输出	
#define L3G_R_SDA      gpio_get (PTC11);  //SDA输入



extern void IIC_Init(void);
extern void IIC_Start(void);
extern void IIC_Stop(void);
extern void Receiveack(void);
extern void IIC_Ack(void);
extern void IIC_NAck(void);
extern void IIC_Send_Byte(uint8 txd);
extern uint8 IIC_Read_Byte(void);
extern void IIC_Write(uint8 SlaveAddress,uint8 addr,uint8 dat);
extern uint8 IIC_Read(uint8 SlaveAddress,uint8 addr) ;
extern void IIC_Read_n_byte(uint8 device,uint8 addr,uint8 n,uint8 *buf) ;    //读n个数据

