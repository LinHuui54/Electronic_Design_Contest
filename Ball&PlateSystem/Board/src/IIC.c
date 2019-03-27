#include"IIC.h"
#include "common.h"
#include "include.h"


void I2C_delay(void)
{
   
}

void IIC_Init(void)        //I2C初始化   
{           
            port_init_NoALT(PTC10,ODO| PULLUP );
            port_init_NoALT(PTC11,ODO| PULLUP);
            gpio_init(PTC10,GPO,1); 
            gpio_init(PTC11,GPO,1); 
           
	//    SDA_H;
	//    SCL_H;
}

void IIC_Start(void)      //产生开始信号
{
	   L3G_SDA_OUT();     //SDA转换为输出
	   L3G_SDA_H;		
	   L3G_SCL_H;
	   I2C_delay();
           L3G_SDA_L;//START:when CLK is high,DATA change form high to low 
	   I2C_delay();
}	  

void IIC_Stop(void)       //产生停止信号
{
	   L3G_SDA_OUT();      //SDA转换为输入
           L3G_SCL_L;
           I2C_delay();
           L3G_SDA_L;
 	   I2C_delay();
           L3G_SCL_H;
           I2C_delay();
	   L3G_SDA_H;          //发送IIC总线结束信号
	   I2C_delay();							   	
}

void Receiveack(void)     //接收应答
{
           uint8 i=0,j;
           L3G_SDA_IN();      //SDA转换为输入
	   L3G_SCL_H;
	   I2C_delay();	 
           j=L3G_R_SDA;
	while(j&&(i<10))
	  {
	     I2C_delay();
             j=L3G_R_SDA;
	     i++;
	  }
   	  L3G_SCL_L;           //时钟输出0	   
	  I2C_delay();	  
} 


void IIC_Ack(void)        //产生ACK应答
{
	L3G_SDA_OUT();
        L3G_SCL_L; 
        I2C_delay();
	L3G_SDA_L;
	I2C_delay();
	L3G_SCL_H;
	I2C_delay();
	L3G_SCL_L;
	I2C_delay();
}
   
void IIC_NAck(void)                //不产生ACK应答	 
{
	L3G_SDA_OUT();
        L3G_SCL_L; 
        I2C_delay();
	L3G_SDA_H;
	I2C_delay();
	L3G_SCL_H;
	I2C_delay();
	L3G_SCL_L;
	I2C_delay();
}					 				     
		  

void IIC_Send_Byte(uint8 txd)         //发送数据
{                        
    uint8 t;   
	  L3G_SDA_OUT(); 	    
    for(t=0;t<8;t++)
         {   
		L3G_SCL_L; 	
		I2C_delay(); 
                if((txd&0x80)>>7) 
                { L3G_SDA_H;}
                else  
                { L3G_SDA_L;}     
                txd<<=1; 	  
		I2C_delay();   //必须延时
	     	L3G_SCL_H;
		I2C_delay(); 
         }	 
		L3G_SCL_L;
//		I2C_delay();
//		SDA_H;
//		DELAY_US(6);	
} 	    


uint8 IIC_Read_Byte(void)             //接收数据
{
	      uint8 i,j,receive=0;
	      L3G_SDA_IN();             ////SDA转换为输入
        for(i=0;i<8;i++ )
	  {
            L3G_SCL_L;
            I2C_delay();
            L3G_SCL_H;
	    I2C_delay();
            receive<<=1;
            j=L3G_R_SDA;
            if(j)receive++;   
//	    I2C_delay();
            
          }	
            L3G_SCL_L;
//	    I2C_delay();
        return receive ;
}

void IIC_Write(uint8 SlaveAddress,uint8 addr,uint8 dat)     //写数据
{
              IIC_Start();
	      IIC_Send_Byte(SlaveAddress) ;
	      Receiveack() ;
	      IIC_Send_Byte(addr) ;
	      Receiveack() ;
	      IIC_Send_Byte(dat) ;
	      Receiveack() ;
	      IIC_Stop();      
}


uint8 IIC_Read(uint8 SlaveAddress,uint8 addr)               //读数据
{
               uint8 read_data;
	       IIC_Start();
	       IIC_Send_Byte(SlaveAddress) ;
	       Receiveack() ;
	       IIC_Send_Byte(addr) ;
	       Receiveack() ;
	       IIC_Start();
	       IIC_Send_Byte((SlaveAddress+1)) ;
	       Receiveack() ;
	       read_data=IIC_Read_Byte();
               IIC_NAck();
               IIC_Stop();  
               return  read_data ;	
}


void IIC_Read_n_byte(uint8 device,uint8 addr,uint8 n,uint8 *buf)     //读n个数据
{
	       uint8 i;
               IIC_Start(); 
	       IIC_Send_Byte(device) ;
	       Receiveack() ;
	       IIC_Send_Byte(addr) ;
	       Receiveack() ;
	       IIC_Start();
	       IIC_Send_Byte((device+1)) ;
	       Receiveack() ;
	       for(i=0;i<n;i++)
	         {
					     buf[i]= IIC_Read_Byte();  
						   if(i==n-1)
							 {
							     IIC_NAck();
							 }
							 else
							 {
							     IIC_Ack();
							 } 
		}
					 IIC_Stop();
     
}
