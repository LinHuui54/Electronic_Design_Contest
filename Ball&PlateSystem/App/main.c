/*
 * AUTHOR:		��Ʒ�
 *
 * FUNCTION:	        ���ϳ���
 *
 * NOTICE:		NONE
 *
 * DATE:		2017��8��
 */

#include "include.h"
#include "common.h"

#define   center    (CAMERA_W/2)    //ͼ������
#define   L_limit   (CAMERA_W-75)   //ͼ����߽�
#define   R_limit   (CAMERA_W-5)    //ͼ���ұ߽�
#define   T_limit   (0)           //ͼ�񶥱߽�
#define   B_limit   (59)

//#define   P_X       20
#define   D_X         0
//#define   P_Y       20
#define   D_Y         0
//***************************�������궨��*************************//
//��ֵ 13790      �ϼ��� 12310    �¼��� 15270
#define s3010_x_Init()			ftm_pwm_init(FTM3,FTM_CH3,100,13790)	
#define s3010_x_Duty(duty)		ftm_pwm_duty(FTM3,FTM_CH3,duty)
//��ֵ 13600     �ϼ��� 12180  �¼��� 15140
#define s3010_y_Init()			ftm_pwm_init(FTM3,FTM_CH7,100,13600)	
#define s3010_y_Duty(duty)		ftm_pwm_duty(FTM3,FTM_CH7,duty)
//****************************��������******************************************
void Init_All();					//ȫ�ֳ�ʼ��
void get_ball_xy();
void beep();
void led_shine(uint8 i);
void control_ball();
int PD_X();
int PD_Y();
void PORTA_IRQHandler();                                //portA�ж�����
void DMA0_IRQHandler();                                 //DMA�ж�����
void PIT0_IRQHandler(void);
//******************************************************************************
uint8 imgbuff[CAMERA_SIZE];                             //�洢����ͼ�������
uint8 img[CAMERA_H][CAMERA_W];                          //��ѹ�����ͼ������飬����ͼ����
//�洢С������
int ball_x=0;    //x��Χ0~79
int ball_y=0;    //y��Χ59~0
int last_ball_x=0;    
int last_ball_y=0;
int ball_xx=0;
int ball_yy=0;
double P_X=0;
double P_Y=0;
double I_X=1,I_Y=1;

//С��Ŀ������
int ball_target_x=3;
int ball_target_y=26;

int titlex[11]={0,25,25, 23, 1,0, -1,-23,-25,-25};
int titley[11]={0,23,-1,-25,25,0,-25,25,  1, -23};

int show_data[6];
double s3010_y_derror[2]={0};	
double s3010_x_derror[2]={0};
int last_s3010_out_x=0;
int last_s3010_out_y=0;
float OutData[5] = {0};
double sum_derror_x,sum_derror_y;
uint8 update_flag=0;
int real_x,real_y;
int x_duty=13790,y_duty=12600;//�����ֵ
uint8 mode=0;  //mode1  ����ƽ��
uint8 key_data;
uint8 P_xflag=0,P_yflag=0;
int speed_flag=0;
int last_xx,last_yy;
uint8 I_flag;
int ball_xlocation[4]={0};
int ball_ylocation[4]={0};
uint8 check_speed=0;
uint8 run_flag=0;
int second_count=0;
uint8 target_flag=0;
uint8 project=0;
double data=0;
uint8 data1=3;
int stop_car_flag=0;
uint8 flag1=0;
uint8 flag2=0;
//****************************ȫ�ֳ�ʼ����**************************************
void Init_All()
{

    key_init(KEY_U);
    key_init(KEY_D);
    key_init(KEY_L);
    key_init(KEY_R);
    key_init(KEY_A);
    key_init(KEY_B);
    key_init(KEY_START);
    key_init(KEY_STOP);
    key_init(KEY_C);   
    //��ʼ������ͷ
    camera_init(imgbuff);				    //�趨  imgbuff Ϊ�ɼ�������
    //�����жϷ�����
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //����PORTA���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //����DMA0���жϷ�����Ϊ DMA0_IRQHandler
    //��ʱ��0
    pit_init_ms(PIT0, 1);                                   //��ʼ��PIT0����ʱʱ��Ϊ�� 1ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϸ�λ����Ϊ PIT0_IRQHandler                                 
    LED_Init();
    s3010_x_Init();
    s3010_y_Init();
    led_init(LED0);
    led_init(LED1);
    led_init(LED2);
    led_init(LED3);
    enable_irq (PIT0_IRQn);
}

void  main(void)
{
    Init_All();
    while(1) 
    {
        //��Ŀ2
        if(key_get(KEY_B) ==  KEY_DOWN)
        { ball_target_x=titlex[5];ball_target_y=titley[5];project=2;}
        //��Ŀ3
        if(key_get(KEY_START) ==  KEY_DOWN&&!flag1)
        {ball_target_x=titlex[4];ball_target_y=titley[4];project=3;flag1=1;}
        //��Ŀ4
        if(key_get(KEY_STOP) ==  KEY_DOWN&&!flag2)
        {ball_target_x=8;ball_target_y=-8;project=4;data1=70;data=70;flag2=1;}
        
        camera_get_img(); 				    	  //����ͷ��ȡͼ��
        img_extract((uint8 *)img,(uint8 *)imgbuff,CAMERA_SIZE);   //ͼ���ѹ 				    	  
        LED_PrintValueI(80,0,ball_x);
        LED_PrintValueI(80,1,ball_y);
        img_show();                                               //OLDE��ʾͼ��
        get_ball_xy();
        if(update_flag>=25)
        {
          if(ball_x&&ball_y)
            control_ball();
          update_flag=0;
        }
        LED_PrintValueI(80,2,real_x);
        LED_PrintValueI(80,3,real_y);
        LED_PrintValueI(80,4,x_duty);
        LED_PrintValueI(80,5,y_duty);
        LED_PrintValueI(80,6,project);  
        if(mode==0)
        {
          if(key_get(KEY_U) ==  KEY_DOWN)
            x_duty+=10;
          if(key_get(KEY_D) ==  KEY_DOWN)
            x_duty-=10;
          s3010_x_Duty(x_duty);
          if(key_get(KEY_L) ==  KEY_DOWN)
             y_duty+=10;
          if(key_get(KEY_R) ==  KEY_DOWN)
             y_duty-=10;
          s3010_y_Duty(y_duty);
       }
       if(key_check(KEY_A) ==  KEY_DOWN&&key_data==0)
       {
          mode=1;
          key_data=1;
          led_shine(0);
       }
       //�ٶ��жϣ�������ж�Ϊ�� ���� ����Ŀ�����꣬��������
       if(check_speed)
       {
         check_speed=0;
         uint8 i=0;
         //�ٶ��ж�
         for(i=1;i<4;i++)
         {
           if(((ball_xlocation[i]-ball_xlocation[0])*(ball_xlocation[i]-ball_xlocation[0])+(ball_ylocation[i]-ball_ylocation[0])*(ball_ylocation[i]-ball_ylocation[0]))>=10)
           {  
             I_flag=0;        //moving
             sum_derror_x=0;
             sum_derror_y=0;
           }
         }
         //λ���ж�
         for(i=0;i<4;i++)
         {
           if(((ball_xlocation[i]-ball_target_x)*(ball_xlocation[i]-ball_target_x)+(ball_ylocation[i]-ball_target_y)*(ball_ylocation[i]-ball_target_y))<=data1)
           {
             I_flag=0;        //moving
             sum_derror_x=0;
             sum_derror_y=0;
             led_shine(2);
             second_count++;
             if(stop_car_flag<5000)
               stop_car_flag++;
           }
           else
           {
             second_count=0;
             led_shine(3);
           }
         }
       }
       if(speed_flag>=1000)
       {
         speed_flag=0;
         if(ball_x&&ball_y)
         {
           //С��ͣס
           if(((last_xx-real_x)*(last_xx-real_x)+(last_yy-real_y)*(last_yy-real_y))<=10)
           {
             I_flag=200;
           }
           else
           {
             I_flag=0;//moving
             sum_derror_x=0;
             sum_derror_y=0;
           }
           last_xx=real_x;
           last_yy=real_y;
         }
       }
       if(second_count>7500&&second_count<8000&&!target_flag&&project==3)
       {
         target_flag=1;
         second_count=9000;
         
         ball_target_x=titlex[5];
         ball_target_y=titley[5];
         
         s3010_y_Duty(11500);
         DELAY_MS(500);
         s3010_y_Duty(14500);
         DELAY_MS(200);
         s3010_y_Duty(10500);
         DELAY_MS(200);
         s3010_y_Duty(15700);
       }
       if(stop_car_flag>=1000&&project==4)
       {
         ball_target_x=-23;
         ball_target_y=-24;
         data=100;
         data1=10;
       }
    }
}

void PIT0_IRQHandler(void)
{   
    update_flag++;
    speed_flag++;
    if(second_count>=4&&second_count<8000)
      second_count++;
    if(I_flag)
      I_flag-=5;
    PIT_Flag_Clear(PIT0);                      //���жϱ�־λ
}

void control_ball()
{
  if(mode)
  {
    P_X+=0.24;//0.12
    P_Y+=0.24;
    if(P_X>=90) P_X=90;
    if(P_Y>=90) P_Y=90;
  }
  
  //С������ת��
  ball_x=ball_x+2;//У��С������
  ball_xx=ball_x-41;
  ball_yy=31-ball_y;
  real_x=ball_xx;
  real_y=ball_yy;
  
  ball_xlocation[3]=ball_xlocation[2];
  ball_xlocation[2]=ball_xlocation[1];
  ball_xlocation[1]=ball_xlocation[0];
  ball_xlocation[0]=real_x;
  
  ball_ylocation[3]=ball_ylocation[2];
  ball_ylocation[2]=ball_ylocation[1];
  ball_ylocation[1]=ball_ylocation[0];
  ball_ylocation[0]=real_y;
  
  check_speed=1;
  
  if(mode==1)
  {
    s3010_x_Duty(PD_X());
    s3010_y_Duty(PD_Y());
  }  
}

int PD_X()
{
   int s3010_out=0;
   int s3010_count=0;
  		
   double derror=0;
   derror=(double)(ball_xx-ball_target_x);//��ƫ��(�ֱ���x��yƫ��)
   show_data[0]=derror;
   s3010_x_derror[1]=s3010_x_derror[0];
   s3010_x_derror[0]=derror;
   
   
    //if(P_X>=80)
    //P_X=80;
     if(((ball_yy-ball_target_y)*(ball_yy-ball_target_y)+(derror*derror))<=150)
     { 
       if(P_X>=(150-data))   P_X=150-data;
       if(P_Y>=(150-data))   P_Y=150-data;
       
       if(((ball_yy-ball_target_y)*(ball_yy-ball_target_y)+(derror*derror))>=100)
        {
           if(P_X>=(120-data))   P_X=120-data;
           if(P_Y>=(120-data))   P_Y=120-data;
        }
     }
     else
     {
       if(P_X>=(90-data)) P_X=90-data;
       if(P_Y>=(90-data)) P_Y=90-data;       
     }  
     
   if(I_flag)
   {
     led_shine(1);
     if(((ball_yy-ball_target_y)*(ball_yy-ball_target_y)+(derror*derror))<=80)
      sum_derror_x+=60*derror;//50
     else
      sum_derror_x+=10*derror;
   }
   s3010_count=(int)((P_X)*s3010_x_derror[0] + (D_X)*(s3010_x_derror[0]-s3010_x_derror[1])+I_X*sum_derror_x);
   
   s3010_out=x_duty+s3010_count;
   //�˲�
   //s3010_out=(int)(s3010_out*0.3+last_s3010_out_x*0.7);
   
   last_s3010_out_x=s3010_out;
   
   if(s3010_out>17300) s3010_out=17300;
   if(s3010_out<10000) s3010_out=10000;
   show_data[2]=s3010_out;
   return s3010_out;
}

int PD_Y()
{
   int s3010_out=0;
   int s3010_count=0;
 
   double derror=0;
   //double P_Y=0;
   derror=(double)(ball_yy-ball_target_y);//��ƫ��(�ֱ���x��yƫ��)
   show_data[1]=derror;
   s3010_y_derror[1]=s3010_y_derror[0];
   s3010_y_derror[0]=derror;
  
   if(I_flag)
   {
     led_shine(1);
     if(((ball_xx-ball_target_x)*(ball_xx-ball_target_x)+(derror*derror))<=80)
      sum_derror_y+=50*derror;
     else
      sum_derror_y+=10*derror;
   }
   //if(P_Y>=80)
      //P_Y=80;
   show_data[5]=P_Y;
   s3010_count=(int)((P_Y)*s3010_y_derror[0] + (D_Y)*(s3010_y_derror[0]-s3010_y_derror[1])+I_Y*sum_derror_y);
   
   s3010_out=y_duty-s3010_count;
   //�˲� 
   //s3010_out=(int)(s3010_out*0.3+last_s3010_out_y*0.7);
   last_s3010_out_y=s3010_out;
   
   if(s3010_out>15280) s3010_out=15280;
   if(s3010_out<10320) s3010_out=10320;
   show_data[3]=s3010_out;
   return s3010_out;
}

void get_ball_xy()
{
  uint8 i=0,n=0,z=0;
  uint8 ball_width=0;
  uint8 white_width=0;
  //ȫ�ֱ������㣡
  ball_x=0;
  ball_y=0;
  for(i=(B_limit);i>(T_limit);i--)
  {
    for(n=L_limit;n<R_limit;n++)
    {
        //������㣬�װ׵���
        if(img[i][n]==0xff&&img[i][n+1]==0xff&&img[i][n+2]==0)
        {
          ball_x=n+2;
          ball_y=i;
          break;
        }
    }
    //ɨ��С����
    if(ball_x&&ball_y)
    {
      for(z=ball_x;z<(ball_x+10);z++)
      {
        //ɨ���߽��˳�
        if(z>R_limit)
        {
          ball_x=0;
          ball_y=0;
          ball_width=0;
          break;
        }        
        if(img[i][z]==0)
          ball_width++;
        else
        {
          ball_x=0;
          ball_y=0;
          ball_width=0;
          break;
        }
        //ɨ��С���ȣ��˳�
        if(ball_width>=4)
          break;
      }
    }
    if(ball_width>=4)
    {
      for(;z<(ball_x+6);z++)
      {
        //ɨ���߽��˳�������
        if(z>R_limit)
        {
          ball_x=0;
          ball_y=0;
          ball_width=0;
          break;
        }
        if(img[i][z+1]==0xff)
          white_width++;
       //ɨ����ɫ�����ȣ��˳�
        if(white_width>3)
          break;
      }
    }
    else
    {
      ball_x=0;
      ball_y=0;
      ball_width=0;
    }
    if(white_width>3)
    {
      break;
    }
  }
}

void PORTA_IRQHandler()
{
    uint8  n = 0;    					//���ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
}

void DMA0_IRQHandler()
{
    camera_dma();
}

void beep()
{
  gpio_init(PTD7,GPO,1);
  DELAY_MS(20);
  gpio_init(PTD7,GPO,0);
}

void led_shine(uint8 i)
{
  //0ָʾ��  1�����ֵ�  2Ŀ��ָʾ�� 3
  switch (i)
  {
    case 0:
      led_turn(LED0);
      break;
    case 1:
      led_turn(LED3);
      break;
    case 2:
      led (LED2,LED_ON);
      break;
    case 3:
      led (LED2,LED_OFF);
      break; 
  }
}