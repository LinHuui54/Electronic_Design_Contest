#ifndef     __ACC_GYRO_H__
#define     __ACC_GYRO_H__
#include "common.h"


#define	L3G4200_Addr  0xD2	 //
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38




#define	MMA8451_Addr          0x38

typedef enum
{
  MMA845x_STATUS_00 = 0x00,          // 0x00
  MMA845x_OUT_X_MSB,              // 0x01
  MMA845x_OUT_X_LSB,              // 0x02
  MMA845x_OUT_Y_MSB,              // 0x03
  MMA845x_OUT_Y_LSB,              // 0x04
  MMA845x_OUT_Z_MSB,              // 0x05
  MMA845x_OUT_Z_LSB,              // 0x06
  MMA845x_F_STATUS=0x08,          // 0x08
  MMA845x_F_SETUP,                // 0x09
  MMA845x_TRIG_CFG,               // 0x0A
  MMA845x_SYSMOD,                 // 0x0B
  MMA845x_INT_SOURCE,             // 0x0C
  MMA845x_WHO_AM_I,               // 0x0D
  MMA845x_XYZ_DATA_CFG,           // 0x0E
  MMA845x_HP_FILTER_CUTOFF,       // 0x0F
  MMA845x_PL_STATUS,              // 0x10
  MMA845x_PL_CFG,                 // 0x11
  MMA845x_PL_COUNT,               // 0x12
  MMA845x_PL_BF_ZCOMP,            // 0x13
  MMA845x_PL_P_L_THS_REG,         // 0x14
  MMA845x_FF_MT_CFG,              // 0x15
  MMA845x_FF_MT_SRC,              // 0x16
  MMA845x_FF_MT_THS,              // 0x17
  MMA845x_FF_MT_COUNT,            // 0x18
  MMA845x_TRANSIENT_CFG=0x1d,     // 0x1D
  MMA845x_TRANSIENT_SRC,          // 0x1E
  MMA845x_TRANSIENT_THS,          // 0x1F
  MMA845x_TRANSIENT_COUNT,        // 0x20
  MMA845x_PULSE_CFG,              // 0x21
  MMA845x_PULSE_SRC,              // 0x22
  MMA845x_PULSE_THSX,             // 0x23
  MMA845x_PULSE_THSY,             // 0x24
  MMA845x_PULSE_THSZ,             // 0x25
  MMA845x_PULSE_TMLT,             // 0x26
  MMA845x_PULSE_LTCY,             // 0x27
  MMA845x_PULSE_WIND,             // 0x28
  MMA845x_ASLP_COUNT,             // 0x29
  MMA845x_CTRL_REG1,              // 0x2A
  MMA845x_CTRL_REG2,              // 0x2B
  MMA845x_CTRL_REG3,              // 0x2C
  MMA845x_CTRL_REG4,              // 0x2D
  MMA845x_CTRL_REG5,              // 0x2E
  MMA845x_OFF_X,                  // 0x2F
  MMA845x_OFF_Y,                  // 0x30
  MMA845x_OFF_Z                   // 0x31
}MMA8451_e;
extern float adeo;
extern float GYRO_ang;
extern float GYRO_X,GYRO_Y,GYRO_Z,ACC_ANGLE;
extern float angleX,angleY, angle_dot;         //外部需要引用的变量         //外部需要引用的变量
extern void MMA845x_init(void);
extern void Init_L3G4200D(void);
extern int16 getdata(uint8 s_addr,uint8 addr);
extern void acc_gyro_Calculate(void);
extern void Kalman_Filter(float angle_m,float gyro_m)     ;
extern void kem(void);
extern float Comlementary2(float newangle,float newgyro);
extern volatile int MMA7361;


#endif