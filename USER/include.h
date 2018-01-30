#ifndef __include_H
#define __include_H	 
#include "sys.h"

#include "stm32f4xx.h"
#include "delay.h"
//#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "MS5611.h"
#include "HMC5883L.h"
#include "IIC.h"
#include  <math.h> 
#include "24cxx.h" 
#include "timer3.h"
#include "IMU.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "RC.h"
#include "control.h"
#include "Algorithm_math.h"
#include "Algorithm_filter.h"
#include "usart.h"
#include "motor.h"
#include "sand_data.h"
#include  "drv_w25qxx.h"
#include "hight.h"



void  SysTick_Configuration(void);
uint32_t GetSysTime_us(void) ;
extern float Yaw_offest;
extern u8 error_check,mode;
extern int TIM5_time,TIM5_last_time;
void init_paramLoad(void); 
#endif
