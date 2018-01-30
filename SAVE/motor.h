#ifndef __motor_H
#define __motor_H	 
#include "sys.h"


void Tim2_init(void);
void Moto_Init(void);
void Moto_PwmRflash(uint16_t MOTO1_PWM,uint16_t MOTO2_PWM,uint16_t MOTO3_PWM,uint16_t MOTO4_PWM);

#endif
