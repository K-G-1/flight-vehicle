#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"


#define key0   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)
#define key1   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)
#define key2   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)
#define key_up GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)


void key_init(void);

#endif

