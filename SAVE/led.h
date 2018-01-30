#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define led0 PFout(7)
#define led1 PFout(8)
#define led2 PFout(9)
#define led3 PFout(10)

void led_init(void);
void led_error_check(void);






#endif


