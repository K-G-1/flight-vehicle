#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f4xx.h"
#include "include.h"

struct _pid{
        float kp;
			  float ki;
	      float kd;
	      float increment;
	      float increment_max;
	      float kp_out;
			  float ki_out;
	      float kd_out;
	      float pid_out;
          };

struct _tache{
    struct _pid shell;
    struct _pid core;	
		struct _pid temp;
          };
	

struct _ctrl{
		      u8  ctrlRate;
      struct _tache pitch;    
	    struct _tache roll;  
	    struct _tache yaw;
			struct _tache height;
            };

extern struct _ctrl ctrl;						
						
extern u8 ARMED;

void CONTROL(float rol, float pit, float yaw);
void PID_INIT(void);
void PID_Test(T_RC_DATA *Rc_Data);
void PID_control(void);
						
						
#endif
