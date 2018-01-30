#include "include.h"


struct _Hight  Hight;
void hight_init(void )
{
	Usart2_init();
}


float g_Alt_Hight=0,g_Alt_HightOld=0;
float g_HightControlold=0;
void fifter_hight(unsigned int set_hight)
{
	static float Alt_Hight[3];
	static float hight_error=0,hight_errorold=0;	
	
	Alt_Hight[2]=Alt_Hight[1];
	Alt_Hight[1]=Alt_Hight[0];
	Alt_Hight[0]=(float)Hight.averag*((float)cos(angle.roll/RtA)*(float)cos(angle.pitch/RtA));
	g_Alt_HightOld=g_Alt_Hight;
	g_Alt_Hight=(Alt_Hight[0]+Alt_Hight[1]+Alt_Hight[2])/3;
	
	if(g_Alt_Hight-g_Alt_HightOld>500)						   //·À²âÁ¿´íÎó
	    g_Alt_Hight=g_Alt_HightOld+500;
	else if(g_Alt_HightOld-g_Alt_Hight>500)
		g_Alt_Hight=g_Alt_HightOld-100;

	if((mode!= 2)||!ARMED)
		ctrl.height.shell.increment=0;
		

		
	hight_errorold=hight_error;
	hight_error=(float)(set_hight - g_Alt_Hight)/10.0f;
	
	ctrl.height.shell.increment+=hight_error;	
	if(ctrl.height.shell.increment > ctrl.height.shell.increment_max)
			ctrl.height.shell.increment = ctrl.height.shell.increment_max;
	else if(ctrl.height.shell.increment < -ctrl.height.shell.increment_max)
			ctrl.height.shell.increment = -ctrl.height.shell.increment_max;	

	g_HightControlold = ctrl.height.shell.pid_out ;
	ctrl.height.shell.pid_out = ctrl.height.shell.kp * hight_error ;
//															+ ctrl.height.shell.ki * ctrl.height.shell.increment 
//															+ ctrl.height.shell.kd * (hight_error - hight_errorold);	
	
	if(ctrl.height.shell.pid_out-g_HightControlold>70)						   //·ÀÍ»±ä
	    ctrl.height.shell.pid_out=g_HightControlold+70;
	else if(g_HightControlold-ctrl.height.shell.pid_out>70)
		ctrl.height.shell.pid_out=g_HightControlold-70;
	
	if(ctrl.height.shell.pid_out>500)
		ctrl.height.shell.pid_out=500;


//	ctrl.height.shell.pid_out = ctrl.height.shell.kp * hight_error ;
}




