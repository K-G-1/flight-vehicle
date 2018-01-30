#include "timer3.h"
#include  "include.h"

int TIM5_time,TIM5_last_time;
void tim5_init(u32 arr,u16 psc)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	//timer3中断配置

	TIM_TimeBaseInitStruct.TIM_Period=arr;
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStruct);
	

	
	//中断优先级配置
	NVIC_InitStruct.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE ;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_Init(&NVIC_InitStruct);
	

	
	//使能timer3中断
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM5, ENABLE);

}


void TIM6_init(void)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	//timer3中断配置

	TIM_TimeBaseInitStruct.TIM_Period= 10000;
	TIM_TimeBaseInitStruct.TIM_Prescaler= 8400-1;
	TIM_TimeBaseInitStruct.TIM_ClockDivision= TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStruct);
	

	
	//中断优先级配置
	NVIC_InitStruct.NVIC_IRQChannel=TIM7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE ;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x00;
	NVIC_Init(&NVIC_InitStruct);
	

	
	//使能timer7中断
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM7, ENABLE);
//	TIM_Cmd(TIM7, DISABLE);
	
	
	
}





//****************************************************************//

void TIM7_IRQHandler(void)
{
	if( TIM_GetITStatus(TIM7 ,TIM_IT_Update)==SET)
	{
		HMC5883_cnt++;
	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
}





u8  times=0,times1=0,Hight_time;
unsigned int g_Set_Hight=0;
void TIM5_IRQHandler(void)
{
	static u16  times=0,times1=0;
	if( TIM_GetITStatus(TIM5 ,TIM_IT_Update)==SET)
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		TIM5_last_time= TIM5->CNT=0;
		
		RC_Data_Refine();
		Get_Attitude();
		mode_contrl();
//		CONTROL(angle.pitch,angle.roll,angle.yaw); //电机控制
//		CONTROL(angle.roll,angle.pitch,angle.yaw); //电机控制
//		CONTROL(angle.roll,angle.pitch,angle.yaw); //电机控制
		times++;
		Hight_time++;
		times1++;
		
		if(times1==4)
		{
			times1=0;
			if(error_check!= 0)
			{
				led_error_check();
				ARMED=0;
			}
			else
			{
				Deblocking();
			}
			if(!ARMED)	 //LED_Sailing(5);
			{
				Yaw_offest=angle.yaw;	
      }
			
			
		}
		
		if(mode == 0)
		{
			CONTROL(angle.roll,angle.pitch,0); //电机控制
		}
//		else if(mode ==1)
//		{
//			CONTROL(angle.roll,angle.pitch,angle.yaw); //电机控制
//		}
//		else if(mode ==2)
//		{
//			fifter_hight(g_Set_Hight);
//			CONTROL(angle.roll,angle.pitch,angle.yaw); //电机控制
//		}

		if(Hight_time%5==0)
		{
			USART2_Send_Byte(0x55);
		}
		if(Hight_time==10&&mode!=2)
		{
			g_Set_Hight=(float)Hight.averag*((float)cos(angle.roll/RtA)*(float)cos(angle.pitch/RtA));
			Hight_time=0;
			
		}
		
//		if(Hight_time==5&&mode==2)
//		{
//			fifter_hight(g_Set_Hight);
//			Hight_time=0;
//		}
		
		
		
/******************************************************/	
	
		if(times==1)
		{
			sand_IMU_data();
		}
		if(times == 3)
		{
			sand_RC_data();		
//			sand_origin();
		}
		if(times==5)
		{
			sand_Motor_data();
			
		}
		if(times==7)
		{
			sand_ACC_GYRO_data();
		}
		if(times>=9)
		{
			sand_F1_origin();
			sand_F2_origin();
			times=0;
			
		}
		TIM5_time = TIM5->CNT - TIM5_last_time ;
		
	}
//	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
}





