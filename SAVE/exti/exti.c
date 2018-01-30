#include "exti.h"
#include "key.h"
#include "led.h"
#include "beep.h"
#include "delay.h"
#include "usart.h"

void exti_init()
{

	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	key_init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��

	//����ӳ��
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource2|EXTI_PinSource3|EXTI_PinSource4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
	
	//�����ж�����
	EXTI_InitStruct.EXTI_Line=EXTI_Line2|EXTI_Line3|EXTI_Line4;   //key0 key1 key2
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStruct);
	
  EXTI_InitStruct.EXTI_Line=EXTI_Line0;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);
	
	//�����жϷ���
	NVIC_InitStruct.NVIC_IRQChannel=EXTI2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=3;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStruct);
	
	//�жϷ�����

}

void EXTI0_IRQHandler(void)
{
	if(key_up==1)
	{
		delay_ms(50);
		if(key_up==1)
		{
			while(!key_up);
			BEEP =! BEEP;
			printf("key_up����");
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI2_IRQHandler(void)
{
	if(key2==0)
	{
		delay_ms(50);
		if(key2==0)
		{
			while(!key2);
			led1 =!led1 ;
			printf("key2����");
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line2);
}

void EXTI3_IRQHandler(void)
{
	if(key1==0)
	{
		delay_ms(50);
		if(key1==0)
		{
			while(!key1);
			led0=!led0 ;
			led1=!led1 ;
			printf("key1����");
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line3);
}

void EXTI4_IRQHandler(void)
{
	if(key0==0)
	{
		delay_ms(50);
		if(key0==0)
		{
			while(!key0);
			led0=!led0 ;
			printf("key0����");
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line4);
}
