#include "include.h"

extern float Pressure,Temperature;
extern u8 Mag_CALIBRATED;
float Yaw_offest;
u8 error_check=0;
u8 mode = 0;
volatile u32 SYS_Time=0;
extern u8 Lenth_buff[2];

void Delay_Ms(u32 Time);




void init_paramLoad(void)
{
	sensor.acc.CALIBRATE=0;
	sensor.gyro.CALIBRATE=0;

	IIC_Init();	
	Init_HMC5883L();                                 //地磁计初始化
	
	MS561101BA_Init();                              //气压计
	MPU_Init();   
	hight_init();																		//串口2(超声波)初始化
	AT24cxx_read_Acc_Gyro_offest();                //读取加速度计，陀螺仪的静态值
	AT24cxx_read_5883_offest();
	AT24cxx_read_PID_shell();
	AT24cxx_read_PID_core();
	AT24cxx_read_PID_hight();
	mode =0; 
//	//Gyro_OFFEST();
//	
//	// The data of pitch
//	ctrl.pitch.shell.kp = 1.05;    //5  2.0
//	ctrl.pitch.shell.ki = 0.00;//0.5;     
//	ctrl.pitch.shell.kd = 0;//5;    //2
//	
//	ctrl.pitch.core.kp = 1.1;   //1.5     2.1
//	ctrl.pitch.core.ki = 0.000;
//	ctrl.pitch.core.kd = 0.14;  //0.16    0.14
//	
//	//The data of roll
//	ctrl.roll.shell.kp = 1.05;
//	ctrl.roll.shell.ki = 0.00;//0.5;
//	ctrl.roll.shell.kd = 0;//5;

//	ctrl.roll.core.kp = 1.1;
//	ctrl.roll.core.ki = 0.000;
//	ctrl.roll.core.kd = 0.14;
//	
//	//The data of yaw
//	ctrl.yaw.shell.kp = 3.4;	
//	ctrl.yaw.shell.kd = 0.13;
//	
//	ctrl.yaw.core.kp = 0.8;
//	ctrl.yaw.core.kd = 0.06;
	

	
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 100; 
	ctrl.roll.shell.increment_max = 100;
	ctrl.yaw.shell.increment_max = 100;
	
	ctrl.pitch.core.increment_max = 100;
	ctrl.roll.core.increment_max = 100;
	ctrl.yaw.core.increment_max = 15;
	
	ctrl.height.shell.increment_max = 15;
	ctrl.ctrlRate = 0;

	Rc_Data.pitch_offset = 1500;
	Rc_Data.roll_offset = 1500;
	Rc_Data.yaw_offset = 1500;
}




int main()
{


//	SysTick_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init();

	uart_init(500000);
	led_init();

	IIC_Init();


	init_paramLoad();                               //数据初始化
	delay_ms(500);
	init_paramLoad();                               //数据初始化
                          

  Moto_Init();                    //TIM2，      即电机初始化
	PWM_IN_Init();                  //TIM3、TIM4，遥控器初始化
	tim5_init(30-1,8400-1);         //TIM5，      即时基初始化，4ms开启一次中断  (ARR+1)*(PSC+1)/clk   clk=84000khz
	TIM6_init();


	while(1)
	{

//		fifter_hight();
//		delay_ms(5);
//		sand_IMU_data();
		
//		printf("%d\n",Hight.histor);
//		GPIO_ResetBits(GPIOE,GPIO_Pin_6);
//		delay_ms(1000);

//		GPIO_SetBits(GPIOE,GPIO_Pin_6);
//		delay_ms(1000);
	}
}


