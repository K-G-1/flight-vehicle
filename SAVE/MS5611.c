/*包含头------------------------------------------------------------------*/
#include "include.h"



/*变量声明----------------------------------------------------------------*/
uint16_t Cal_C[7];  //用于存放PROM中的6组数据	
uint32_t D1_Pres,D2_Temp; // 存放数字压力和温度
float Pressure=0;				//温度补偿大气压
float dT,Temperature,Temperature2;//实际和参考温度之间的差异,实际温度,中间值
double OFF,SENS;  //实际温度抵消,实际温度灵敏度
float Aux,OFF2,SENS2;  //温度校验值

uint32_t ex_Pressure;			//串口读数转换值
uint8_t  exchange_num[8];


/*函数声明----------------------------------------------------------------*/
 void MS561101BA_Reset(void);
 void MS561101BA_readPROM(void);
 uint32_t MS561101BA_DO_CONVERSION(u8 command);
 void MS561101BA_GetTemperature(u8 OSR_Temp);
 void MS561101BA_GetPressure(u8 OSR_Pres);
 void MS561101BA_Init(void);
 void SampleANDExchange(void);
/************************************************************   
* 函数名:MS561101BA_Reset   
* 描述 : 复位  
* 输入  :无   
* 输出  :无    
*/ 
void MS561101BA_Reset(void)
{
		IIC_Start();
		IIC_Send_Byte(MS561101BA_ADDR);   //发送低地址
		IIC_Wait_Ack(); 	 										  		   
		IIC_Send_Byte(MS561101BA_RESET);     //发送字节							   
		IIC_Wait_Ack();  		    	   
		IIC_Stop();//产生一个停止条件 
		delay_ms(10);
}


/************************************************************   
* 函数名:MS561101BA_readPROM   
* 描述 : 从PROM读取出厂校准数据
* 输入  :无   
* 输出  :无    
*/ 
void MS561101BA_readPROM(void)
{   

	  u8 i,d1,d2;
	  for(i=1;i<=6;i++)
	 {
		IIC_Start();
		IIC_Send_Byte(MS561101BA_ADDR);
		IIC_Wait_Ack();
		IIC_Send_Byte((MS561101BA_PROM_BASE_ADDR+i*2));
		IIC_Wait_Ack();
	  IIC_Stop();
		delay_ms(1);

		IIC_Start();
		IIC_Send_Byte(MS561101BA_ADDR+1);
		IIC_Wait_Ack();
		d1=IIC_Read_Byte();
		IIC_Ack();
		d2=IIC_Read_Byte();
		IIC_NAck();
		IIC_Stop();


		Cal_C[i]=((uint16_t)d1<<8)|d2;
	  delay_ms(10);
	 }
	 

//	printf("\r设备MS561101BA读取PROM: \r");
// printf("\rC1=%d,C2=%d,C3=%d,C4=%d,C5=%d,C6=%d\r\n\r",Cal_C[1],Cal_C[2],Cal_C[3],Cal_C[4],Cal_C[5],Cal_C[6]);

  
}

/************************************************************   
* 函数名:MS561101BA_DO_CONVERSION   
* 描述 :  
* 输入  :无   
* 输出  :无    
*/
uint32_t MS561101BA_DO_CONVERSION(uint8_t command)
{
	unsigned long conversion = 0;
	uint8_t conv1,conv2,conv3; 
	
	IIC_NoAddr_WriteByte(MS561101BA_ADDR,command);
	 
	delay_ms(10);//延时,去掉数据错�
	
	
	IIC_Start();
	IIC_Send_Byte(MS561101BA_ADDR);
	IIC_Wait_Ack();
	IIC_Send_Byte(0);
	IIC_Wait_Ack();
  IIC_Stop();


	IIC_Start();
	IIC_Send_Byte(MS561101BA_ADDR+1);
	IIC_Wait_Ack();
	conv1=IIC_Read_Byte();
	IIC_Wait_Ack();
	conv2=IIC_Read_Byte();
	IIC_Ack();
	conv3=IIC_Read_Byte();

	IIC_NAck();
	IIC_Stop();

	conversion=conv1*65535+conv2*256+conv3;
	return conversion;

}

/************************************************************   
* 函数名:MS561101BA_GetTemperature   
* 描述 : 读取数字温度
* 输入  :过采样率   
* 输出  :无    
*/
void MS561101BA_GetTemperature(u8 OSR_Temp)
{
   
	D2_Temp= MS561101BA_DO_CONVERSION(OSR_Temp);	
	delay_ms(100);
	
	dT=D2_Temp - (((uint32_t)Cal_C[5])<<8);
	Temperature=2000+dT*((uint32_t)Cal_C[6])/8388608;	//算出温度值的100倍，2001表示20.01°


}


/************************************************************   
* 函数名:MS561101BA_GetPressure   
* 描述 : 读取数字气压
* 输入  :过采样率   
* 输出  :无    
*/
void MS561101BA_GetPressure(u8 OSR_Pres)
{
	
 
	
	D1_Pres= MS561101BA_DO_CONVERSION(OSR_Pres);

	delay_ms(100); 
	
	OFF=(double)((uint32_t)(Cal_C[2]<<16)+((uint32_t)Cal_C[4]*dT))/128.0;
	SENS=(uint32_t)(Cal_C[1]<<15)+((uint32_t)Cal_C[3]*dT)/256.0f;
	//温度补偿
	if(Temperature < 2000)// second order temperature compensation when under 20 degrees C
	{
		Temperature2 = (dT*dT) / 0x80000000;
		Aux = (Temperature-2000)*(Temperature-2000);
		OFF2 = 2.5f*Aux;
		SENS2 = 1.25f*Aux;
		if(Temperature < -1500)
		{
			Aux = (Temperature+1500)*(Temperature+1500);
			OFF2 = OFF2 + 7*Aux;
			SENS2 = SENS + 5.5f*Aux;
		}
	}else  //(Temperature > 2000)
	{
		Temperature2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	
	Temperature = Temperature - Temperature2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;	

	Pressure=(D1_Pres*SENS/2097152.0-OFF)/32768.0;

}

/************************************************************   
* 函数名:MS561101BA_Init   
* 描述 : MS561101BA初始化
* 输入  :无   
* 输出  :无    
*/ 
void MS561101BA_Init(void)
{
	MS561101BA_Reset();
	delay_ms(100);
	MS561101BA_readPROM();
	delay_ms(100);
} 

/************************************************************   
* 函数名:SampleANDExchange   
* 描述 : 读取数据并转换串口发送
* 输入  :无   
* 输出  :无    
*/ 
void SampleANDExchange(void) 
{


	MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);//0x58
	MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);		//0x48

}

