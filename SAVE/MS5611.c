/*°üº¬Í·------------------------------------------------------------------*/
#include "include.h"



/*±äÁ¿ÉùÃ÷----------------------------------------------------------------*/
uint16_t Cal_C[7];  //ÓÃÓÚ´æ·ÅPROMÖĞµÄ6×éÊı¾İ	
uint32_t D1_Pres,D2_Temp; // ´æ·ÅÊı×ÖÑ¹Á¦ºÍÎÂ¶È
float Pressure=0;				//ÎÂ¶È²¹³¥´óÆøÑ¹
float dT,Temperature,Temperature2;//Êµ¼ÊºÍ²Î¿¼ÎÂ¶ÈÖ®¼äµÄ²îÒì,Êµ¼ÊÎÂ¶È,ÖĞ¼äÖµ
double OFF,SENS;  //Êµ¼ÊÎÂ¶ÈµÖÏû,Êµ¼ÊÎÂ¶ÈÁéÃô¶È
float Aux,OFF2,SENS2;  //ÎÂ¶ÈĞ£ÑéÖµ

uint32_t ex_Pressure;			//´®¿Ú¶ÁÊı×ª»»Öµ
uint8_t  exchange_num[8];


/*º¯ÊıÉùÃ÷----------------------------------------------------------------*/
 void MS561101BA_Reset(void);
 void MS561101BA_readPROM(void);
 uint32_t MS561101BA_DO_CONVERSION(u8 command);
 void MS561101BA_GetTemperature(u8 OSR_Temp);
 void MS561101BA_GetPressure(u8 OSR_Pres);
 void MS561101BA_Init(void);
 void SampleANDExchange(void);
/************************************************************   
* º¯ÊıÃû:MS561101BA_Reset   
* ÃèÊö : ¸´Î»  
* ÊäÈë  :ÎŞ   
* Êä³ö  :ÎŞ    
*/ 
void MS561101BA_Reset(void)
{
		IIC_Start();
		IIC_Send_Byte(MS561101BA_ADDR);   //·¢ËÍµÍµØÖ·
		IIC_Wait_Ack(); 	 										  		   
		IIC_Send_Byte(MS561101BA_RESET);     //·¢ËÍ×Ö½Ú							   
		IIC_Wait_Ack();  		    	   
		IIC_Stop();//²úÉúÒ»¸öÍ£Ö¹Ìõ¼ş 
		delay_ms(10);
}


/************************************************************   
* º¯ÊıÃû:MS561101BA_readPROM   
* ÃèÊö : ´ÓPROM¶ÁÈ¡³ö³§Ğ£×¼Êı¾İ
* ÊäÈë  :ÎŞ   
* Êä³ö  :ÎŞ    
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
	 

//	printf("\rÉè±¸MS561101BA¶ÁÈ¡PROM: \r");
// printf("\rC1=%d,C2=%d,C3=%d,C4=%d,C5=%d,C6=%d\r\n\r",Cal_C[1],Cal_C[2],Cal_C[3],Cal_C[4],Cal_C[5],Cal_C[6]);

  
}

/************************************************************   
* º¯ÊıÃû:MS561101BA_DO_CONVERSION   
* ÃèÊö :  
* ÊäÈë  :ÎŞ   
* Êä³ö  :ÎŞ    
*/
uint32_t MS561101BA_DO_CONVERSION(uint8_t command)
{
	unsigned long conversion = 0;
	uint8_t conv1,conv2,conv3; 
	
	IIC_NoAddr_WriteByte(MS561101BA_ADDR,command);
	 
	delay_ms(10);//ÑÓÊ±,È¥µôÊı¾İ´íÎ
	
	
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
* º¯ÊıÃû:MS561101BA_GetTemperature   
* ÃèÊö : ¶ÁÈ¡Êı×ÖÎÂ¶È
* ÊäÈë  :¹ı²ÉÑùÂÊ   
* Êä³ö  :ÎŞ    
*/
void MS561101BA_GetTemperature(u8 OSR_Temp)
{
   
	D2_Temp= MS561101BA_DO_CONVERSION(OSR_Temp);	
	delay_ms(100);
	
	dT=D2_Temp - (((uint32_t)Cal_C[5])<<8);
	Temperature=2000+dT*((uint32_t)Cal_C[6])/8388608;	//Ëã³öÎÂ¶ÈÖµµÄ100±¶£¬2001±íÊ¾20.01¡ã


}


/************************************************************   
* º¯ÊıÃû:MS561101BA_GetPressure   
* ÃèÊö : ¶ÁÈ¡Êı×ÖÆøÑ¹
* ÊäÈë  :¹ı²ÉÑùÂÊ   
* Êä³ö  :ÎŞ    
*/
void MS561101BA_GetPressure(u8 OSR_Pres)
{
	
 
	
	D1_Pres= MS561101BA_DO_CONVERSION(OSR_Pres);

	delay_ms(100); 
	
	OFF=(double)((uint32_t)(Cal_C[2]<<16)+((uint32_t)Cal_C[4]*dT))/128.0;
	SENS=(uint32_t)(Cal_C[1]<<15)+((uint32_t)Cal_C[3]*dT)/256.0f;
	//ÎÂ¶È²¹³¥
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
* º¯ÊıÃû:MS561101BA_Init   
* ÃèÊö : MS561101BA³õÊ¼»¯
* ÊäÈë  :ÎŞ   
* Êä³ö  :ÎŞ    
*/ 
void MS561101BA_Init(void)
{
	MS561101BA_Reset();
	delay_ms(100);
	MS561101BA_readPROM();
	delay_ms(100);
} 

/************************************************************   
* º¯ÊıÃû:SampleANDExchange   
* ÃèÊö : ¶ÁÈ¡Êı¾İ²¢×ª»»´®¿Ú·¢ËÍ
* ÊäÈë  :ÎŞ   
* Êä³ö  :ÎŞ    
*/ 
void SampleANDExchange(void) 
{


	MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);//0x58
	MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);		//0x48

}

