#ifndef _IIC_H_
#define _IIC_H_

#include "stm32f4xx_conf.h"
#include "sys.h"
#include "delay.h"
/*��������----------------------------------------------------------------*/

#define SCL     GPIO_Pin_6 //24C02 SCL
#define SDA     GPIO_Pin_7 //24C02 SDA 

//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //PB9���ģʽ
//IO��������	
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA
#define SDA_H               GPIO_SetBits(GPIOB,SDA)          /*SDA�����*/
#define SDA_L               GPIO_ResetBits(GPIOB,SDA)        /*SDA������*/
#define SDA_READ            GPIO_ReadInputDataBit(GPIOB,SDA) /* ��ȡSDA*/




//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
u16 IIC_Start(void);
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(void);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 


//���⺯��
void IIC_ADD_write(u8 DeviceAddr,u8 address,u8 Bytes);
void IIC_ADD_writelen_BYTE(u16 WriteAddr,u32 DataToWrite,u8 Len);
u8 IIC_ADD_read(u8 DeviceAddr,u8 address);
int16_t IIC_Read_MultiBytes(u8 DeviceAddr,u8 address,u8 Len);
void IIC_NoAddr_WriteByte(unsigned char address,unsigned char Bytes);



#endif

//------------------End of File----------------------------

