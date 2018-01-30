#include "include.h" 
				 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//24CXX ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//��ʼ��IIC�ӿ�
void AT24CXX_Init(void)
{
	IIC_Init();//IIC��ʼ��
	if((AT24CXX_Check())==0)
		error_check=1;
}
//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8  AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8  temp=0;		  	    																 
    IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(0XA0);	   //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(ReadAddr>>8);//���͸ߵ�ַ	    
	}else IIC_Send_Byte(0XA0+((ReadAddr/256)<<1));   //����������ַ0XA0,д���� 	   
	IIC_Wait_Ack(); 
    IIC_Send_Byte(ReadAddr%256);   //���͵͵�ַ
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(0XA1);           //�������ģʽ			   
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte();	
		IIC_NAck();
    IIC_Stop();//����һ��ֹͣ����	    
	return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(u16 WriteAddr,u32 DataToWrite)
{				   	  	    																 
    IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(0XA0);	    //����д����
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//���͸ߵ�ַ	  
	}else IIC_Send_Byte(0XA0+((WriteAddr/256)<<1));   //����������ַ0XA0,д���� 	 
	IIC_Wait_Ack();	   
    IIC_Send_Byte(WriteAddr%256);   //���͵͵�ַ
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //�����ֽ�							   
	IIC_Wait_Ack();  		    	   
    IIC_Stop();//����һ��ֹͣ���� 
	delay_ms(10);	 
}
//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
int16_t AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//����ÿ�ο�����дAT24CXX			   
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(32767,0X55);
	    temp=AT24CXX_ReadOneByte(32767);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}


/******************************���ݴ���*************************************************/
extern int16_t offset_mx,offset_my,offset_mz,HMC5883_minx,HMC5883_miny,HMC5883_minz,
								HMC5883_maxx,HMC5883_maxy,HMC5883_maxz;

void AT24CXX_Write_save_Byte(u8 addr,int16_t data,u8 num)
{
	int temp;
	
	temp = BYTE0(data);
	AT24CXX_WriteOneByte(addr,temp);
	temp = BYTE1(data);
	AT24CXX_WriteOneByte(addr+1,temp);
}

void AT24cxx_save_Acc_Gyro_offest(void)
{
	if(sensor.gyro.quiet.x < 0)
	{
		sensor.gyro.flag.x = 0;
	}else sensor.gyro.flag.x = 1;
	if(sensor.gyro.quiet.y < 0)
	{
		sensor.gyro.flag.y = 0;
	}else sensor.gyro.flag.y = 1;
	if(sensor.gyro.quiet.z < 0)
	{
		sensor.gyro.flag.z = 0;
	}else sensor.gyro.flag.z = 1;
	
	if(sensor.acc.quiet.x < 0)
	{
		sensor.acc.flag.x = 0;
	}else sensor.acc.flag.x = 1;
	if(sensor.acc.quiet.y < 0)
	{
		sensor.acc.flag.y = 0;
	}else sensor.acc.flag.y = 1;
	if(sensor.acc.quiet.z < 0)
	{
		sensor.acc.flag.z = 0;
	}else sensor.acc.flag.z = 1;
	
	AT24CXX_WriteLenByte(save_gyro_x-1,sensor.gyro.flag.x,1);
	AT24CXX_WriteLenByte(save_gyro_y-1,sensor.gyro.flag.y,1);
	AT24CXX_WriteLenByte(save_gyro_z-1,sensor.gyro.flag.z,1);
	AT24CXX_WriteLenByte(save_acc_x-1,sensor.acc.flag.x,1);
	AT24CXX_WriteLenByte(save_acc_y-1,sensor.acc.flag.y,1);
	AT24CXX_WriteLenByte(save_acc_z-1,sensor.acc.flag.z,1);
	
	
	IIC_ADD_writelen_BYTE(save_gyro_x,(sensor.gyro.quiet.x),2);
	IIC_ADD_writelen_BYTE(save_gyro_y,(sensor.gyro.quiet.y),2);
	IIC_ADD_writelen_BYTE(save_gyro_z,(sensor.gyro.quiet.z),2);
	
	AT24CXX_WriteLenByte(save_acc_x,(sensor.acc.quiet.x),2);
	AT24CXX_WriteLenByte(save_acc_y,(sensor.acc.quiet.y),2);
	AT24CXX_WriteLenByte(save_acc_z,(sensor.acc.quiet.z),2);
	
}

void AT24cxx_save_5883_offest(void)
{
	IIC_ADD_writelen_BYTE(save_5883_x,offset_mx,2);
	IIC_ADD_writelen_BYTE(save_5883_y,offset_my,2);
	IIC_ADD_writelen_BYTE(save_5883_z,offset_mz,2);
	
	IIC_ADD_writelen_BYTE(save_5883_max_x,HMC5883_maxx,1);
	IIC_ADD_writelen_BYTE(save_5883_max_y,HMC5883_maxy,1);
	IIC_ADD_writelen_BYTE(save_5883_max_z,HMC5883_maxz,1);
	IIC_ADD_writelen_BYTE(save_5883_min_x,HMC5883_minx,1);
	IIC_ADD_writelen_BYTE(save_5883_min_y,HMC5883_miny,1);
	IIC_ADD_writelen_BYTE(save_5883_min_z,HMC5883_minz,1);
	
}




void AT24cxx_save_PID_shell(void)
{
	u16 temp_P,temp_I,temp_D;
	
	temp_P= ctrl.pitch.shell.kp *1000;
	temp_I= ctrl.pitch.shell.ki *1000;
	temp_D= ctrl.pitch.shell.kd *1000;
	
	AT24CXX_WriteLenByte(Shell_Pitch_P,temp_P,2);
	AT24CXX_WriteLenByte(Shell_Pitch_I,temp_I,2);
	AT24CXX_WriteLenByte(Shell_Pitch_D,temp_D,2);

	
	temp_P= ctrl.roll.shell.kp *1000;
	temp_I= ctrl.roll.shell.ki *1000;
	temp_D= ctrl.roll.shell.kd *1000;
	
	IIC_ADD_writelen_BYTE(Shell_Roll_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Shell_Roll_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Shell_Roll_D,temp_D,2);
	
	
	temp_P= ctrl.yaw.shell.kp *1000;
	temp_I= ctrl.yaw.shell.ki *1000;
	temp_D= ctrl.yaw.shell.kd *1000;
	
	IIC_ADD_writelen_BYTE(Shell_Yaw_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Shell_Yaw_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Shell_Yaw_D,temp_D,2);
	
	
}



void AT24cxx_save_PID_core(void)
{
	u16 temp_P,temp_I,temp_D;
	
	temp_P= ctrl.pitch.core.kp *1000;
	temp_I= ctrl.pitch.core.ki *1000;
	temp_D= ctrl.pitch.core.kd *1000;
	
	IIC_ADD_writelen_BYTE(Core_Pitch_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Core_Pitch_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Core_Pitch_D,temp_D,2);
	
	
	temp_P= ctrl.roll.core.kp *1000;
	temp_I= ctrl.roll.core.ki *1000;
	temp_D= ctrl.roll.core.kd *1000;
	
	IIC_ADD_writelen_BYTE(Core_Roll_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Core_Roll_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Core_Roll_D,temp_D,2);
	
	
	temp_P= ctrl.yaw.core.kp *1000;
	temp_I= ctrl.yaw.core.ki *1000;
	temp_D= ctrl.yaw.core.kd *1000;
	
	IIC_ADD_writelen_BYTE(Core_Yaw_P,temp_P,2);
	IIC_ADD_writelen_BYTE(Core_Yaw_I,temp_I,2);
	IIC_ADD_writelen_BYTE(Core_Yaw_D,temp_D,2);
	
}


void AT24cxx_save_PID_hight(void)
{
	u16 temp_P,temp_I,temp_D;
	
	temp_P= ctrl.height.shell.kp *1000;
	temp_I= ctrl.height.shell.ki *1000;
	temp_D= ctrl.height.shell.kd *1000;
	
	AT24CXX_WriteLenByte(Hight_P,temp_P,2);
	AT24CXX_WriteLenByte(Hight_I,temp_I,2);
	AT24CXX_WriteLenByte(Hight_D,temp_D,2);
	
	
}


//*************************************************************************//

u32 AT24CXX_Read_save_Byte(u8 addr)
{
	u32 data;
	u32 data_high,data_low;
	data_high=AT24CXX_ReadOneByte(addr);
	data_low=AT24CXX_ReadOneByte(addr+1);
	data=(data_high<<8)|data_low;
	return data;
}
void AT24cxx_read_Acc_Gyro_offest(void)
{
	
//	sensor.gyro.quiet.x=AT24CXX_ReadLenByte(save_gyro_x-1,1);
//	sensor.gyro.quiet.y=AT24CXX_ReadLenByte(save_gyro_y-1,1);
//	sensor.gyro.quiet.z=AT24CXX_ReadLenByte(save_gyro_z-1,1);
	sensor.acc.quiet.x=AT24CXX_ReadLenByte(save_acc_x,2);
	sensor.acc.quiet.y=AT24CXX_ReadLenByte(save_acc_y,2);
	sensor.acc.quiet.z=AT24CXX_ReadLenByte(save_acc_z,2);
//	
//	
//	if(sensor.gyro.quiet.x ==0)
//	{
//		sensor.gyro.quiet.x= -IIC_Read_MultiBytes(0xA0,save_gyro_x,2);
//	}else sensor.gyro.quiet.x=IIC_Read_MultiBytes(0xA0,save_gyro_x,2);
//	if(sensor.gyro.quiet.y ==0)
//	{
//		sensor.gyro.quiet.y= -AT24CXX_ReadLenByte(save_gyro_y,2);
//	}else sensor.gyro.quiet.y= AT24CXX_ReadLenByte(save_gyro_y,2);
//	if(sensor.gyro.quiet.z ==0)
//	{
//		sensor.gyro.quiet.z= -IIC_Read_MultiBytes(0xA0,save_gyro_z,2);
//	}else sensor.gyro.quiet.z=IIC_Read_MultiBytes(0xA0,save_gyro_z,2);
//	

  sensor.gyro.quiet.x=IIC_Read_MultiBytes(0xA0,save_gyro_x,2);
	sensor.gyro.quiet.y=IIC_Read_MultiBytes(0xA0,save_gyro_y,2);
	sensor.gyro.quiet.z=IIC_Read_MultiBytes(0xA0,save_gyro_z,2);
	
	
//	sensor.acc.quiet.x=IIC_Read_MultiBytes(0xA0,save_acc_x,2);
//	sensor.acc.quiet.y=IIC_Read_MultiBytes(0xA0,save_acc_y,2);
//	sensor.acc.quiet.z=IIC_Read_MultiBytes(0xA0,save_acc_z,2);

}

void AT24cxx_read_5883_offest(void)
{
	offset_mx=IIC_Read_MultiBytes(0xA0,save_5883_x,2);
	offset_my=IIC_Read_MultiBytes(0xA0,save_5883_y,2);
	offset_mz=IIC_Read_MultiBytes(0xA0,save_5883_z,2);
	offset_mx=0;
	offset_my=0;
	offset_mz=0;
}


void AT24cxx_read_PID_shell(void)
{
		u16 temp_P,temp_I,temp_D;
	
	temp_P= AT24CXX_ReadLenByte(Shell_Pitch_P,2);
	temp_I= AT24CXX_ReadLenByte(Shell_Pitch_I,2);
	temp_D= AT24CXX_ReadLenByte(Shell_Pitch_D,2);
	
	ctrl.pitch.shell.kp= (float)temp_P/1000.0f;
	ctrl.pitch.shell.ki= (float)temp_I/1000.0f;
	ctrl.pitch.shell.kd= (float)temp_D/1000.0f;
	
	
	temp_P= IIC_Read_MultiBytes(0xA0,Shell_Roll_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Shell_Roll_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Shell_Roll_D,2);
	
	ctrl.roll.shell.kp= (float)temp_P/1000.0f;
	ctrl.roll.shell.ki= (float)temp_I/1000.0f;
	ctrl.roll.shell.kd= (float)temp_D/1000.0f;
	
	
	temp_P= IIC_Read_MultiBytes(0xA0,Shell_Yaw_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Shell_Yaw_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Shell_Yaw_D,2);
	
	ctrl.yaw.shell.kp= (float)temp_P/1000.0f;
	ctrl.yaw.shell.ki= (float)temp_I/1000.0f;
	ctrl.yaw.shell.kd= (float)temp_D/1000.0f;
	
}




void AT24cxx_read_PID_core(void)
{
		u16 temp_P,temp_I,temp_D;
	
	temp_P= IIC_Read_MultiBytes(0xA0,Core_Pitch_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Core_Pitch_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Core_Pitch_D,2);
	
	ctrl.pitch.core.kp= (float)temp_P/1000.0f;
	ctrl.pitch.core.ki= (float)temp_I/1000.0f;
	ctrl.pitch.core.kd= (float)temp_D/1000.0f;
	
	
	temp_P= IIC_Read_MultiBytes(0xA0,Core_Roll_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Core_Roll_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Core_Roll_D,2);
	
	ctrl.roll.core.kp= (float)temp_P/1000.0f;
	ctrl.roll.core.ki= (float)temp_I/1000.0f;
	ctrl.roll.core.kd= (float)temp_D/1000.0f;
	
	
	temp_P= IIC_Read_MultiBytes(0xA0,Core_Yaw_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Core_Yaw_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Core_Yaw_D,2);
	
	ctrl.yaw.core.kp= (float)temp_P/1000.0f;
	ctrl.yaw.core.ki= (float)temp_I/1000.0f;
	ctrl.yaw.core.kd= (float)temp_D/1000.0f;
	
}

void AT24cxx_read_PID_hight(void)
{
		u16 temp_P,temp_I,temp_D;
	
	temp_P= IIC_Read_MultiBytes(0xA0,Hight_P,2);
	temp_I= IIC_Read_MultiBytes(0xA0,Hight_I,2);
	temp_D= IIC_Read_MultiBytes(0xA0,Hight_D,2);
	
	ctrl.height.shell.kp= (float)temp_P/1000.0f;
	ctrl.height.shell.ki= (float)temp_I/1000.0f;
	ctrl.height.shell.kd= (float)temp_D/1000.0f;
	

	
}



//****************************************************************************************//

