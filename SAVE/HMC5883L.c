/*����ͷ------------------------------------------------------------------*/
#include "include.h"


///*��������----------------------------------------------------------------*/
int Magn_x,Magn_y,Magn_z;    //5883�õ�
int Magn_Ori_x,Magn_Ori_y,Magn_Ori_z;

u8 Mag_CALIBRATED=0;

uint8_t HMC5883L_buffer[8];



#define  FILL_NUM  10


int16_t X_BUFF[FILL_NUM],Y_BUFF[FILL_NUM],Z_BUFF[FILL_NUM];
float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;
//�����Ʊ궨ֵ
int16_t  HMC5883_maxx=0,HMC5883_maxy=0,HMC5883_maxz=0,
		 HMC5883_minx=-0,HMC5883_miny=-0,HMC5883_minz=-0;

int16_t offset_mx,offset_my,offset_mz;	
int16_t temp_mx_scale,temp_my_scale,temp_mz_scale;
float mx_scale = 1.0,my_scale = 1.0 ,mz_scale = 1.0;

unsigned char HMC5883_calib=0; //��ʼ����ɱ�־
u8 HMC5883_cnt=0;

u8 Init_HMC5883L(void)
{
		u8 tempA ,tempB ,tempC;
	IIC_Init();
		IIC_ADD_write(HMC5883L_Addr,0x00,0x70);
		IIC_ADD_write(HMC5883L_Addr,0x01,0x20);
		IIC_ADD_write(HMC5883L_Addr,0x02,0x00);

		tempA= IIC_ADD_read(HMC5883L_Addr, 0x0A);
		tempA= IIC_ADD_read(HMC5883L_Addr, 0x0B);
		tempA= IIC_ADD_read(HMC5883L_Addr, 0x0C);
   return 1; 		 
}



//******************************************************
//��������HMC5883�ڲ��Ƕ����ݣ���ַ��Χ0x3~0x5
//******************************************************
static void read_HMC5883L(void)
{
//		u8 i=0;

	HMC5883L_buffer[0]= IIC_ADD_read(HMC5883L_Addr, 0x03);
	HMC5883L_buffer[1]= IIC_ADD_read(HMC5883L_Addr, 0x04);
	
	HMC5883L_buffer[2]= IIC_ADD_read(HMC5883L_Addr, 0x05);
	HMC5883L_buffer[3]= IIC_ADD_read(HMC5883L_Addr, 0x06);
	
	HMC5883L_buffer[4]= IIC_ADD_read(HMC5883L_Addr, 0x07);
	HMC5883L_buffer[5]= IIC_ADD_read(HMC5883L_Addr, 0x08);

	
//    IIC_Start();                          //��ʼ�ź�
//    IIC_Send_Byte(HMC5883L_Addr);                   //�����豸��ַ+д�ź�
//	  IIC_Wait_Ack();
//    IIC_Send_Byte(0x03);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
//	  IIC_Wait_Ack();
//    IIC_Start();                          //��ʼ�ź�
//    IIC_Send_Byte(HMC5883L_Addr+1);     //�����豸��ַ+���ź�
//	  IIC_Wait_Ack();
//	  for (i=0; i<6; i++)                   //������ȡ6����ַ���ݣ��洢��BUF
//    {
//        HMC5883L_buffer[i] = IIC_Read_Byte();          //BUF[0]�洢����
//        if (i == 5)
//           IIC_NAck();                   //���һ��������Ҫ��NOACK
//        else
//           IIC_Ack();                     //��ӦACK
//    }
//    IIC_Stop();                           //ֹͣ�ź�
//		__NOP();
		

	Magn_Ori_x = HMC5883L_buffer[0] << 8 | HMC5883L_buffer[1]; //Combine MSB and LSB of X Data output register;
	Magn_Ori_y = HMC5883L_buffer[4] << 8 | HMC5883L_buffer[5]; //Combine MSB and LSB of Y Data output register;
	Magn_Ori_z = HMC5883L_buffer[2] << 8 | HMC5883L_buffer[3]; //Combine MSB and LSB of Z Data output register;
	
}


void Multiple_Read_HMC5883L(void)
{      	
 	 u8 i;

	 static uint8_t filter_cnt=0;
	 int32_t temp1=0,temp2=0,temp3=0;
	
//    IIC_Start();                          //��ʼ�ź�
//    IIC_Send_Byte(HMC5883L_Addr);                   //�����豸��ַ+д�ź�
//	  IIC_Wait_Ack();
//    IIC_Send_Byte(0x03);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
//	  IIC_Wait_Ack();
//    IIC_Start();                          //��ʼ�ź�
//    IIC_Send_Byte(HMC5883L_Addr+1);     //�����豸��ַ+���ź�
//	  IIC_Wait_Ack();
//	  for (i=0; i<6; i++)                   //������ȡ6����ַ���ݣ��洢��BUF
//    {
//        BUF[i] = IIC_Read_Byte();          //BUF[0]�洢����
//        if (i == 5)
//           IIC_NAck();                   //���һ��������Ҫ��NOACK
//        else
//           IIC_Ack();                     //��ӦACK
//    }
//    IIC_Stop();                           //ֹͣ�ź�
//    delay_ms(2);

		read_HMC5883L();
		
		X_BUFF[filter_cnt] = Magn_Ori_x; //Combine MSB and LSB of X Data output register;
	  Y_BUFF[filter_cnt] = Magn_Ori_y; //Combine MSB and LSB of Y Data output register;
	  Z_BUFF[filter_cnt] = Magn_Ori_z; //Combine MSB and LSB of Z Data output register;
	  for(i=0;i<FILL_NUM;i++)  //10��ȵĻ����˲�
	  {
		   temp1 += X_BUFF[i];
		   temp2 += Y_BUFF[i];
		   temp3 += Z_BUFF[i];
	  }
	  Magn_x = ( temp1 / FILL_NUM - offset_mx ) * mx_scale;
	  Magn_y = ( temp2 / FILL_NUM - offset_my ) * my_scale;
	  Magn_z = ( temp3 / FILL_NUM - offset_mz ) * mz_scale;
		
	  filter_cnt++;
	  if(filter_cnt==FILL_NUM)	filter_cnt=0;
		
		
		if(Mag_CALIBRATED)
		{
			led3= 1;
			HMC5883L_Start_Calib();

				//ָʾ ���ڱ궨
			led3= 0;
			Mag_CALIBRATED= 0;
			HMC5883L_Save_Calib();
		}


}	   
 
/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883L_Start_Calib(void)
*��������:	   ��������Ʊ궨
���������     	
���������  ��
*******************************************************************************/
void HMC5883L_Start_Calib(void)
{
	HMC5883_calib++;//��ʼ�궨
	HMC5883_maxx = -4096;	//��ԭ���ı궨ֵ���
	HMC5883_maxy = -4096;
	HMC5883_maxz = -4096;
	HMC5883_minx = 4096;
	HMC5883_miny = 4096;
	HMC5883_minz = 4096;
	offset_mx = 0;
  offset_my = 0;
  offset_mz = 0;
	mx_scale = 1.0; 
  my_scale = 1.0;
  mz_scale = 1.0;
	
	
	TIM_Cmd(TIM7, ENABLE);
	TIM_Cmd(TIM5, DISABLE);
	HMC5883_cnt= 0;
	TIM_SetCounter(TIM7 ,0);
	do{
		led1= 0;
		read_HMC5883L();
					//У����Ч�Ļ� �ɼ��궨ֵ
		if(HMC5883_minx>Magn_Ori_x)HMC5883_minx=(int16_t)Magn_Ori_x;
		if(HMC5883_miny>Magn_Ori_y)HMC5883_miny=(int16_t)Magn_Ori_y;
		if(HMC5883_minz>Magn_Ori_z)HMC5883_minz=(int16_t)Magn_Ori_z;

		if(HMC5883_maxx<Magn_Ori_x)HMC5883_maxx=(int16_t)Magn_Ori_x;
		if(HMC5883_maxy<Magn_Ori_y)HMC5883_maxy=(int16_t)Magn_Ori_y;
		if(HMC5883_maxz<Magn_Ori_z)HMC5883_maxz=(int16_t)Magn_Ori_z;
		
	}while(HMC5883_cnt<= 30);

	TIM_Cmd(TIM7, DISABLE);
	TIM_Cmd(TIM5, ENABLE);
	HMC5883_cnt= 0;
	


}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5883L_Save_Calib(void)
*��������:	  ��������Ʊ궨ֵ ��Flash
���������     	
���������  ��
*******************************************************************************/
void HMC5883L_Save_Calib(void)
{
	//�������Ʊ궨ֵд�� Flash ����
	offset_mx = (HMC5883_maxx+HMC5883_minx)/2;
  offset_my = (HMC5883_maxy+HMC5883_miny)/2;
  offset_mz = (HMC5883_maxz+HMC5883_minz)/2;
	temp_mx_scale = mx_scale*100;
	temp_my_scale = my_scale*100;
	temp_mz_scale = mz_scale*100;
//	Config.dMx_scale = mx_scale;
//	Config.dMy_scale = my_scale;
//	Config.dMz_scale = mz_scale;
	AT24cxx_save_5883_offest();
	Mag_CALIBRATED=0; //�����궨

}





