#include<reg52.h>

#define INIT 0xFF
#define TRUE 0x00
#define FALSE 0x01

#define uint unsigned int
#define uchar unsigned char
extern float temp_X,temp_Y,temp_Z;
extern unsigned char ReadAdxl345;

void Init_ADXL345(void);    
void conversion(uint temp_data);
void  Single_Write_ADXL345(uchar REG_Address,uchar REG_data);   //����д������
uchar Single_Read_ADXL345(uchar REG_Address);                   //������ȡ�ڲ��Ĵ�������
void  Multiple_Read_ADXL345(void);                                  //�����Ķ�ȡ�ڲ��Ĵ�������
void Delay5us();
void Delay5ms();
void ADXL345_Start();
void ADXL345_Stop();
void ADXL345_SendACK(uchar ack);
uchar  ADXL345_RecvACK();
void ADXL345_SendByte(uchar dat);
uchar ADXL345_RecvByte();
void ADXL345_ReadPage();
void ADXL345_WritePage();
void ReadData_x();
