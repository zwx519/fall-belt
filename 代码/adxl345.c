#include "adxl345.h"
#include  <INTRINS.H>

#define Lcd1602
sbit SCL=P1^2;
sbit SDA=P1^3;

#define SCL_OUT 	;//(P4DIR|=BIT2) //��λscl
#define SET_SCL SCL=1;//(P4OUT|=BIT2) //��λscl
#define CLE_SCL SCL=0;//(P4OUT&=~BIT2)//���scl

                    
#define SDA_OUT ;//(P4DIR|=BIT3)
#define SDA_INT ;//(P4DIR&=~BIT3)
#define SET_SDA SDA=1;//(P4OUT|=BIT3)//��λsda
#define CLE_SDA SDA=0;//(P4OUT&=~BIT3)//���sda
#define SDA_VAL SDA//(P4IN&BIT3)


#define	SlaveAddress   0xA6	  //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�                  //ALT  ADDRESS���Žӵ�ʱ��ַΪ0xA6���ӵ�Դʱ��ַΪ0x3A

unsigned char BUF[8];                         //�������ݻ�����      	
uchar ge,shi,bai,qian,wan;           //��ʾ����
unsigned char err;
float temp_X,temp_Y,temp_Z;

void Delay5us()
{
    _nop_();_nop_();_nop_();_nop_();
    _nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();
}
void Delay5ms()
{
    unsigned int n = 560;

    while (n--);
}

/**************************************
��ʼ�ź�
**************************************/
void ADXL345_Start()
{
    SDA = 1;                    //����������
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SDA = 0;                    //�����½���
    Delay5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
}

/**************************************
ֹͣ�ź�
**************************************/
void ADXL345_Stop()
{
    SDA = 0;                    //����������
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SDA = 1;                    //����������
    Delay5us();                 //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void ADXL345_SendACK(uchar ack)
{   
    SDA = ack;                  //дӦ���ź�
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
    Delay5us();                 //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
uchar ADXL345_RecvACK()
{
	bit CY00;
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    CY00 = SDA;                   //��Ӧ���ź�
    SCL = 0;                    //����ʱ����
    Delay5us();                 //��ʱ

    return CY00;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
void ADXL345_SendByte(unsigned char dat)
{
    unsigned char i;

    for (i=0; i<8; i++)         //8λ������
    {
         if(dat&0x80)	//dat <<= 1;              //�Ƴ����ݵ����λ
        {
			SDA = 1;               //�����ݿ�	 //		SDA = CY;               //�����ݿ�
		}
		else 
			SDA=0;
		dat <<= 1;
        SCL = 1;                //����ʱ����
        Delay5us();             //��ʱ
        SCL = 0;                //����ʱ����
        Delay5us();             //��ʱ
    }
    ADXL345_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
unsigned char ADXL345_RecvByte()
{
    unsigned char i;
    unsigned char dat = 0;

    SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL = 1;                //����ʱ����
        Delay5us();             //��ʱ
        dat |= SDA;             //������               
        SCL = 0;                //����ʱ����
        Delay5us();             //��ʱ
    }
    return dat;
}

//******���ֽ�д��*******************************************

void Single_Write_ADXL345(uchar REG_Address,uchar REG_data)
{
    ADXL345_Start();                  //��ʼ�ź�
    ADXL345_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
    ADXL345_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ����ο�����pdf22ҳ 
    ADXL345_SendByte(REG_data);       //�ڲ��Ĵ������ݣ���ο�����pdf22ҳ 
    ADXL345_Stop();                   //����ֹͣ�ź�
}

//********���ֽڶ�ȡ*****************************************
uchar Single_Read_ADXL345(uchar REG_Address)
{  uchar REG_data;
    ADXL345_Start();                          //��ʼ�ź�
    ADXL345_SendByte(SlaveAddress);           //�����豸��ַ+д�ź�
    ADXL345_SendByte(REG_Address);            //���ʹ洢��Ԫ��ַ����0��ʼ	
    ADXL345_Start();                          //��ʼ�ź�
    ADXL345_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
    REG_data=ADXL345_RecvByte();              //�����Ĵ�������
	ADXL345_SendACK(1);   
	ADXL345_Stop();                           //ֹͣ�ź�
    return REG_data; 
}
//*********************************************************
//
//��������ADXL345�ڲ����ٶ����ݣ���ַ��Χ0x32~0x37
//
//*********************************************************
void Multiple_Read_ADXL345(void)
{   uchar i;
    ADXL345_Start();                          //��ʼ�ź�
    ADXL345_SendByte(SlaveAddress);           //�����豸��ַ+д�ź�
    ADXL345_SendByte(0x32);                   //���ʹ洢��Ԫ��ַ����0x32��ʼ	
    ADXL345_Start();                          //��ʼ�ź�
    ADXL345_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
	 for (i=0; i<6; i++)                      //������ȡ6����ַ���ݣ��洢��BUF
    {
        BUF[i] = ADXL345_RecvByte();          //BUF[0]�洢0x32��ַ�е�����
        if (i == 5)
        {
           ADXL345_SendACK(1);                //���һ��������Ҫ��NOACK
        }
        else
        {
          ADXL345_SendACK(0);                //��ӦACK
       }
   }
    ADXL345_Stop();                          //ֹͣ�ź�
    Delay5ms();
}


//*****************************************************************

//��ʼ��ADXL345��������Ҫ��ο�pdf�����޸�************************
void Init_ADXL345()
{
   Single_Write_ADXL345(0x31,0x0B);   //������Χ,����16g��13λģʽ
   Single_Write_ADXL345(0x2C,0x08);   //�����趨Ϊ12.5 �ο�pdf13ҳ
   Single_Write_ADXL345(0x2D,0x08);   //ѡ���Դģʽ   �ο�pdf24ҳ
   Single_Write_ADXL345(0x2E,0x80);   //ʹ�� DATA_READY �ж�
   Single_Write_ADXL345(0x1E,0x00);   //X ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ
   Single_Write_ADXL345(0x1F,0x00);   //Y ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ
   Single_Write_ADXL345(0x20,0x05);   //Z ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ
}
//***********************************************************************
//��ʾx��
void ReadData_x()
{   
  int  dis_data;                       //����
  Multiple_Read_ADXL345();       	//�����������ݣ��洢��BUF��
  dis_data=(BUF[1]<<8)+BUF[0];  //�ϳ�����   
//  if(dis_data<0)
//  {
//    dis_data=-dis_data;
//  }
  temp_X=(float)dis_data*3.9;  //�������ݺ���ʾ,�鿼ADXL345�������ŵ�4ҳ
 
  dis_data=(BUF[3]<<8)+BUF[2];  //�ϳ�����   
//  if(dis_data<0)
//  {
//    dis_data=-dis_data;
//  }
  temp_Y=(float)dis_data*3.9;  //�������ݺ���ʾ,�鿼ADXL345�������ŵ�4ҳ

  dis_data=(BUF[5]<<8)+BUF[4];    //�ϳ�����   
//  if(dis_data<0)
//  {
//    dis_data=-dis_data;
//  }
  temp_Z=(float)dis_data*3.9;  //�������ݺ���ʾ,�鿼ADXL345�������ŵ�4ҳ
}