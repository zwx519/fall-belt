#include<reg52.h> //����ͷ�ļ���һ���������Ҫ�Ķ���ͷ�ļ��������⹦�ܼĴ����Ķ���
#include<stdio.h>
#include "delay.h"
#include "math.h"
#include "adxl345.h"

#define STAGE_SOHE  0x01
#define STAGE_TYPE  0x02
#define STAGE_NONE  0x03
#define STAGE_DATA  0x04

sbit buzzer=P2^1;
sbit LED=P2^2;
sbit key=P2^0;
sbit qinang=P2^4;

unsigned char ReadAdxl345;	   //��ʱ��ȡadxl345����
char displaytemp[16];		 //	 ��ʾ�ݴ� �������
unsigned long time_20ms=0; 	//	ϵͳ��ʱ����
unsigned long keyLater=3;	 //������ʱ����
unsigned int ReportLater=0; //�ϱ���ʱ
char Crtl_Z[1]={0x1a};	   //���Ͷ�������ֽ�
unsigned char i; 

xdata unsigned char Lin0_No[13]="N:000.000000";//��ʾ��γ
xdata unsigned char Lin1_Ea[12]="E:000.000000";//��ʾ����

unsigned char xdata	devide_flag;		        //GPS���ݶ��ŷָ���
unsigned char xdata	speed_end;			//���ٶ����ݽ�����־
unsigned char xdata	dir_end;			//�շ�������ݽ�����־
unsigned char xdata  sysmode_GPS=FALSE;                    //gps��Ч��Ч��־
unsigned char xdata ew_flag;                        //������־
unsigned char xdata ns_flag;                        //�ϱ���־

unsigned char xdata	gps_infor_weijing[17];    //�ݴ澭γ�� ��ʽ���Զȷ��������ʽ
unsigned char xdata	gps_infor_speed[4];       //�ݴ�����
unsigned char xdata	gps_infor_time[6];        //ʱ���ݴ�
unsigned char xdata	gps_infor_date[6];        //�����ݴ�
unsigned char xdata	gps_infor_dir[3];         //�����ݴ�

unsigned char xdata recv1_step=STAGE_SOHE;                       //���ڽ���ָ���
unsigned char xdata uart1_r_buf;                       //���ڵĻ���
unsigned char xdata rev1_buf_busy;                    //���ڽ���æµ��־
unsigned char xdata temp1_buf[85];                   //���ڽ�������
unsigned int  xdata record1=0;   					//���ڽ��ռ���
unsigned char dealGpsMes = 0;
void Init_Timer0(void);	//��������
void uartSendStr(unsigned char *s,unsigned char length);
void UART_Init(void);
void uartSendByte(unsigned char dat);
void gpsDealfun(void);

void main (void)
{     
	static unsigned char ErrorNum=0;   //adxl345�����жϽ������
	static unsigned char CheckNum=0;
	Init_Timer0();        //��ʱ��0��ʼ��
	UART_Init();
	LED=0;
	DelayMs(20);          //��ʱ�������ȶ�
	Init_ADXL345();	  //����
	if(Single_Read_ADXL345(0X00)==0xe5)	//����������Ϊ0XE5,��ʾ��ȷ
	{
		DelayMs(5);
		uartSendStr("ready ok",8);//��ʾ�ڶ���
	}
	else
	{
		DelayMs(3);
	}

	for(i=0;i<10;i++)
	DelayMs(100);          //��ʱ�������ȶ�

	uartSendStr("AT+CMGF=1\r\n",11);	 //�����ַ�
	for(i=0;i<7;i++)
		DelayMs(100);          //��ʱ�������ȶ�

	buzzer=1;
	qinang=1;
	LED=1;
	ReportLater=0;
	while (1)         //��ѭ��
	{
		if(dealGpsMes>2)   //gps��Ϣ����
		{
			dealGpsMes=0;  //�����־
			gpsDealfun();	   //gps��Ϣ������
		}

	    if(ReadAdxl345==TRUE)   //��ʱ��ȡadxl345����
	    {
			ReadAdxl345=FALSE;
			ReadData_x();  						//�����⺯��
			CheckNum++;
			if((temp_X<550)||(abs(temp_Y)>750))       //��λֵ�ж� �鿴��������
			{
				ErrorNum++;
			}
			if(CheckNum>=5)	  	//����5�δ���
			{
				if(keyLater>=3)	  	//�ǰ�����
				{
					if(ErrorNum>1)	   //�Ƕȳ��ִ��� 5�γ�Ϣ3�����
					{
						buzzer=0;	   //�򿪷�����
						qinang=0;      //������
					}
					else
					{					
						buzzer=1;		//�رշ�����
						qinang=1;
						ReportLater=0;		//�ϱ���ʱ����
					}					
				}
			
				ErrorNum=0;		  //����˲�����
				CheckNum=0;	
			}
	    }
          if(ReportLater>= 15)	//15s
		{
			LED=0;	
								
			uartSendStr("AT+CMGF=1\r\n",11);
			for(i=0;i<4;i++)
				DelayMs(100);          //��ʱ�������ȶ�

		    uartSendStr("AT+CSCS=\"GSM\"\r\n",15);
			for(i=0;i<4;i++)
				DelayMs(100);          //��ʱ�������ȶ�

			uartSendStr("AT+CMGS=\"+8613077317956\"\r\n",26); //�����޸ĵ绰����
//			uartSendStr("AT+CMGS=\"+8613661848681\"\r\n",26); //�����޸ĵ绰����
			for(i=0;i<3;i++)
				DelayMs(100);          //��ʱ�������ȶ�

			uartSendStr("Dangerous!",10);//����Σ���ͺ�
			uartSendStr(Lin0_No,12);//���Ͷ������� ��gps���ݴ������	
			uartSendStr(Lin1_Ea,12);
			DelayMs(100);
	        uartSendStr( (uchar *)Crtl_Z, 1);        //����
			for(i=0;i<30;i++)
				DelayMs(100);          //��ʱ�������ȶ�
			ReportLater=0;
			LED=1;			   //�ϱ������ ������ʼ��
			keyLater=0;
		}
		if(key==0)		 //��������
		{
			DelayMs(10);  	//����������
			if(key==0)
			{
				ReportLater=0;	  //������ʱ�ϱ�
				buzzer=1;		 //�رշ�����
				keyLater=0;		//������ʱ����
			}
		}
	}
}

void gpsDealfun(void)
{
	unsigned char num=0;
	unsigned long Mid_Du;       //�м���� �ݴ澭γ�ȵ��������� ����
	unsigned long Mid_Fen;      //�м���� �ݴ澭γ�ȵ�С������ ����  gpsԭʼ�����Ƕȷ����ʽ
	unsigned long Mid_Vale;     ////�м���� �ݴ澭γ�� ������������10000000��

	if(sysmode_GPS==TRUE)		//���gps�Ƿ���Ч����
	{
		sysmode_GPS=FALSE;			//�����Чλ

        Mid_Du=(gps_infor_weijing[0]-0x30)*10000000+(gps_infor_weijing[1]-0x30)*1000000;    //����γ�� �ݴ�������������10000000
        
        Mid_Fen=(gps_infor_weijing[2]-0x30)*10000000+(gps_infor_weijing[3]-0x30)*1000000+
          (gps_infor_weijing[4]-0x30)*100000+(gps_infor_weijing[5]-0x30)*10000+
            (gps_infor_weijing[6]-0x30)*1000+(gps_infor_weijing[7]-0x30)*100;        //����γ�� �ݴ�С����������10000000       
        Mid_Fen=Mid_Fen/60;                                                     //���뻻��ΪС��λ
        Mid_Vale=Mid_Du+Mid_Fen;        //����γ�� ��ʽΪ000.00000000 �Ƕȷ����ʽ
        Lin0_No[0]='N';                  
		Lin0_No[1]=':';                  
		Lin0_No[2]='0';                  
        Lin0_No[3]=Mid_Vale/10000000+0x30;                  //��������γ�������ַ��� ����ӡ��ʾ
        Lin0_No[4]=(Mid_Vale/1000000)%10+0x30;
        Lin0_No[5]='.';
        Lin0_No[6]=(Mid_Vale/100000)%10+0x30;
        Lin0_No[7]=(Mid_Vale/10000)%10+0x30;
        Lin0_No[8]=(Mid_Vale/1000)%10+0x30;
        Lin0_No[9]=(Mid_Vale/100)%10+0x30;
        Lin0_No[10]=(Mid_Vale/10)%10+0x30;
        Lin0_No[11]=Mid_Vale%10+0x30;
        
        Mid_Du=(gps_infor_weijing[8]-0x30)*100000000+(gps_infor_weijing[9]-0x30)*10000000+(gps_infor_weijing[10]-0x30)*1000000; //������ �ݴ�������������10000000
        
        Mid_Fen=(gps_infor_weijing[11]-0x30)*10000000+(gps_infor_weijing[12]-0x30)*1000000+
          (gps_infor_weijing[13]-0x30)*100000+(gps_infor_weijing[14]-0x30)*10000+
            (gps_infor_weijing[15]-0x30)*1000+(gps_infor_weijing[16]-0x30)*100; //������ �ݴ�С����������10000000       
        Mid_Fen=Mid_Fen/60;                                                //���뻻��ΪС��λ
        Mid_Vale=Mid_Du+Mid_Fen;                                          //���վ��� ��ʽΪ000.00000000 �Ƕȷ����ʽ
        Lin1_Ea[0]='E';                  
		Lin1_Ea[1]=':';     
        Lin1_Ea[2]=Mid_Vale/100000000+0x30;                            //�������ľ��������ַ��� ����ӡ��ʾ
        Lin1_Ea[3]=(Mid_Vale/10000000)%10+0x30;
        Lin1_Ea[4]=(Mid_Vale/1000000)%10+0x30;
        Lin1_Ea[5]='.';
        Lin1_Ea[6]=(Mid_Vale/100000)%10+0x30;
        Lin1_Ea[7]=(Mid_Vale/10000)%10+0x30;
        Lin1_Ea[8]=(Mid_Vale/1000)%10+0x30;
        Lin1_Ea[9]=(Mid_Vale/100)%10+0x30;
        Lin1_Ea[10]=(Mid_Vale/10)%10+0x30;
        Lin1_Ea[11]=Mid_Vale%10+0x30;
       
	}
	else
	{
        Lin1_Ea[0]='G';               //��gps�ź������ ��ӡ��������
		Lin1_Ea[1]='P';                  
		Lin1_Ea[2]='S';                  
        Lin1_Ea[3]=' ';                  /*��������γ�������ַ��� ����ӡ��ʾ*/
        Lin1_Ea[4]='L';
        Lin1_Ea[5]='I';
        Lin1_Ea[6]='N';
        Lin1_Ea[7]='K';
        Lin1_Ea[8]='I';
        Lin1_Ea[9]='N';
        Lin1_Ea[10]='G';
        Lin1_Ea[11]='.';Lin1_Ea[12]='.';Lin1_Ea[13]='.';
		for(i=0;i<14;i++)
		{
			Lin0_No[i]=Lin1_Ea[i];
		}
	}
//	DelayMs(10);
//	SendStr(Lin0_No,12);
//	DelayMs(10);
//	SendStr(Lin1_Ea,12);
}



void Init_Timer0(void)
{
	TMOD |= 0x01;	  //ʹ��ģʽ1��16λ��ʱ����ʹ��"|"���ſ�����ʹ�ö����ʱ��ʱ����Ӱ��		     
	TH0=(65536-20000)/256;		  //���¸�ֵ 20ms
	TL0=(65536-20000)%256;
	EA=1;            //���жϴ�
	ET0=1;           //��ʱ���жϴ�
	TR0=1;           //��ʱ�����ش�
}

void Timer0_isr(void) interrupt 1 
{
	TH0=(65536-20000)/256;		  //���¸�ֵ 20ms
	TL0=(65536-20000)%256;
	time_20ms++;
	if(time_20ms%10==0)		//��ʱ��ȡadxl345����
	{
		ReadAdxl345=TRUE;
	}
	if(time_20ms%50==0)		  //��ʱ��ʱ�ϱ�
	{
		ReportLater++;
		keyLater++;			//������ʱ�ϱ�����
		dealGpsMes++;
	}

}

void UART_Init(void)
{
    SCON  = 0x50;		        // SCON: ģʽ 1, 8-bit UART, ʹ�ܽ���  
    TMOD |= 0x20;               // TMOD: timer 1, mode 2, 8-bit ��װ
    TH1   = 0xFD;               // TH1:  ��װֵ 9600 ������ ���� 11.0592MHz
	TL1 = TH1;  
    TR1   = 1;                  // TR1:  timer 1 ��                         
    EA    = 1;                  //�����ж�
    ES    = 1;                  //�򿪴����ж�
}

void uartSendByte(unsigned char dat)//���ڷ��͵��ֽ�����
{
	unsigned char time_out;
	time_out=0x00;
	SBUF = dat;			  //�����ݷ���SBUF��
	while((!TI)&&(time_out<100))  //����Ƿ��ͳ�ȥ
	{time_out++;DelayUs2x(10);}	//δ���ͳ�ȥ ���ж�����ʱ
	TI = 0;						//���ti��־
}

void uartSendStr(unsigned char *s,unsigned char length)	   //���Ͷ������ַ���
{
	unsigned char NUM;
	NUM=0x00;
	while(NUM<length)	//���ͳ��ȶԱ�
	{
		uartSendByte(*s);  //���ɵ��ֽ�����
		s++;		  //ָ��++
		NUM++;		  //��һ��++
  	 }
}

void UART_SER (void) interrupt 4 //�����жϷ������
{
    if(RI)                        //�ж��ǽ����жϲ���
    {
	  RI=0;                      //��־λ����
	  uart1_r_buf=SBUF;                      //��ȡbuf�е�ֵ
	  rev1_buf_busy=0x00;                         //�б� ����break����
	  switch(recv1_step)
	  {
	  case STAGE_SOHE: if(uart1_r_buf == '$')     //�жϽ��յ���$  ����ԭ��ο�GPS��׼Э��NMEA0183
	  {
	    rev1_buf_busy=0x01;
	    if(uart1_r_buf == '$')              //�ٴβ鿴���յ��Ƿ���$
	    {
	      recv1_step=STAGE_TYPE;            //��ת����һ��
	      record1=0;                        //��������
	    }
	    else
	    {
	      recv1_step=STAGE_SOHE;        //�ָ���ʼ��
	      record1=0;
	    }
	  }
	  break;
	  case STAGE_TYPE: if(rev1_buf_busy == 0x00)
	  {

	    rev1_buf_busy=0x01;
	    temp1_buf[record1]=uart1_r_buf;
	    record1++;
	    if(record1 == 0x05)
	    {                                                 //�鿴$GPRMC��ͷ��������
	      if((temp1_buf[0] == 'G') && (temp1_buf[1] == 'P') && (temp1_buf[2] == 'R') && (temp1_buf[3] == 'M') && (temp1_buf[4] == 'C'))
	      {
	        recv1_step=STAGE_NONE;    //��ת����һ��
	        record1=0;
	      } 
	      else
	      {
	        recv1_step=STAGE_SOHE;//�ָ���ʼ��
	        record1=0;
	      }
	    }
	  }
	  break;
	  case STAGE_NONE: if(rev1_buf_busy == 0x00)//���������ʽ��$GPRMC,054347.00,A,3202.04770,N,11846.23632,E,0.000,0.00,221013,,,A*67
	  {
	    rev1_buf_busy=0x01;
	    record1++;
	    if((record1 > 0x01) && (record1 < 0x08))                                                                                    
	    {
	      gps_infor_time[record1-2]=uart1_r_buf;			//ʱ��洢						
	    }
	    if((uart1_r_buf == ',') && (record1 > 0x07) && (record1 < 0x010))   //||((uart1_r_buf == ',') && (record1==0x02))
	    {
	      record1=0xcc;
	    }
	    if(record1 ==  0xcd)
	    {
	      record1=0;
	      devide_flag=2;
	      speed_end=0x00;
	      dir_end=0x00;
	      if(uart1_r_buf == 'A')  //gps�յ����� ����Ч
	      { 
	        recv1_step=STAGE_DATA;    //��ת����һ��
	      }
	      else
	      {
	        sysmode_GPS=FALSE;
	        recv1_step=STAGE_SOHE;    //��Ч�ָ���ʼ��
	        record1=0;
	      }
	    }
	  }
	  break;
	  case STAGE_DATA:  if(rev1_buf_busy == 0x00)
	  {
	    rev1_buf_busy=0x01;
	    record1++;
	    if(uart1_r_buf == ',')    //�ж϶���
	    { 
	      devide_flag++;      //���Ŵ�����¼
	      record1=0;
	    }
	    if(devide_flag == 3)
	    {
	      if((record1 > 0) && (record1 < 5))
	      {
	        gps_infor_weijing[record1-1]=uart1_r_buf;	    //�洢��γ�� �˴�Ϊγ��							
	      }
	      if((record1 > 5) && (record1 < 10))             //����С����Ĵ洢
	      {
	        gps_infor_weijing[record1-2]=uart1_r_buf;	   //�洢��γ�� �˴�Ϊγ��															
	      }
	    }
	    if(devide_flag == 4)
	    {
	      if(record1 > 0)
	      {
	        ns_flag=uart1_r_buf;            //����γ��NS��־
	      }
	    }
	    if(devide_flag == 5)
	    {
	      if((record1 > 0) && (record1 < 6))
	      {
	        gps_infor_weijing[record1+7]=uart1_r_buf;	  //�洢��γ�� �˴�Ϊγ��										
	      }
	      if((record1 > 6) && (record1 < 11))                //����С����Ĵ洢
	      {
	        gps_infor_weijing[record1+6]=uart1_r_buf;       //�洢��γ�� �˴�Ϊ����																			
	      }
	    }
	    if(devide_flag == 6)
	    {
	      if(record1 > 0)
	      {
	        ew_flag=uart1_r_buf;            //���ȶ� EW��־
	      }
	    }
	    if(devide_flag == 7)
	    {
	      if(speed_end == 0x00)
	      {
	        if((record1 > 0) && (uart1_r_buf != '.'))
	        {
	          gps_infor_speed[record1-1]=uart1_r_buf;   //��������
	        }
	        else if(uart1_r_buf == '.')
	        {
	          record1--;
	          speed_end=0xff;
	        }
	      }
	      else if(speed_end == 0xff)
	      {
	        speed_end=0xfe;
	        gps_infor_speed[record1-1]=uart1_r_buf;
	        gps_infor_speed[3]=gps_infor_speed[record1-1];
	        gps_infor_speed[2]=gps_infor_speed[record1-2];
	        if(record1 > 2)
	        {
	          gps_infor_speed[1]=gps_infor_speed[record1-3];
	        }
	        else
	        {
	          gps_infor_speed[1]=0x30;
	        }
	        if(record1 > 3)
	        {
	          gps_infor_speed[0]=gps_infor_speed[record1-4];
	        }
	        else
	        {
	          gps_infor_speed[0]=0x30;
	        }
	      }
	    }
	    if(devide_flag == 8)
	    {
	      if(dir_end == 0x00)
	      {
	        if((record1 > 0) && (uart1_r_buf != '.'))
	        {
	          gps_infor_dir[record1-1]=uart1_r_buf;   //�洢����
	        }
	        else if(uart1_r_buf == '.')
	        {
	          record1--;
	          dir_end=0xff;
	        }
	      }
	      else if(dir_end == 0xff)
	      {
	        dir_end=0xfe;
	        if(record1 == 2)
	        {
	          gps_infor_dir[2]=gps_infor_dir[record1-2];
	          gps_infor_dir[1]=0x30;
	          gps_infor_dir[0]=0x30;
	        }
	        if(record1 == 3)
	        {
	          gps_infor_dir[2]=gps_infor_dir[record1-2];
	          gps_infor_dir[1]=gps_infor_dir[record1-3];
	          gps_infor_dir[0]=0x30;
	        }
	      }
	    }
	    if(devide_flag == 9)
	    {
	      if((record1 > 0) && (record1 < 7))
	      {
	        gps_infor_date[record1-1]=uart1_r_buf;
	      }
	    }
	    if(uart1_r_buf == 0x0d)
	    {
	      recv1_step=STAGE_SOHE;    //������� ���ź�ȷ�� 
	      record1=0;                //�ָ���ʼ��״̬ Ϊ��һ��׼��
	      devide_flag=0;
	      sysmode_GPS=TRUE;         //��λ GPS �ź�Ϊ��ȷ
	    }
	  }
	  break;												  
	  }
	}
   if(TI)  //����Ƿ��ͱ�־λ������
	TI=0;
}
 


