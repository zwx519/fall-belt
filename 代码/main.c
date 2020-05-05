#include<reg52.h> //包含头文件，一般情况不需要改动，头文件包含特殊功能寄存器的定义
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

unsigned char ReadAdxl345;	   //定时读取adxl345数据
char displaytemp[16];		 //	 显示暂存 方便调试
unsigned long time_20ms=0; 	//	系统定时计数
unsigned long keyLater=3;	 //按键延时计数
unsigned int ReportLater=0; //上报延时
char Crtl_Z[1]={0x1a};	   //发送短信最后字节
unsigned char i; 

xdata unsigned char Lin0_No[13]="N:000.000000";//显示北纬
xdata unsigned char Lin1_Ea[12]="E:000.000000";//显示东经

unsigned char xdata	devide_flag;		        //GPS数据逗号分隔符
unsigned char xdata	speed_end;			//收速度数据结束标志
unsigned char xdata	dir_end;			//收方向角数据结束标志
unsigned char xdata  sysmode_GPS=FALSE;                    //gps有效无效标志
unsigned char xdata ew_flag;                        //东西标志
unsigned char xdata ns_flag;                        //南北标志

unsigned char xdata	gps_infor_weijing[17];    //暂存经纬度 格式是以度分秒的是形式
unsigned char xdata	gps_infor_speed[4];       //暂存速率
unsigned char xdata	gps_infor_time[6];        //时间暂存
unsigned char xdata	gps_infor_date[6];        //日期暂存
unsigned char xdata	gps_infor_dir[3];         //方向暂存

unsigned char xdata recv1_step=STAGE_SOHE;                       //串口接收指令步骤
unsigned char xdata uart1_r_buf;                       //串口的缓存
unsigned char xdata rev1_buf_busy;                    //串口接收忙碌标志
unsigned char xdata temp1_buf[85];                   //串口接收数组
unsigned int  xdata record1=0;   					//串口接收计数
unsigned char dealGpsMes = 0;
void Init_Timer0(void);	//函数声明
void uartSendStr(unsigned char *s,unsigned char length);
void UART_Init(void);
void uartSendByte(unsigned char dat);
void gpsDealfun(void);

void main (void)
{     
	static unsigned char ErrorNum=0;   //adxl345数据判断结果计数
	static unsigned char CheckNum=0;
	Init_Timer0();        //定时器0初始化
	UART_Init();
	LED=0;
	DelayMs(20);          //延时有助于稳定
	Init_ADXL345();	  //清屏
	if(Single_Read_ADXL345(0X00)==0xe5)	//读出的数据为0XE5,表示正确
	{
		DelayMs(5);
		uartSendStr("ready ok",8);//显示第二行
	}
	else
	{
		DelayMs(3);
	}

	for(i=0;i<10;i++)
	DelayMs(100);          //延时有助于稳定

	uartSendStr("AT+CMGF=1\r\n",11);	 //设置字符
	for(i=0;i<7;i++)
		DelayMs(100);          //延时有助于稳定

	buzzer=1;
	qinang=1;
	LED=1;
	ReportLater=0;
	while (1)         //主循环
	{
		if(dealGpsMes>2)   //gps信息处理
		{
			dealGpsMes=0;  //清除标志
			gpsDealfun();	   //gps信息处理函数
		}

	    if(ReadAdxl345==TRUE)   //定时读取adxl345数据
	    {
			ReadAdxl345=FALSE;
			ReadData_x();  						//三轴检测函数
			CheckNum++;
			if((temp_X<550)||(abs(temp_Y)>750))       //方位值判断 查看正常次数
			{
				ErrorNum++;
			}
			if(CheckNum>=5)	  	//进行5次处理
			{
				if(keyLater>=3)	  	//非按键下
				{
					if(ErrorNum>1)	   //角度出现错误 5次出息3次情况
					{
						buzzer=0;	   //打开蜂鸣器
						qinang=0;      //打开气囊
					}
					else
					{					
						buzzer=1;		//关闭蜂鸣器
						qinang=1;
						ReportLater=0;		//上报延时计数
					}					
				}
			
				ErrorNum=0;		  //清空滤波计数
				CheckNum=0;	
			}
	    }
          if(ReportLater>= 15)	//15s
		{
			LED=0;	
								
			uartSendStr("AT+CMGF=1\r\n",11);
			for(i=0;i<4;i++)
				DelayMs(100);          //延时有助于稳定

		    uartSendStr("AT+CSCS=\"GSM\"\r\n",15);
			for(i=0;i<4;i++)
				DelayMs(100);          //延时有助于稳定

			uartSendStr("AT+CMGS=\"+8613077317956\"\r\n",26); //可以修改电话号码
//			uartSendStr("AT+CMGS=\"+8613661848681\"\r\n",26); //可以修改电话号码
			for(i=0;i<3;i++)
				DelayMs(100);          //延时有助于稳定

			uartSendStr("Dangerous!",10);//发送危险型号
			uartSendStr(Lin0_No,12);//发送短信内容 在gps数据处理情况	
			uartSendStr(Lin1_Ea,12);
			DelayMs(100);
	        uartSendStr( (uchar *)Crtl_Z, 1);        //发送
			for(i=0;i<30;i++)
				DelayMs(100);          //延时有助于稳定
			ReportLater=0;
			LED=1;			   //上报晚程序 参数初始化
			keyLater=0;
		}
		if(key==0)		 //按键处理
		{
			DelayMs(10);  	//按键消抖动
			if(key==0)
			{
				ReportLater=0;	  //按键延时上报
				buzzer=1;		 //关闭蜂鸣器
				keyLater=0;		//按键延时处理
			}
		}
	}
}

void gpsDealfun(void)
{
	unsigned char num=0;
	unsigned long Mid_Du;       //中间变量 暂存经纬度的整数部分 即度
	unsigned long Mid_Fen;      //中间变量 暂存经纬度的小数部分 即分  gps原始数据是度分秒格式
	unsigned long Mid_Vale;     ////中间变量 暂存经纬度 并将其扩大了10000000倍

	if(sysmode_GPS==TRUE)		//检测gps是否有效数据
	{
		sysmode_GPS=FALSE;			//清除有效位

        Mid_Du=(gps_infor_weijing[0]-0x30)*10000000+(gps_infor_weijing[1]-0x30)*1000000;    //处理纬度 暂存整数部分扩大10000000
        
        Mid_Fen=(gps_infor_weijing[2]-0x30)*10000000+(gps_infor_weijing[3]-0x30)*1000000+
          (gps_infor_weijing[4]-0x30)*100000+(gps_infor_weijing[5]-0x30)*10000+
            (gps_infor_weijing[6]-0x30)*1000+(gps_infor_weijing[7]-0x30)*100;        //处理纬度 暂存小数部分扩大10000000       
        Mid_Fen=Mid_Fen/60;                                                     //分秒换算为小数位
        Mid_Vale=Mid_Du+Mid_Fen;        //最终纬度 格式为000.00000000 非度分秒格式
        Lin0_No[0]='N';                  
		Lin0_No[1]=':';                  
		Lin0_No[2]='0';                  
        Lin0_No[3]=Mid_Vale/10000000+0x30;                  //将处理后的纬度填入字符串 并打印显示
        Lin0_No[4]=(Mid_Vale/1000000)%10+0x30;
        Lin0_No[5]='.';
        Lin0_No[6]=(Mid_Vale/100000)%10+0x30;
        Lin0_No[7]=(Mid_Vale/10000)%10+0x30;
        Lin0_No[8]=(Mid_Vale/1000)%10+0x30;
        Lin0_No[9]=(Mid_Vale/100)%10+0x30;
        Lin0_No[10]=(Mid_Vale/10)%10+0x30;
        Lin0_No[11]=Mid_Vale%10+0x30;
        
        Mid_Du=(gps_infor_weijing[8]-0x30)*100000000+(gps_infor_weijing[9]-0x30)*10000000+(gps_infor_weijing[10]-0x30)*1000000; //处理经度 暂存整数部分扩大10000000
        
        Mid_Fen=(gps_infor_weijing[11]-0x30)*10000000+(gps_infor_weijing[12]-0x30)*1000000+
          (gps_infor_weijing[13]-0x30)*100000+(gps_infor_weijing[14]-0x30)*10000+
            (gps_infor_weijing[15]-0x30)*1000+(gps_infor_weijing[16]-0x30)*100; //处理经度 暂存小数部分扩大10000000       
        Mid_Fen=Mid_Fen/60;                                                //分秒换算为小数位
        Mid_Vale=Mid_Du+Mid_Fen;                                          //最终经度 格式为000.00000000 非度分秒格式
        Lin1_Ea[0]='E';                  
		Lin1_Ea[1]=':';     
        Lin1_Ea[2]=Mid_Vale/100000000+0x30;                            //将处理后的经度填入字符串 并打印显示
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
        Lin1_Ea[0]='G';               //无gps信号情况下 打印正在连接
		Lin1_Ea[1]='P';                  
		Lin1_Ea[2]='S';                  
        Lin1_Ea[3]=' ';                  /*将处理后的纬度填入字符串 并打印显示*/
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
	TMOD |= 0x01;	  //使用模式1，16位定时器，使用"|"符号可以在使用多个定时器时不受影响		     
	TH0=(65536-20000)/256;		  //重新赋值 20ms
	TL0=(65536-20000)%256;
	EA=1;            //总中断打开
	ET0=1;           //定时器中断打开
	TR0=1;           //定时器开关打开
}

void Timer0_isr(void) interrupt 1 
{
	TH0=(65536-20000)/256;		  //重新赋值 20ms
	TL0=(65536-20000)%256;
	time_20ms++;
	if(time_20ms%10==0)		//定时读取adxl345数据
	{
		ReadAdxl345=TRUE;
	}
	if(time_20ms%50==0)		  //延时定时上报
	{
		ReportLater++;
		keyLater++;			//按键延时上报计数
		dealGpsMes++;
	}

}

void UART_Init(void)
{
    SCON  = 0x50;		        // SCON: 模式 1, 8-bit UART, 使能接收  
    TMOD |= 0x20;               // TMOD: timer 1, mode 2, 8-bit 重装
    TH1   = 0xFD;               // TH1:  重装值 9600 波特率 晶振 11.0592MHz
	TL1 = TH1;  
    TR1   = 1;                  // TR1:  timer 1 打开                         
    EA    = 1;                  //打开总中断
    ES    = 1;                  //打开串口中断
}

void uartSendByte(unsigned char dat)//串口发送单字节数据
{
	unsigned char time_out;
	time_out=0x00;
	SBUF = dat;			  //将数据放入SBUF中
	while((!TI)&&(time_out<100))  //检测是否发送出去
	{time_out++;DelayUs2x(10);}	//未发送出去 进行短暂延时
	TI = 0;						//清除ti标志
}

void uartSendStr(unsigned char *s,unsigned char length)	   //发送定长度字符串
{
	unsigned char NUM;
	NUM=0x00;
	while(NUM<length)	//发送长度对比
	{
		uartSendByte(*s);  //放松单字节数据
		s++;		  //指针++
		NUM++;		  //下一个++
  	 }
}

void UART_SER (void) interrupt 4 //串行中断服务程序
{
    if(RI)                        //判断是接收中断产生
    {
	  RI=0;                      //标志位清零
	  uart1_r_buf=SBUF;                      //提取buf中的值
	  rev1_buf_busy=0x00;                         //判别 放置break问题
	  switch(recv1_step)
	  {
	  case STAGE_SOHE: if(uart1_r_buf == '$')     //判断接收到了$  具体原因参考GPS标准协议NMEA0183
	  {
	    rev1_buf_busy=0x01;
	    if(uart1_r_buf == '$')              //再次查看接收的是否是$
	    {
	      recv1_step=STAGE_TYPE;            //跳转到下一步
	      record1=0;                        //计数清零
	    }
	    else
	    {
	      recv1_step=STAGE_SOHE;        //恢复初始化
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
	    {                                                 //查看$GPRMC开头的命令行
	      if((temp1_buf[0] == 'G') && (temp1_buf[1] == 'P') && (temp1_buf[2] == 'R') && (temp1_buf[3] == 'M') && (temp1_buf[4] == 'C'))
	      {
	        recv1_step=STAGE_NONE;    //跳转到下一步
	        record1=0;
	      } 
	      else
	      {
	        recv1_step=STAGE_SOHE;//恢复初始化
	        record1=0;
	      }
	    }
	  }
	  break;
	  case STAGE_NONE: if(rev1_buf_busy == 0x00)//接收命令格式：$GPRMC,054347.00,A,3202.04770,N,11846.23632,E,0.000,0.00,221013,,,A*67
	  {
	    rev1_buf_busy=0x01;
	    record1++;
	    if((record1 > 0x01) && (record1 < 0x08))                                                                                    
	    {
	      gps_infor_time[record1-2]=uart1_r_buf;			//时间存储						
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
	      if(uart1_r_buf == 'A')  //gps收到数据 且有效
	      { 
	        recv1_step=STAGE_DATA;    //跳转到下一步
	      }
	      else
	      {
	        sysmode_GPS=FALSE;
	        recv1_step=STAGE_SOHE;    //无效恢复初始化
	        record1=0;
	      }
	    }
	  }
	  break;
	  case STAGE_DATA:  if(rev1_buf_busy == 0x00)
	  {
	    rev1_buf_busy=0x01;
	    record1++;
	    if(uart1_r_buf == ',')    //判断逗号
	    { 
	      devide_flag++;      //逗号次数记录
	      record1=0;
	    }
	    if(devide_flag == 3)
	    {
	      if((record1 > 0) && (record1 < 5))
	      {
	        gps_infor_weijing[record1-1]=uart1_r_buf;	    //存储经纬度 此处为纬度							
	      }
	      if((record1 > 5) && (record1 < 10))             //跳过小数点的存储
	      {
	        gps_infor_weijing[record1-2]=uart1_r_buf;	   //存储经纬度 此处为纬度															
	      }
	    }
	    if(devide_flag == 4)
	    {
	      if(record1 > 0)
	      {
	        ns_flag=uart1_r_buf;            //接收纬度NS标志
	      }
	    }
	    if(devide_flag == 5)
	    {
	      if((record1 > 0) && (record1 < 6))
	      {
	        gps_infor_weijing[record1+7]=uart1_r_buf;	  //存储经纬度 此处为纬度										
	      }
	      if((record1 > 6) && (record1 < 11))                //跳过小数点的存储
	      {
	        gps_infor_weijing[record1+6]=uart1_r_buf;       //存储经纬度 此处为经度																			
	      }
	    }
	    if(devide_flag == 6)
	    {
	      if(record1 > 0)
	      {
	        ew_flag=uart1_r_buf;            //经度度 EW标志
	      }
	    }
	    if(devide_flag == 7)
	    {
	      if(speed_end == 0x00)
	      {
	        if((record1 > 0) && (uart1_r_buf != '.'))
	        {
	          gps_infor_speed[record1-1]=uart1_r_buf;   //接收速率
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
	          gps_infor_dir[record1-1]=uart1_r_buf;   //存储方向
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
	      recv1_step=STAGE_SOHE;    //接收完成 并信号确定 
	      record1=0;                //恢复初始化状态 为下一次准备
	      devide_flag=0;
	      sysmode_GPS=TRUE;         //置位 GPS 信号为正确
	    }
	  }
	  break;												  
	  }
	}
   if(TI)  //如果是发送标志位，清零
	TI=0;
}
 


