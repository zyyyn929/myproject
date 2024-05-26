#include <string.h>
#include "TOF.h"
#include "Delay.h"
#include "Serial.h"
#include "sys.h"
#include "LED.h"


u8 send_buf[18]={'\0'};
uint8_t Rx_buffer_temp[18];
uint8_t Rx_buffer_ok[18];
uint8_t Sensor_Data[18];

//计算检验和
u8 countsum(u8 *buf)
{
	u8 len = 0,i;
	u8 checksum =0;
	len = sizeof(buf)+1;
	while(len --)
	{
		checksum += *buf;
		buf++;
	}
	
	//保留最后两位
	checksum &=0xFF;
	
	return checksum;
}

//初始化配置SMD15
void SMD15_init(u8 bound)//要放在串口2初始化后面
{
	Delay_ms(200);//等待串口初始化完成
//	//停止扫描
//	stop_scan();
//	delay_ms(1);
	SMD15_setstandard();//设置用标准数据的格式输出
//	SMD15_setScanfHZ(1);//设置扫描频率
//	delay_ms(5);
//	
////	SMD15_setbaudrate(bound);//设置波特率 设置成功后，重启雷达生效 一般不设置
//	
//	//开始扫描
	start_scan();
//	SMD15_setstandard();//设置用标准数据的格式输出
}

//停止扫描
void stop_scan(void)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x61;
	send_buf[3] = 0x00;
	send_buf[4] = 0x60;
	
	Serial_SendArray(USART2,send_buf,5);
	memset(send_buf,0,sizeof(send_buf));
}

//开始扫描
void start_scan(void)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x60;
	send_buf[3] = 0x00;
	send_buf[4] = 0x5F;
	
	Serial_SendArray(USART2,send_buf,5);
	memset(send_buf,0,sizeof(send_buf));
}

//设置用标准数据的格式输出
void SMD15_setstandard(void)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x67;
	send_buf[3] = 0x01;
	send_buf[4] = 0x00;
	send_buf[5] = 0x67;
	
	Serial_SendArray(USART2,send_buf,6);
	memset(send_buf,0,sizeof(send_buf));
}

//设置用pixhawk数据的格式输出-这种模式雷达直接输出测距信息到串口调试助手可以直接显示 
void SMD15_setpixhawk(void)//不使用
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x67;
	send_buf[3] = 0x01;
	send_buf[4] = 0x01;
	send_buf[5] = 0x68;
	
	Serial_SendArray(USART2,send_buf,6);
	memset(send_buf,0,sizeof(send_buf));
}

//设置波特率 
//230400、460800、512000、921600、1500000 分别对应代号 0-4（默认为 460800）
void SMD15_setbaudrate(u8 i)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x66;
	send_buf[3] = 0x01;
	
	switch(i)
	{
		case 0:send_buf[4] = 0x00;break;
		case 1:send_buf[4] = 0x01;break;
		case 2:send_buf[4] = 0x02;break;
		case 3:send_buf[4] = 0x03;break;
		case 4:send_buf[4] = 0x04;break;
		default :break;
	}
	send_buf[5] = countsum(send_buf);
	
	Serial_SendArray(USART2,send_buf,6);
	memset(send_buf,0,sizeof(send_buf));
	
}

//设置输出频率 
//10hz、100hz、200hz、500hz、1000hz、1800hz 输出频率，分别对应代号 0-5（默认为100hz）
void SMD15_setScanfHZ(u8 hz)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x64;
	send_buf[3] = 0x01;
	
	switch(hz)
	{
		case 0:send_buf[4] = 0x00;break;
		case 1:send_buf[4] = 0x01;break;
		case 2:send_buf[4] = 0x02;break;
		case 3:send_buf[4] = 0x03;break;
		case 4:send_buf[4] = 0x04;break;
		case 5:send_buf[4] = 0x05;break;
		default :break;
	}
	send_buf[5] = countsum(send_buf);
	
	Serial_SendArray(USART2,send_buf,6);
	memset(send_buf,0,sizeof(send_buf));
}



//判断接收到的数据包是否完整
void SDM15_Decode(uint8_t RxData)
{
	static uint8_t RecCmd_Step=0,Checksum=0,RecCmd_Data_len=0,Data_cnt=0;
	switch(RecCmd_Step){
		case 0:
		if(RxData == 0xAA){
			Rx_buffer_temp[0]=RxData;
			RecCmd_Step++;
		}
		Checksum = 0xAA;
		break;
		
		case 1:
		if(RxData == 0x55){
			Rx_buffer_temp[1]=RxData;
			RecCmd_Step++;
			Data_cnt = 0;
			Checksum+=RxData;
			RecCmd_Data_len = 0;
		}
		else{
			RecCmd_Step = 0;
		}
		break;
		
		case 2:
			Rx_buffer_temp[2]=RxData;
			Checksum+=RxData;
			RecCmd_Step++;
		break;
		
		case 3:
			Rx_buffer_temp[3]=RxData;
			Checksum+=RxData;
			RecCmd_Data_len = Rx_buffer_temp[3];
			RecCmd_Step = RecCmd_Data_len==0 ? 5 : RecCmd_Step+1; 
		break;
		
		case 4:
		if(Data_cnt<RecCmd_Data_len){
			Rx_buffer_temp[4+Data_cnt++]=RxData;
			Checksum+=RxData;
			if(Data_cnt>=RecCmd_Data_len){
			RecCmd_Step++;
			}
		}
		break;
		
		case 5:
		if(Checksum==RxData){
			memcpy(Sensor_Data,&Rx_buffer_temp[0],18); 
			memset(Rx_buffer_temp,0,18);//清一下数据
			RecCmd_Step=0;
			Data_cnt = 0;
			Checksum = 0;
		}
		else{
			RecCmd_Step = 0;
			Data_cnt = 0;
			Checksum = 0;
			memset(Rx_buffer_temp,0,18);//清一下数据
		}
		break;
		
		default: break;
	}
}

//输出距离、强度、干扰
u8 print_message(void)
{
	u16 dis;
	u8  engy;
	u8  noise;
	if(Sensor_Data[0] == '\0') return 0;//没信息返回
	
	if(Sensor_Data[3]==0x00)//没数据长度也返回
	{
		memset(Sensor_Data,0,18);//清一下数据
		return 0; 
	}
	dis = Sensor_Data[5]<<8 | Sensor_Data[4];
	engy = Sensor_Data[6];
	noise = Sensor_Data[7];
	printf("dis = %d, engy = %d, noise = %d\r\n",dis,engy,noise);
	memset(Sensor_Data,0,18);//清一下数据
	LED1_Turn();
	return 1;
}
