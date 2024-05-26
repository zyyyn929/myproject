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

//��������
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
	
	//���������λ
	checksum &=0xFF;
	
	return checksum;
}

//��ʼ������SMD15
void SMD15_init(u8 bound)//Ҫ���ڴ���2��ʼ������
{
	Delay_ms(200);//�ȴ����ڳ�ʼ�����
//	//ֹͣɨ��
//	stop_scan();
//	delay_ms(1);
	SMD15_setstandard();//�����ñ�׼���ݵĸ�ʽ���
//	SMD15_setScanfHZ(1);//����ɨ��Ƶ��
//	delay_ms(5);
//	
////	SMD15_setbaudrate(bound);//���ò����� ���óɹ��������״���Ч һ�㲻����
//	
//	//��ʼɨ��
	start_scan();
//	SMD15_setstandard();//�����ñ�׼���ݵĸ�ʽ���
}

//ֹͣɨ��
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

//��ʼɨ��
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

//�����ñ�׼���ݵĸ�ʽ���
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

//������pixhawk���ݵĸ�ʽ���-����ģʽ�״�ֱ����������Ϣ�����ڵ������ֿ���ֱ����ʾ 
void SMD15_setpixhawk(void)//��ʹ��
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

//���ò����� 
//230400��460800��512000��921600��1500000 �ֱ��Ӧ���� 0-4��Ĭ��Ϊ 460800��
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

//�������Ƶ�� 
//10hz��100hz��200hz��500hz��1000hz��1800hz ���Ƶ�ʣ��ֱ��Ӧ���� 0-5��Ĭ��Ϊ100hz��
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



//�жϽ��յ������ݰ��Ƿ�����
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
			memset(Rx_buffer_temp,0,18);//��һ������
			RecCmd_Step=0;
			Data_cnt = 0;
			Checksum = 0;
		}
		else{
			RecCmd_Step = 0;
			Data_cnt = 0;
			Checksum = 0;
			memset(Rx_buffer_temp,0,18);//��һ������
		}
		break;
		
		default: break;
	}
}

//������롢ǿ�ȡ�����
u8 print_message(void)
{
	u16 dis;
	u8  engy;
	u8  noise;
	if(Sensor_Data[0] == '\0') return 0;//û��Ϣ����
	
	if(Sensor_Data[3]==0x00)//û���ݳ���Ҳ����
	{
		memset(Sensor_Data,0,18);//��һ������
		return 0; 
	}
	dis = Sensor_Data[5]<<8 | Sensor_Data[4];
	engy = Sensor_Data[6];
	noise = Sensor_Data[7];
	printf("dis = %d, engy = %d, noise = %d\r\n",dis,engy,noise);
	memset(Sensor_Data,0,18);//��һ������
	LED1_Turn();
	return 1;
}
