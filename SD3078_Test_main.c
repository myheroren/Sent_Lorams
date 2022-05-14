/**
  ******************************************************************************
  * @file    main.c
  * @author  FAE
  * @version V1.0
  * @date    2018-04-10
  * @brief   RTC���ԣ����ģ��I2C��
  ******************************************************************************
  * @attention
  *
  * Website: http://www.whwave.com.cn
  * E-mail : fae@whwave.com.cn
  * Tel    : 0755-83114387
  *
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "bsp_usart1.h"
#include "rtc.h"
#include <string.h>

/****************** ���ر��� **********************/
u8	data_Buf[8];
u8	VBAT_Buf[2];
u16	VBAT_VAL;
Time_Def time_init={0x01,0x19,0x17,0x05,0x17,0x06,0x16};	//��ʼ��ʵʱʱ��
Time_Def Alarm_init={0x30,0x19,0x17,0x00,0x15,0x08,0x18};	//��ʼ������ʱ��
Time_Def sysTime;

/***************** ���غ������� *******************/
void Delay(u32 nCount);

/**
  * @brief  ������
  * @param  ��  
  * @retval ��
  */
int main(void)
{	
	u8 i;
	CountDown_Def cdInit;
	
/********* USART1 ����ģʽΪ 115200 8-N-1**********/
	USARTx_Config();
	printf("\r\n--����һ����дRTC����ʾ����-- \r\n");	
	
/***************** I2C���߳�ʼ�� *****************/	
	IIC_Init();	
	
/************* ��RTCд���ʱ�ĳ�ʼʱ�� ************/		
	RTC_WriteDate(&time_init);
	RTC_ReadDate(&sysTime);
	printf("д��ĳ�ʼʱ��Ϊ�� \r\n");
	printf("%02X-%02X-%02X  %02X:%02X:%02X  ����%02X\r\n\r\n", \
	sysTime.year, sysTime.month, sysTime.day, sysTime.hour,\
	sysTime.minute, sysTime.second, sysTime.week);	
	printf("----------------------------\r\n\r\n");	

#ifdef SD30XX_25XX	
/*************** ����оƬ��ID����ʾ ****************/	
	I2CReadSerial(RTC_Address,ID_Address,8,data_Buf);
	printf("оƬ64bit ID��Ϊ��\r\n");
	for (i=0; i<8; i++)
	printf("0x%02X ", data_Buf[i]);
	printf("\r\n----------------------------\r\n\r\n");

/**************** ���ʹ��������ʾ *****************/
	data_Buf[0] = Chg_enable;
	I2CWriteSerial(RTC_Address,Chg_MG,1,data_Buf);

/************** �����󱸵�ص�ѹ��ʾ ***************/	
	I2CReadSerial(RTC_Address,Bat_H8,2,VBAT_Buf);
	VBAT_VAL = (VBAT_Buf[0]>>7)*255 + VBAT_Buf[1];
	printf("VBAT�ŵĵ�ص�ѹΪ��%d.%d%dV\r\n\r\n", VBAT_VAL/100, VBAT_VAL%100/10, VBAT_VAL%10);
#endif

#if (INT_TYPE == FREQUENCY)
/**************** Ƶ���ж�������ʾ ****************/
	SetFrq(F2Hz);//���2HzƵ�ʷ���
#elif (INT_TYPE == ALARM)
/**************** �����ж�������ʾ ****************/
	Set_Alarm(sec_ALM|min_ALM|hor_ALM, &Alarm_init);//����ʹ�ܣ�ʱ���֡���
#elif (INT_TYPE == COUNTDOWN)	
/*************** ����ʱ�ж�������ʾ ***************/
	cdInit.IM = 1;				//����Ϊ�������ж�
	cdInit.d_clk = S_1s;		//����ʱ�ж�Դѡ��1s
	cdInit.init_val = 2;	//����ʱ��ֵ����Ϊ2
	Set_CountDown(&cdInit);
#elif (INT_TYPE == DISABLE)	
/****************** ��ֹ�ж���ʾ *****************/
	ClrINT(INTDE|INTAE|INTFE);
#endif

	while( 1 )
	{
		RTC_ReadDate(&sysTime);
		printf("%02X-%02X-%02X  %02X:%02X:%02X  ����%02X\r\n", \
		sysTime.year, sysTime.month, sysTime.day, sysTime.hour,\
		sysTime.minute, sysTime.second, sysTime.week);

		Delay(0x7FFFFF);//1����ʱ
	}
}

/**
  * @brief  ��ʱ����
  * @param  nCount
  * @retval ��
  */
void Delay(u32 nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/
