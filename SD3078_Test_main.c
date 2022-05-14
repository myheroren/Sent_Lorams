/**
  ******************************************************************************
  * @file    main.c
  * @author  FAE
  * @version V1.0
  * @date    2018-04-10
  * @brief   RTC测试（软件模拟I2C）
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

/****************** 本地变量 **********************/
u8	data_Buf[8];
u8	VBAT_Buf[2];
u16	VBAT_VAL;
Time_Def time_init={0x01,0x19,0x17,0x05,0x17,0x06,0x16};	//初始化实时时间
Time_Def Alarm_init={0x30,0x19,0x17,0x00,0x15,0x08,0x18};	//初始化报警时间
Time_Def sysTime;

/***************** 本地函数声明 *******************/
void Delay(u32 nCount);

/**
  * @brief  主函数
  * @param  无  
  * @retval 无
  */
int main(void)
{	
	u8 i;
	CountDown_Def cdInit;
	
/********* USART1 配置模式为 115200 8-N-1**********/
	USARTx_Config();
	printf("\r\n--这是一个读写RTC的演示程序-- \r\n");	
	
/***************** I2C总线初始化 *****************/	
	IIC_Init();	
	
/************* 给RTC写入计时的初始时间 ************/		
	RTC_WriteDate(&time_init);
	RTC_ReadDate(&sysTime);
	printf("写入的初始时间为： \r\n");
	printf("%02X-%02X-%02X  %02X:%02X:%02X  星期%02X\r\n\r\n", \
	sysTime.year, sysTime.month, sysTime.day, sysTime.hour,\
	sysTime.minute, sysTime.second, sysTime.week);	
	printf("----------------------------\r\n\r\n");	

#ifdef SD30XX_25XX	
/*************** 读出芯片的ID号演示 ****************/	
	I2CReadSerial(RTC_Address,ID_Address,8,data_Buf);
	printf("芯片64bit ID号为：\r\n");
	for (i=0; i<8; i++)
	printf("0x%02X ", data_Buf[i]);
	printf("\r\n----------------------------\r\n\r\n");

/**************** 充电使能设置演示 *****************/
	data_Buf[0] = Chg_enable;
	I2CWriteSerial(RTC_Address,Chg_MG,1,data_Buf);

/************** 读出后备电池电压演示 ***************/	
	I2CReadSerial(RTC_Address,Bat_H8,2,VBAT_Buf);
	VBAT_VAL = (VBAT_Buf[0]>>7)*255 + VBAT_Buf[1];
	printf("VBAT脚的电池电压为：%d.%d%dV\r\n\r\n", VBAT_VAL/100, VBAT_VAL%100/10, VBAT_VAL%10);
#endif

#if (INT_TYPE == FREQUENCY)
/**************** 频率中断设置演示 ****************/
	SetFrq(F2Hz);//输出2Hz频率方波
#elif (INT_TYPE == ALARM)
/**************** 报警中断设置演示 ****************/
	Set_Alarm(sec_ALM|min_ALM|hor_ALM, &Alarm_init);//报警使能：时、分、秒
#elif (INT_TYPE == COUNTDOWN)	
/*************** 倒计时中断设置演示 ***************/
	cdInit.IM = 1;				//设置为周期性中断
	cdInit.d_clk = S_1s;		//倒计时中断源选择1s
	cdInit.init_val = 2;	//倒计时初值设置为2
	Set_CountDown(&cdInit);
#elif (INT_TYPE == DISABLE)	
/****************** 禁止中断演示 *****************/
	ClrINT(INTDE|INTAE|INTFE);
#endif

	while( 1 )
	{
		RTC_ReadDate(&sysTime);
		printf("%02X-%02X-%02X  %02X:%02X:%02X  星期%02X\r\n", \
		sysTime.year, sysTime.month, sysTime.day, sysTime.hour,\
		sysTime.minute, sysTime.second, sysTime.week);

		Delay(0x7FFFFF);//1秒延时
	}
}

/**
  * @brief  延时函数
  * @param  nCount
  * @retval 无
  */
void Delay(u32 nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/
