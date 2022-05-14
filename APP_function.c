
#include "main.h"
#include <string.h>
#include <stdio.h>

#include "General.h"
//#include "FLASH_W25Qxxx.h"
//#include "EEPROM_24Cxx.h"
//#include "stm32f0xx_hal_uart.h"
//#include "SD3078_IIC.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "SX1278Lora.h"
#include "SX1278LoraHal.h"

void	UART_To_Lora_Tx(void);
void	Save_DataPacket_to_Buffer(uint8_t BufferItemNumber);	
void	Lora_Config_Function(void);
uint8_t CHeck_Lora_RCV_Packet(void);	
void	UART_CMD_Function(void);
uint16_t BufferItemNumber;			
uint16_t BufferItem_LastByte;		
uint8_t Simulate_645_DataPacket[120]={0xFE , 0xFE , 0xFE , 0xFE , 0x68 , 0x88 , 0x88 , 0x88 , 0x88 , 0x88 , 0x88 , 0x68 , 0x14 , 0x0E , 0x41 , 0x37 , 0x33 , 0x37 , 0x33 , 0x33 , 0x33 , 0x33 , 0xF7 , 0xF6 , 0xF5 , 0xF4 , 0x37 , 0x34 , 0x11 , 0x16};
//========================================================================================================
//===== Lora 接收到的指令处理=====
void Lora_Rcv_CMD_Run()
{
					//======== 转发 UART========
					Send_Data_Packet(1,Lora_RCV_Buffer,Lora_RCV_Status&0x7FFF);
}

//========================================================================================================
void UART_Data_Process(void)
{
			if(USART1_RX_STA > 0x8000)
			{
					if((USART1_RX_BUF[0]== 0x54)&& (USART1_RX_BUF[1]== 0x6A)&&(USART1_RX_BUF[2]== 0x4A)&&(USART1_RX_BUF[3]== 0x4E)&&(USART1_RX_BUF[4]== 0x4A)&&(USART1_RX_BUF[5]== 0x44))
					{
								UART_CMD_Function();
					}
					else 
					{
								UART_To_Lora_Tx();
					}
					USART1_RX_STA = 0;
			}
}
							
//========================================================================================================
void	UART_To_Lora_Tx(void)
{
	uint8_t i,DataTemp[200];
		//====== 填充 ==============
		for(i=0;i<(USART1_RX_STA & 0x7FFF); i++)
		{
				DataTemp[i] = USART1_RX_BUF[i];
		}
		//==== 发送======
		Lora_Send_DataPacket(DataTemp,i);
	
}
//========================================================================================================
void	UART_CMD_Function(void)
{
	uint8_t Temp_Data[16],i;
		//配置模块
			for(i=0;i<USART1_RX_BUF[6];i++)
					Temp_Data[i] = 	USART1_RX_BUF[7+i];
	  	
			switch(Temp_Data[0])
			{
					case 0x01:	//写入
								Update_LoRa_ConfigData(Temp_Data);
						break;
					case 0x02:	//读取
								Get_LoRa_ConfigData();
						break;
					case 0x03:	//写入
								Update_LoRa_ID(Temp_Data);
						break;
					case 0x04:	//读取
								Get_LoRa_ID();
						break;
					case 0x05:	//写入
								Update_Ver(Temp_Data);
						break;
					case 0x06:	//读取
								Get_Ver();
						break;
			}
}
//========================================================================================================
uint8_t CHeck_Lora_RCV_Packet(void)
{
	uint8_t i;
		for(i=0;i<9;i++)
				if(Lora_RCV_Buffer[i+1] != SelfID[i])
		if(i>8)
		{
			return 1;
		}
		return 0;
}

///=========================== END =============================

