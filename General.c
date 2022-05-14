#include "main.h"
#include <string.h>
#include <stdio.h>

#include "General.h"
#include "EEPROM_24Cxx.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "SX1278Lora.h"
#include "SX1278LoraHal.h"

uint16_t 	LED_FLash_Timer;
uint8_t 	LED_FLash_Timer_Flag;

uint16_t 	USART1_RX_STA = 0;   	 
uint16_t 	Uart1_Rcv_Overtime=0;
uint8_t 	Uart1_Recving_flag = 0;
uint8_t 	USART1_RCVDATA_Buf[1];
uint8_t 	USART1_RX_BUF[USART1_MAX_RECV_LEN];

uint32_t 	Uart1_OverTime;
uint8_t 	USART1_Start_Flag=0;

uint8_t RF_REC_RLEN_i = 0;
uint8_t Lora_RCV_Buffer[256]; //定义数组
uint16_t Lora_RCV_Status; 

LoRa_Config_Data LoRa_Data;
uint8_t SelfID[9];
uint16_t Hw_Ver,Sw_Ver;

void Send_Data_Packet(uint8_t Uart_X,uint8_t *Packet,uint16_t Data_len);   //发射数据
void Lora_Send_DataPacket(uint8_t *LoraTxBuffer,uint8_t Data_len);
void Respond_Uart_CMD(uint8_t *Respond_Data, uint8_t DataLen , uint8_t RspCode);
//========================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //
{ 
    if (htim->Instance == htim3.Instance)
    {
				LED_FLash_Timer++;
				Uart1_Rcv_Overtime++;
				Uart1_OverTime++;
					if(LED_FLash_Timer > 50)
					{
								HAL_GPIO_TogglePin(LED_SYS_RUN_GPIO_Port, LED_SYS_RUN_Pin);
								LED_FLash_Timer = 0;
								LED_FLash_Timer_Flag = !LED_FLash_Timer_Flag;
								HAL_IWDG_Refresh(&hiwdg);
					}	
					if(Uart1_OverTime > 10)  //超时
					{
							if(USART1_Start_Flag ==1)
							{
									USART1_RX_STA = USART1_RX_STA | 0x8000;
								  USART1_Start_Flag = 0;
							}
					}
		}    
}

//========================================================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)  	
  {
				Uart1_OverTime = 0;
				USART1_Start_Flag = 1;		
				USART1_RX_BUF[ USART1_RX_STA++] = USART1_RCVDATA_Buf[0];
  }
	 HAL_UART_Receive_IT(&huart1, USART1_RCVDATA_Buf, 1);
}

//========================================================================
void Send_Data_Packet(uint8_t Uart_X,uint8_t *Packet,uint16_t Data_len)   //发射数据	
{
		switch(Uart_X)
		{
				case 1:
							if(HAL_UART_Transmit(&huart1, Packet , Data_len, 10000) == 0)
							{
							}
						break;
		}
}
//========================================================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == Lora_DIO_0_Pin)  
		{
				#if NORMAL_MODE
					static int received_times = 0;
					u8 buf[TEMP_BUF_LEN];
					int received_data = 0;
					static int received_sum = 0;
				#endif
					if(DIO_0 == 1)   //确认触发
					{
								
								RF_EX0_STATUS = SX1278ReadBuffer(REG_LR_IRQFLAGS);		
								SX1278WriteBuffer(REG_LR_IRQFLAGS, 0xff); 						
								if(RF_EX0_STATUS > 0)  
								{
											if((RF_EX0_STATUS & 0x40) == 0x40)  //接收确认
											{
													CRC_Value = SX1278ReadBuffer(REG_LR_MODEMCONFIG2);
													Lora_RCV_Status = 0;
													if ((CRC_Value & 0x04) == 0x04) //CRC
													{
																SX1278WriteBuffer(REG_LR_FIFOADDRPTR, 0x00);
																SX1278_RLEN = SX1278ReadBuffer(REG_LR_NBRXBYTES);  
																
																SX1278_CSL;
																SX1278_1_ReadWrite(0);
																memset(Lora_RCV_Buffer, 0, sizeof(Lora_RCV_Buffer));
																for (RF_REC_RLEN_i = 0; RF_REC_RLEN_i < SX1278_RLEN; RF_REC_RLEN_i++)
																{
																			Lora_RCV_Buffer[RF_REC_RLEN_i] = SX1278_1_ReadWrite(0);
																}
																SX1278_CSH;
																Lora_RCV_Status= Lora_RCV_Status|0x8000|SX1278_RLEN;
													}
													SX1278LoRaSetOpMode(Stdby_mode);  												
													SX1278WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value); 	
													SX1278WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);		
													SX1278WriteBuffer(REG_LR_DIOMAPPING1, 0X00);							
													SX1278WriteBuffer(REG_LR_DIOMAPPING2, 0x00);							
													SX1278LoRaSetOpMode(Receiver_mode);												
											}
											else
											{
													if ((RF_EX0_STATUS & 0x08) == 0x08) //如果数据发送中断
													{
															SX1278LoRaSetOpMode(Stdby_mode);
															SX1278WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);  
														  SX1278WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);  
															SX1278WriteBuffer(REG_LR_DIOMAPPING1, 0X00);
															SX1278WriteBuffer(REG_LR_DIOMAPPING2, 0x00);
															SX1278LoRaSetOpMode(Receiver_mode);
													}
													SX1278WriteBuffer(REG_LR_IRQFLAGS, 0xff);
											}
							}
					}
    }
    if(GPIO_Pin == Lora_DIO_1_Pin)
		{
    }
    if(GPIO_Pin == Lora_DIO_2_Pin)
		{
    }
    if(GPIO_Pin == Lora_DIO_3_Pin)
		{
    }
    if(GPIO_Pin == Lora_DIO_4_Pin)
		{
    }
}
//========================================================================================================
void Lora_Send_DataPacket(uint8_t *LoraTxBuffer,uint8_t Data_len)
{
	SX1278WriteBuffer(REG_LR_IRQFLAGS, 0xff); 					
	FUN_RF_SENDPACKET(LoraTxBuffer, Data_len);  				
	HAL_Delay(1000);		
	SX1278WriteBuffer(REG_LR_IRQFLAGS, 0xff); 					
}
 
/****************************************************************
* Function:    Flash_EnableReadProtection
* Description: Enable the read protection of user flash area.
* Input:        NONE
* Output:        NONE
* Return:  NONE
*****************************************************************/

void Flash_EnableReadProtection(void)
{

  FLASH_OBProgramInitTypeDef OBInit;
  
  __HAL_FLASH_PREFETCH_BUFFER_DISABLE();
  
  HAL_FLASHEx_OBGetConfig(&OBInit);
  if(OBInit.RDPLevel == OB_RDP_LEVEL_0)
  {
    OBInit.OptionType = OPTIONBYTE_RDP;
    OBInit.RDPLevel = OB_RDP_LEVEL_1;
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    HAL_FLASHEx_OBProgram(&OBInit);
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
  }
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

}

//========================================================================================================
void UART_RXTX_TEST(void)  
{
			if(USART1_RX_STA > 0x8000)
			{
					Send_Data_Packet(1,USART1_RX_BUF,USART1_RX_STA&0x7FFF);
					USART1_RX_STA = 0;
			}
}
//========================================================================================================
uint8_t  vE2romTest(void)
{    
	uint8_t WriteBuffer[10],ReadBuffer[10];    
	uint16_t i;    
	for(i=0; i<10; i++)
	{	
			WriteBuffer[i]=i;
		  ReadBuffer[i] = 0;
	}    
	
		//==============  wrinte data to EEPROM    
		if(!writeAT24Cxx(0x05,WriteBuffer,10))            
			readAT24Cxx(0x05,ReadBuffer, 10);    
		
		if((WriteBuffer[0] == ReadBuffer[0]) && (WriteBuffer[5] == ReadBuffer[5]))
				return 1;
		else
				return 0;
}
//========================================================================================================
void	Load_LoRa_ConfigData_ID(void)
{
	uint8_t Temp_Data[21],i;
				readAT24Cxx(0x00,Temp_Data, 21);    

		LoRa_Data.Freq[0]= Temp_Data[0];
		LoRa_Data.Freq[1]= Temp_Data[1];
		LoRa_Data.Freq[2]= Temp_Data[2];
		
		LoRa_Data.SF = Temp_Data[3];
		LoRa_Data.BW = Temp_Data[4];
		LoRa_Data.Powr = Temp_Data[5];
		LoRa_Data.CR = Temp_Data[6];
	  
		//==== 提取本机ID =====
		for(i= 0 ;i<9;i++)
			SelfID[i] = Temp_Data[i+8];
	  
		//==== 提取软硬件版本=====
			Hw_Ver = Temp_Data[17];
			Hw_Ver = Hw_Ver << 8 | Temp_Data[18];
			
			Sw_Ver = Temp_Data[19];
			Sw_Ver = Sw_Ver << 8 | Temp_Data[20];
}
//========================================================================================================
void	Update_LoRa_ConfigData(	uint8_t *Temp_Data)  //EEPROM Add0-->Add7
{
	uint8_t i,RspDat[10];
		for(i=0;i<7;i++)
				RspDat[i] = Temp_Data[i+1];
		
		writeAT24Cxx(0x00,RspDat, 7);    
		LoRa_Data.Freq[0]= RspDat[0];
		LoRa_Data.Freq[1]= RspDat[1];
		LoRa_Data.Freq[2]= RspDat[2];
		LoRa_Data.SF  = RspDat[3];
		LoRa_Data.BW = RspDat[4];
		LoRa_Data.Powr= RspDat[5];
		LoRa_Data.CR = RspDat[6];
		Respond_Uart_CMD(RspDat,7,0x01);
}
//========================================================================================================
void	Get_LoRa_ConfigData(void)
{
	uint8_t Temp_Data[7];
		Temp_Data[0] = LoRa_Data.Freq[0];
		Temp_Data[1] = LoRa_Data.Freq[1];
		Temp_Data[2] = LoRa_Data.Freq[2];
		
		Temp_Data[3] = LoRa_Data.SF;
		Temp_Data[4] = LoRa_Data.BW;
		Temp_Data[5] = LoRa_Data.Powr;
		Temp_Data[6] = LoRa_Data.CR;
	
			Respond_Uart_CMD(Temp_Data,7,0x02);
}
//========================================================================================================
void	Update_LoRa_ID(uint8_t *Temp_Data)
{
	uint8_t i;
				for(i=0;i<9;i++)
						SelfID[i] = Temp_Data[i+1];

		writeAT24Cxx(0x08,SelfID, 9);    
		Respond_Uart_CMD(SelfID,9,0x03);
}
//========================================================================================================
void	Get_LoRa_ID(void)
{
		Respond_Uart_CMD(SelfID,9,0x04);
}
//========================================================================================================
void	Update_Ver(uint8_t *Temp_Data)
{
	uint8_t i,tp_data[8];
				for(i=0;i<8;i++)
						tp_data[i] = Temp_Data[i+1];
				writeAT24Cxx(17,tp_data, 4);    
				Hw_Ver = tp_data[0]<<8|tp_data[1];
				Sw_Ver = tp_data[2]<<8|tp_data[3];
		
		Respond_Uart_CMD(tp_data,4,0x05);
}
//========================================================================================================
void	Get_Ver(void)
{
		uint8_t temp_data[4];
			temp_data[0] = Hw_Ver>>8;
			temp_data[1] = Hw_Ver;
			temp_data[2] = Sw_Ver>>8;
			temp_data[3] = Sw_Ver;
	
		Respond_Uart_CMD(temp_data,4,0x06);
}
//========================================================================================================
void	Respond_Uart_CMD(uint8_t *Respond_Data, uint8_t DataLen , uint8_t RspCode)
{
		uint8_t Finial_Data[20],i;
		
		Finial_Data[0]= 0x54;
		Finial_Data[1]= 0x6A;
		Finial_Data[2]= 0x4A;
		Finial_Data[3]= 0x4E;
		Finial_Data[4]= 0x4A;
		Finial_Data[5]= 0x44;
		Finial_Data[6]= DataLen + 1; //后跟数据长度
		Finial_Data[7]= RspCode<<4 | 1; //应答功能号
	
		for(i= 0;i< DataLen;i++)
				Finial_Data[i+8] = Respond_Data[i];
		
		Send_Data_Packet(1,Finial_Data,DataLen+8);
}


//========================================================================================================
uint32_t McuID[3];
uint32_t GetMcuID(void)				//读取芯片的ID
{
		//获取CPU唯一ID
		McuID[0]=*(uint32_t*)(0x1ffff7e8);
		McuID[1]=*(uint32_t*)(0x1ffff7ec);
		McuID[2]=*(uint32_t*)(0x1ffff7f0);
		return 1;
}



