
#include "SX1278Lora.h"
#include "SX1278LoraHal.h"
//#include "delay.h"
#include "spi.h"
#include "General.h"

/**************************************************************
//SPI时钟 10MHz
//SX1278: 137~525MHz 	6-12 	7.8~500kHz
**************************************************************/
extern LoRa_Config_Data LoRa_Data;

//====== 功率 ===========
unsigned char power_data[8]   	= {0X80, 0X80, 0X80, 0X83, 0X86, 0x89, 0x8c, 0x8f };
unsigned char powerValue      	= 7;		//功率值

//====== 频率 ==========
unsigned char Frequency[3]    	= {0x6c, 0x80, 0x00 };//
unsigned char Frequency2[3]    	= {0x75, 0x7F, 0x0A };//
unsigned char Frequency3[3]    	= {0x75, 0x80, 0x00 };//
unsigned char Frequency4[3]    	= {0x75, 0x80, 0xF6 };//


//======= 带宽 ==========
unsigned char Bw_Frequency    	= 7;     //6-9 		带宽

unsigned char SpreadingFactor 	= 10;    //
unsigned char CodingRate      	= 2;     //

unsigned char RF_EX0_STATUS;
unsigned char CRC_Value;
unsigned char SX1278_RLEN;


//========================================================================================================
unsigned char SX1278_1_ReadWrite(unsigned char writedat)
{
uint8_t	TxData[1];
uint8_t	RxData[1];
		TxData[0] = writedat;
    /* Send the read ID command */   
    HAL_SPI_TransmitReceive(&hspi1,TxData, RxData,1, 1000);  
		return RxData[0];
}
//========================================================================================================
void  SX1278_HardRest(void) //
{
	SX1278_RSTL;
	HAL_Delay(100);
	SX1278_RSTH;
	HAL_Delay(100);
}

//========================================================================================================
uint8_t  SX1278ReadBuffer(uint8_t addr) //读取
{
	uint8_t Value;
		SX1278_CSL;
		SX1278_1_ReadWrite(addr & 0x7F);
		Value = SX1278_1_ReadWrite(0);
		SX1278_CSH;
	return Value;
}

//========================================================================================================
void  SX1278WriteBuffer(uint8_t addr,uint8_t buffer) 
{
	SX1278_CSL; //NSS = 0;
	SX1278_1_ReadWrite(addr | 0x80);
	SX1278_1_ReadWrite(buffer);	 
	SX1278_CSH; //NSS = 1;
}

//========================================================================================================
void  SX1278LoRaSetOpMode(RFMode_SET opMode)  //工作模式设定
{
	uint8_t opModePrev;
		opModePrev = SX1278ReadBuffer(REG_LR_OPMODE);//
		opModePrev &= 0xf8;
		opModePrev |= (uint8_t) opMode;
	SX1278WriteBuffer( REG_LR_OPMODE, opModePrev);
}

//========================================================================================================
uint8_t  SX1278LoRaGetOpMode(void)		 
{
	uint8_t opModePrev;
	opModePrev = SX1278ReadBuffer(REG_LR_OPMODE);
	return opModePrev;
}

//========================================================================================================
uint8_t  SX1278LoRaSetOpMode_test(RFMode_SET opMode) 
{
	uint8_t opModePrev;
	uint8_t opModePrev2;
	opModePrev = SX1278ReadBuffer(REG_LR_OPMODE);
	opModePrev &= 0xf8;
	opModePrev |= (uint8_t) opMode;
	SX1278WriteBuffer( REG_LR_OPMODE, opModePrev);
	opModePrev2 = SX1278ReadBuffer(REG_LR_OPMODE);
	if (opModePrev == opModePrev2)
		return 1;
	else
		return 0;
}

//========================================================================================================
void  SX1278LoRaSetRFFrequency(void) 	//频率
{

	SX1278WriteBuffer(REG_LR_FRFMSB, LoRa_Data.Freq[0]);
	SX1278WriteBuffer(REG_LR_FRFMID, LoRa_Data.Freq[1]);
	SX1278WriteBuffer(REG_LR_FRFLSB, LoRa_Data.Freq[2]);
}

//========================================================================================================
void  SX1278LoRaSetRFFrequency_434M(void) 
{
	SX1278WriteBuffer(REG_LR_FRFMSB, Frequency[0]);
	SX1278WriteBuffer(REG_LR_FRFMID, Frequency[1]);
	SX1278WriteBuffer(REG_LR_FRFLSB, Frequency[2]);
}

//========================================================================================================
void  SX1278LoRaSetRFFrequency_469_985M(void) 
{
	SX1278WriteBuffer(REG_LR_FRFMSB, Frequency2[0]);
	SX1278WriteBuffer(REG_LR_FRFMID, Frequency2[1]);
	SX1278WriteBuffer(REG_LR_FRFLSB, Frequency2[2]);
}

//========================================================================================================
void  SX1278LoRaSetRFFrequency_470M(void) 
{
	SX1278WriteBuffer(REG_LR_FRFMSB, Frequency3[0]);
	SX1278WriteBuffer(REG_LR_FRFMID, Frequency3[1]);
	SX1278WriteBuffer(REG_LR_FRFLSB, Frequency3[2]);
}

//========================================================================================================
void  SX1278LoRaSetRFFrequency_470_015M(void) 
{
	SX1278WriteBuffer(REG_LR_FRFMSB, Frequency4[0]);
	SX1278WriteBuffer(REG_LR_FRFMID, Frequency4[1]);
	SX1278WriteBuffer(REG_LR_FRFLSB, Frequency4[2]);
}

//========================================================================================================
void  SX1278LoRaSetNbTrigPeaks(uint8_t value) 
{
	uint8_t RECVER_DAT;
	RECVER_DAT = SX1278ReadBuffer(0x31);
	RECVER_DAT = (RECVER_DAT & 0xF8) | value;
	SX1278WriteBuffer(0x31, RECVER_DAT);
}

//========================================================================================================
void  SX1278LoRaSetSpreadingFactor(uint8_t factor) 
{
	uint8_t RECVER_DAT;
	SX1278LoRaSetNbTrigPeaks(3);
	RECVER_DAT = SX1278ReadBuffer( REG_LR_MODEMCONFIG2);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG2_SF_MASK) | (factor << 4);
	SX1278WriteBuffer(REG_LR_MODEMCONFIG2, RECVER_DAT);
}

//========================================================================================================
void  SX1278LoRaSetErrorCoding(uint8_t value) 
{
	uint8_t RECVER_DAT;
	RECVER_DAT = SX1278ReadBuffer( REG_LR_MODEMCONFIG1);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG1_CODINGRATE_MASK)|(value << 1);
	SX1278WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT);
}

//========================================================================================================
void  SX1278LoRaSetSignalBandwidth(uint8_t bw) //带宽
{
	uint8_t RECVER_DAT;
	RECVER_DAT = SX1278ReadBuffer( REG_LR_MODEMCONFIG1);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG1_BW_MASK) | (bw << 4);
	SX1278WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT);
}

//========================================================================================================
void  SX1278LoRaSetImplicitHeaderOn(BOOL enable) //模式
{
	uint8_t RECVER_DAT;
	RECVER_DAT = SX1278ReadBuffer( REG_LR_MODEMCONFIG1);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK)|(enable);
	SX1278WriteBuffer(REG_LR_MODEMCONFIG1, RECVER_DAT);
}

//========================================================================================================
void  SX1278LoRaSetPayloadLength(uint8_t value)	 
{
	SX1278WriteBuffer( REG_LR_PAYLOADLENGTH, value);
}

//========================================================================================================
void  SX1278LoRaSetSymbTimeout(uint16_t value) 
{
	uint8_t RECVER_DAT[2];
	RECVER_DAT[0] = SX1278ReadBuffer( REG_LR_MODEMCONFIG2);
	RECVER_DAT[1] = SX1278ReadBuffer( REG_LR_SYMBTIMEOUTLSB);
	RECVER_DAT[0] = (RECVER_DAT[0] & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK)
			| ((value >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK);
	RECVER_DAT[1] = value & 0xFF;
	SX1278WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT[0]);
	SX1278WriteBuffer( REG_LR_SYMBTIMEOUTLSB, RECVER_DAT[1]);
}

//========================================================================================================
void  SX1278LoRaSetMobileNode(BOOL enable) 
{
	uint8_t RECVER_DAT;
	RECVER_DAT = SX1278ReadBuffer( REG_LR_MODEMCONFIG3);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG3_MOBILE_NODE_MASK)| (enable << 3);
	SX1278WriteBuffer(REG_LR_MODEMCONFIG3, RECVER_DAT);
}

//========================================================================================================
void  RF_RECEIVE(void) 
{
	SX1278LoRaSetOpMode(Stdby_mode);
	SX1278WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);  
	SX1278WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
	SX1278WriteBuffer(REG_LR_DIOMAPPING1, 0X00);		
	SX1278WriteBuffer(REG_LR_DIOMAPPING2, 0X00);
	SX1278LoRaSetOpMode(Receiver_mode);
}

//========================================================================================================
void  SX1278LoRaSetPacketCrcOn(BOOL enable) //CRC
{
	uint8_t RECVER_DAT;
	RECVER_DAT = SX1278ReadBuffer( REG_LR_MODEMCONFIG2);
	RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK)	| (enable << 2);
	SX1278WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT);
}

//========================================================================================================
void  SX1278LoRaFsk(Debugging_fsk_ook opMode) 
{
	uint8_t opModePrev;
	opModePrev = SX1278ReadBuffer(REG_LR_OPMODE);
	opModePrev &= 0x7F;
	opModePrev |= (uint8_t) opMode;
	SX1278WriteBuffer( REG_LR_OPMODE, opModePrev);
}

//========================================================================================================
void  SX1278LoRaSetRFPower(uint8_t power) //功率
{
	SX1278WriteBuffer(REG_LR_PADAC, 0x87);  
	SX1278WriteBuffer(REG_LR_PACONFIG, power_data[power]);  
}

//========================================================================================================
void  SX1278LoRaSetRFPower_parameters(uint8_t PaDac, uint8_t PaConfig) 
{
	SX1278WriteBuffer(REG_LR_PADAC, PaDac);
	SX1278WriteBuffer(REG_LR_PACONFIG, PaConfig);
}

//========================================================================================================
void  SX1278LORA_INT(void) 
{
	SX1278LoRaSetOpMode(Sleep_mode);   		
	SX1278LoRaFsk(LORA_mode);	       			
	SX1278LoRaSetOpMode(Stdby_mode);   		
	
	SX1278WriteBuffer(REG_LR_DIOMAPPING1, GPIO_VARE_1); 
	SX1278WriteBuffer(REG_LR_DIOMAPPING2, GPIO_VARE_2); 

	SX1278LoRaSetRFFrequency_470M();
	SX1278LoRaSetRFPower(powerValue);									//功率
	SX1278LoRaSetSpreadingFactor(SpreadingFactor);	
	SX1278LoRaSetErrorCoding(CodingRate);		  				
	
	SX1278LoRaSetSignalBandwidth(Bw_Frequency);	  		//带宽


	SX1278LoRaSetImplicitHeaderOn(false);		  		
	SX1278LoRaSetPayloadLength(0xff);		      		//0xff 
	SX1278LoRaSetPacketCrcOn(true);			      		//CRC 
	
	SX1278LoRaSetSymbTimeout(0x3FF);							
	SX1278LoRaSetMobileNode(true); 			      		
	RF_RECEIVE();
}

//========================================================================================================
void  SX1278LORA_INT_parameters(int speed, uint8_t PaDac, uint8_t PaConfig) 
{
	SX1278LoRaSetOpMode(Sleep_mode);   			
	SX1278LoRaFsk(LORA_mode);	       			
	SX1278LoRaSetOpMode(Stdby_mode);   		
	
	SX1278WriteBuffer( REG_LR_DIOMAPPING1, GPIO_VARE_1);
	SX1278WriteBuffer( REG_LR_DIOMAPPING1, GPIO_VARE_1);
	SX1278WriteBuffer( REG_LR_DIOMAPPING2, GPIO_VARE_2);
	
	if(speed == 0)
		SX1278LoRaSetRFFrequency_434M();
	if(speed == 1)
		SX1278LoRaSetRFFrequency_469_985M();
	if(speed == 2)
		SX1278LoRaSetRFFrequency_470M();
	if(speed == 3)
		SX1278LoRaSetRFFrequency_470_015M();
		
	//======= 设定发射功率 ======
	SX1278LoRaSetRFPower_parameters(PaDac, PaConfig);
	SX1278LoRaSetSpreadingFactor(SpreadingFactor);		
	SX1278LoRaSetErrorCoding(CodingRate);		  				
	SX1278LoRaSetPacketCrcOn(true);			      				
	SX1278LoRaSetSignalBandwidth(Bw_Frequency);	  		
	SX1278LoRaSetImplicitHeaderOn(false);		  				
	SX1278LoRaSetPayloadLength(0xff);		      				
	SX1278LoRaSetSymbTimeout(0x3FF);
	SX1278LoRaSetMobileNode(true); 			      				
	RF_RECEIVE();
}

//========================================================================================================
void  FUN_RF_SENDPACKET(uint8_t *RF_TRAN_P,uint8_t LEN) 
{
	uint8_t ASM_i;
	
	//=======待机 ======= 
	SX1278LoRaSetOpMode(Stdby_mode);   				
	
	SX1278WriteBuffer(REG_LR_HOPPERIOD,0);	              	
	SX1278WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_TXD_Value);	//打开中断
	SX1278WriteBuffer(REG_LR_PAYLOADLENGTH,LEN);	      		
	SX1278WriteBuffer(REG_LR_FIFOTXBASEADDR,0);
	SX1278WriteBuffer(REG_LR_FIFOADDRPTR,0);
	
	SX1278_CSL;
	SX1278_1_ReadWrite(0x80);
	for (ASM_i = 0; ASM_i < LEN; ASM_i++) 
	{
			SX1278_1_ReadWrite(*RF_TRAN_P);
			RF_TRAN_P++;
	}
	SX1278_CSH;
	
	SX1278WriteBuffer(REG_LR_DIOMAPPING1, 0x40); 
	SX1278WriteBuffer(REG_LR_DIOMAPPING2, 0x00); 
	
	SX1278LoRaSetOpMode(Transmitter_mode);
}


