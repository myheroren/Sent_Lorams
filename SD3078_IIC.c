/**
  ******************************************************************************
  * @file    rtc.c
  * @author  FAE
  * @version V1.0
  * @date    2018-04-10
  * @brief   i2c RTCӦ�ú���
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */ 


#include "stm32f0xx_hal.h"
#include "SD3078_IIC.h"

uint8_t Shadow_Second;
uint8_t Shadow_Minute;
uint8_t Shadow_Hour;

/* Private define ------------------------------------------------------------*/
#define WAITEDELAY  100

/* Private functions ---------------------------------------------------------*/


uint8_t RTC_ReadDate(Time_Def	*psRTC);

/*********************************************
 * ��������I2Cdelay
 * ��  ����I2C��ʱ����
 * ��  �룺��
 * ��  ������
 ********************************************/
static void I2Cdelay(void)
{	
   uint8_t i=WAITEDELAY; //��������Ż��ٶȣ�ͨ����ʱ3~10us��������ʾ����������������
   while(i) 
   { 
     i--; 
   }
}


/*********************************************
 * ��������I2CStart
 * ��  ��������I2C����
 * ��  �룺��
 * ��  ����TRUE:�����ɹ���FALSE:����ʧ��
 ********************************************/
static uint8_t I2CStart(void)
{
    SDA_H;
		I2Cdelay();
    SCL_H;
    I2Cdelay();
    if(!SDA_read)return FALSE;	//SDA��Ϊ�͵�ƽ������æ,�˳�
    SDA_L;
    I2Cdelay();
    SCL_L;
    I2Cdelay();
    return TRUE;
}

/*********************************************
 * ��������I2CStop
 * ��  �����ͷ�I2C����
 * ��  �룺��
 * ��  ������
 ********************************************/
static void I2CStop(void)
{
    SCL_L;
    I2Cdelay();
    SDA_L;
    I2Cdelay();
    SCL_H;
    I2Cdelay();
    SDA_H;
    I2Cdelay();
}

/*********************************************
 * ��������I2CAck
 * ��  ��������ASK
 * ��  �룺��
 * ��  ������
 ********************************************/
static void I2CAck(void)
{
    SCL_L;
    I2Cdelay();
    SDA_L;
    I2Cdelay();
    SCL_H;
    I2Cdelay();
    SCL_L;
    I2Cdelay();
}

/*********************************************
 * ��������I2CNoAck
 * ��  ��������NOASK
 * ��  �룺��
 * ��  ������
 ********************************************/
static void I2CNoAck(void)
{
    SCL_L;
    I2Cdelay();
    SDA_H;
    I2Cdelay();
    SCL_H;
    I2Cdelay();
    SCL_L;
    I2Cdelay();
}

/*********************************************
 * ��������I2CWaitAck
 * ��  ������ȡACK�ź�
 * ��  �룺��
 * ��  ����TRUE=��ACK,FALSE=��ACK
 ********************************************/
static uint8_t I2CWaitAck(void)
{
    SCL_L;
    I2Cdelay();
    SDA_H;			
    I2Cdelay();
    SCL_H;
    I2Cdelay();
    if(SDA_read)
    {
			SCL_L;
			return FALSE;
    }
    SCL_L;
    return TRUE;
}

/*********************************************
 * ��������I2CSendByte
 * ��  ����MCU����һ���ֽ�
 * ��  �룺��
 * ��  ������
 ********************************************/
static void I2CSendByte(uint8_t SendByte) //���ݴӸ�λ����λ
{
    uint8_t i=8;
		while(i--)
		{
			SCL_L;
			I2Cdelay();
			if(SendByte&0x80)
			SDA_H;  
			else 
			SDA_L;   
			SendByte<<=1;
			I2Cdelay();
			SCL_H;
			I2Cdelay();
		}
    SCL_L;
}

/*********************************************
 * ��������I2CReceiveByte
 * ��  ����MCU����һ���ֽ�
 * ��  �룺��
 * ��  ����ReceiveByte
 ********************************************/
static uint8_t I2CReceiveByte(void)
{
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;         
      SCL_L;
      I2Cdelay();
      SCL_H;
      I2Cdelay();	
      if(SDA_read)
      {
        ReceiveByte|=0x1;
      }
    }
    SCL_L;
    return ReceiveByte;   
}

/*********************************************
 * ��������WriteRTC_Enable
 * ��  ����RTCд�������
 * ��  �룺��
 * ��  ����TRUE:�����ɹ���FALSE:����ʧ��
 ********************************************/
uint8_t WriteRTC_Enable(void)
{
    if(FALSE == I2CStart()) return FALSE;
    I2CSendByte(RTC_Address); 
    if(I2CWaitAck()== FALSE){I2CStop();return FALSE;}
    I2CSendByte(CTR2);      
    I2CWaitAck();	
    I2CSendByte(0x80);//��WRTC1=1      
    I2CWaitAck();
    I2CStop(); 
										
    I2CStart();
    I2CSendByte(RTC_Address);      
    I2CWaitAck();   
    I2CSendByte(CTR1);
    I2CWaitAck();	
    I2CSendByte(0x84);//��WRTC2,WRTC3=1      
    I2CWaitAck();
    I2CStop(); 
    return TRUE;
}

/*********************************************
 * ��������WriteRTC_Disable
 * ��  ����RTCд��ֹ����
 * ��  �룺��
 * ��  ����TRUE:�����ɹ���FALSE:����ʧ��
 ********************************************/
uint8_t WriteRTC_Disable(void)
{
    if(FALSE == I2CStart()) return FALSE;
    I2CSendByte(RTC_Address); 
    if(!I2CWaitAck()){I2CStop(); return FALSE;}
    I2CSendByte(CTR1);//����д��ַ0FH      
    I2CWaitAck();	
    I2CSendByte(0x0) ;//��WRTC2,WRTC3=0      
    I2CWaitAck();
    I2CSendByte(0x0) ;//��WRTC1=0(10H��ַ)      
    I2CWaitAck();
    I2CStop(); 
    return TRUE; 
}

/*********************************************
 * ��������RTC_WriteDate
 * ��  ����дRTCʵʱ���ݼĴ���
 * ��  �룺ʱ��ṹ��ָ��
 * ��  ����TRUE:�����ɹ���FALSE:����ʧ��
 ********************************************/
uint8_t RTC_WriteDate(Time_Def	*psRTC)	//дʱ�����Ҫ��һ�ζ�ʵʱʱ��Ĵ���(00H~06H)����д�룬
{                               //�����Ե�����7��ʱ�������е�ĳһλ����д����,������ܻ�����ʱ�����ݵĴ����λ. 
                                //Ҫ�޸�����ĳһ������ , Ӧһ����д��ȫ�� 7 ��ʵʱʱ������.

		WriteRTC_Enable();				//ʹ�ܣ�����

		I2CStart();
		I2CSendByte(RTC_Address); 
		if(!I2CWaitAck()){I2CStop(); return FALSE;}
		I2CSendByte(0);			//����д��ʼ��ַ      
		I2CWaitAck();	
		I2CSendByte(psRTC->second);		//second     
		I2CWaitAck();	
		I2CSendByte(psRTC->minute);		//minute      
		I2CWaitAck();	
		I2CSendByte(psRTC->hour|0x80);//hour ,ͬʱ����Сʱ�Ĵ������λ��0��Ϊ12Сʱ�ƣ�1��Ϊ24Сʱ�ƣ�
		I2CWaitAck();	
		I2CSendByte(psRTC->week);		//week      
		I2CWaitAck();	
		I2CSendByte(psRTC->day);		//day      
		I2CWaitAck();	
		I2CSendByte(psRTC->month);		//month      
		I2CWaitAck();	
		I2CSendByte(psRTC->year);		//year      
		I2CWaitAck();	
		I2CStop();

		WriteRTC_Disable();				//����
		return	TRUE;
}

/*********************************************
 * ��������RTC_ReadDate
 * ��  ������RTCʵʱ���ݼĴ���
 * ��  �룺ʱ��ṹ��ָ��
 * ��  ����TRUE:�����ɹ���FALSE:����ʧ��
 ********************************************/
uint8_t RTC_ReadDate(Time_Def	*psRTC)
{
		I2CStart();
		I2CSendByte(RTC_Address);      
		if(!I2CWaitAck()){I2CStop(); return FALSE;}
		I2CSendByte(0);
		I2CWaitAck();
		I2CStart();	
		I2CSendByte(RTC_Address+1);
		I2CWaitAck();
		psRTC->second=I2CReceiveByte();
		I2CAck();
		psRTC->minute=I2CReceiveByte();
		I2CAck();
		psRTC->hour=I2CReceiveByte() & 0x7F;
		I2CAck();
		psRTC->week=I2CReceiveByte();
		I2CAck();
		psRTC->day=I2CReceiveByte();
		I2CAck();
		psRTC->month=I2CReceiveByte();
		I2CAck();
		psRTC->year=I2CReceiveByte();		
		I2CNoAck();
		I2CStop(); 
		return	TRUE;
}
/*********************************************
 * ������     ��I2CWriteSerial
 * ��  ��     ��I2C��ָ����ַдһ�ֽ�����
 * Device_Addr��I2C�豸��ַ
 * Address    ���ڲ���ַ
 * length     ���ֽڳ���
 * ps         ��������ָ��
 * ���       ��TRUE �ɹ���FALSE ʧ��
 ********************************************/	
uint8_t I2CWriteSerial(uint8_t DeviceAddress, uint8_t Address, uint8_t length, uint8_t *ps)
{
		if(DeviceAddress == RTC_Address)  WriteRTC_Enable();

		I2CStart();
		I2CSendByte(DeviceAddress);   
		if(!I2CWaitAck()){I2CStop(); return FALSE;}
		I2CSendByte(Address);			
		I2CWaitAck();
		for(;(length--)>0;)
		{ 	
			I2CSendByte(*(ps++));		
			I2CAck();			
		}
		I2CStop(); 

		if(DeviceAddress == RTC_Address)  WriteRTC_Disable();
		return	TRUE;
}

/*********************************************
 * ������     ��I2CReadSerial
 * ��  ��     ��I2C��ָ����ַдһ�ֽ�����
 * Device_Addr��I2C�豸��ַ
 * Address    ���ڲ���ַ
 * length     ���ֽڳ���
 * ps         ��������ָ��
 * ���       ��TRUE �ɹ���FALSE ʧ��
 ********************************************/	
uint8_t I2CReadSerial(uint8_t DeviceAddress, uint8_t Address, uint8_t length, uint8_t *ps)
{
		I2CStart();
		I2CSendByte(DeviceAddress);      
		if(!I2CWaitAck()){I2CStop(); return FALSE;}
		I2CSendByte(Address);
		I2CWaitAck();
		I2CStart();	
		I2CSendByte(DeviceAddress+1);
		I2CWaitAck();
		for(;--length>0;ps++)
		{
			*ps = I2CReceiveByte();
			I2CAck();
		}
		*ps = I2CReceiveByte();	
		I2CNoAck();
		I2CStop(); 
		return	TRUE;
}

/*********************************************
 * ��������Set_CountDown
 * ��  �������õ���ʱ�ж�
 * ��  �룺CountDown_Init ����ʱ�жϽṹ��ָ�� 
 * ��  ������
 ********************************************/
void Set_CountDown(CountDown_Def *CountDown_Init)					
{
		uint8_t buf[6];
		uint8_t tem=0xF0;
		buf[0] = (CountDown_Init->IM<<6)|0xB4;							//10H
		buf[1] = CountDown_Init->d_clk<<4;									//11H
		buf[2] = 0;																					//12H
		buf[3] = CountDown_Init->init_val & 0x0000FF;				//13H
		buf[4] = (CountDown_Init->init_val & 0x00FF00) >> 8;//14H
		buf[5] = (CountDown_Init->init_val & 0xFF0000) >> 16;//15H
		I2CWriteSerial(RTC_Address,CTR2,1,&tem);
		I2CWriteSerial(RTC_Address,CTR2,6,buf);
}

/*********************************************
 * ��������Set_Alarm
 * ��  �������ñ����жϣ����ӹ��ܣ�
 * Enable_config��ʹ������  
 * psRTC������ʱ���ʱ��ṹ��ָ��
 * ��  ������
 ********************************************/
void Set_Alarm(uint8_t Enable_config, Time_Def *psRTC)					
{
		uint8_t buf[10];
		buf[0] = psRTC->second;
		buf[1] = psRTC->minute;
		buf[2] = psRTC->hour;
		buf[3] = 0;
		buf[4] = psRTC->day;
		buf[5] = psRTC->month;
		buf[6] = psRTC->year;
		buf[7] = Enable_config;
		buf[8] = 0xFF;
		buf[9] = 0x92;	
		I2CWriteSerial(RTC_Address,Alarm_SC,10,buf);
}

/*********************************************
 * ��������SetFrq
 * ��  ��������RTCƵ���жϣ���INT�����Ƶ�ʷ���
 * ��  �룺Ƶ��ֵ
 * ��  ������
 ********************************************/
void SetFrq(enum Freq F_Out)					
{
		uint8_t buf[2];
		buf[0] = 0xA1;
		buf[1] = F_Out;
		I2CWriteSerial(RTC_Address,CTR2,2,buf);
}

/*********************************************
 * ��������ClrINT
 * ��  ������ֹ�ж�
 * int_EN���ж����� INTDE��INTDE��INTDE
 * ��  ������
 ********************************************/
void ClrINT(uint8_t int_EN)         
{
		uint8_t buf;
		buf = 0x80 & (~int_EN);
		I2CWriteSerial(RTC_Address,CTR2,1,&buf);
}
/*********************************************END OF FILE**********************/
