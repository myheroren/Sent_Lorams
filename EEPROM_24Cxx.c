#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "General.h"
#include "EEPROM_24Cxx.h"

uint8_t vE2romTest(void);

//24C64 : 地址13位，256页，每页32字节，8K字节，65535位

#define E2PROM_SIZE 			0x2000    
#define E2PROM_BASE_ID    0xA0 
#define E2PROM_WRITE 			0x00
#define E2PROM_READ     	0x01 
#define E2PROM_BASE_WID E2PROM_BASE_ID + E2PROM_WRITE
#define E2PROM_BASE_RID E2PROM_BASE_ID + E2PROM_READ 

/************************************************************************************************/
uint8_t writeAT24Cxx(uint16_t Startaddr, uint8_t *data, uint16_t len)
{
	uint8_t *p = data;        

	/*is the address overfolw*/    
	if(Startaddr + len >= E2PROM_SIZE)        
		return 1;        
	
	/*calculate the current write position to know how many word can write continully*/   
	//HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	if( HAL_I2C_Mem_Write(&hi2c1, E2PROM_BASE_WID, Startaddr&0xFFFF,I2C_MEMADD_SIZE_16BIT, p, len, 1000) == HAL_OK)        
	{            
			HAL_Delay(5);            
			return HAL_OK;
	}                    
	return HAL_ERROR;    
} 

/************************************************************************************************/
uint8_t readAT24Cxx(uint16_t Startaddr, uint8_t *data, uint16_t len)
{
	uint8_t *p = data;     /*is the address overfolw*/    

	if(Startaddr + len >= E2PROM_SIZE)        
			return 1;        /*calculate the current write position to know how many word can write continully*/    
	// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	if( HAL_I2C_Mem_Read(&hi2c1, E2PROM_BASE_RID, Startaddr&0xFFFF,I2C_MEMADD_SIZE_16BIT, p, len, 20) != HAL_OK)        
	{            
				return HAL_OK;    
	} 
	return HAL_ERROR;    
}  
