/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "spi_flash.h"
#include "stm32_hal_legacy.h"

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Status of Disk Functions */
typedef unsigned char	BYTE;
typedef BYTE	DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;
/* Disk Status Bits (DSTATUS) */

#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */
#define STA_PROTECT		0x04	/* Write protected */

static volatile DSTATUS Stat = STA_NOINIT;
struct spi_flash *spiflashmem;

#define USER_FLASH_START		0x08004000
#define IFLASH_PAGESIZE         2048 //1024  //for internal flash

#define SECTOR_SIZE	4096

#define FLASH_SIZE_MAX	(4*1024*1024)//4Mbytes
#define FIRMWARE_MAX_SIZE		0x00040000

#define BASE_FIRMWARE_INFO_ADDR 			0
#define BASE_FIRMWARE_BASE_ADDR			(0x00001000 + BASE_FIRMWARE_INFO_ADDR)

#define FIRMWARE_INFO_ADDR 								(BASE_FIRMWARE_BASE_ADDR + FIRMWARE_MAX_SIZE)
#define FIRMWARE_BASE_ADDR								(0x00001000 + FIRMWARE_INFO_ADDR)

#define FIRMWARE_STATUS_OFFSET			(1)
#define FIRMWARE_FILE_SIZE_OFFSET		(2)
#define FIRMWARE_CRC_OFFSET				(3)
#define FIRMWARE_START_ADDR_OFFSET		(4)
#define FIRMWARE_ERR_ADDR_OFFSET		(5)

uint8_t buff[4096];
uint8_t flashBuff[4096];//[IFLASH_PAGESIZE];
uint32_t fileSize;
uint32_t fileCrc;
uint32_t packetNo;
uint32_t firmwareFileOffSet;
uint32_t firmwareFileSize;
uint8_t usbRecv = 0;
uint32_t memAddr = 0;
uint32_t flashCheckSum;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t CfgCalcCheckSum(uint8_t *buff, uint32_t length)
{
	uint32_t i;
	uint8_t crc = 0;
	for(i = 0;i < length; i++)
	{
		crc += buff[i];
	}
	return crc;
}

typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

void execute_user_code(void)
{
	//SysDeInit();
	HAL_DeInit();
	//HAL_GPIO_DeInit();
	HAL_SPI_DeInit(&hspi2);

	if (((*(vu32*)USER_FLASH_START) & 0x2FFF0000 ) == 0x20000000)
    { /* Jump to user application */
			//SCB->VTOR = USER_FLASH_START;
      JumpAddress = *(vu32*) (USER_FLASH_START + 4);
      Jump_To_Application = (pFunction) JumpAddress;
      /* Initialize user application's Stack Pointer */
      __set_MSP(*(vu32*) USER_FLASH_START);
      Jump_To_Application();
    }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	  DRESULT res = RES_ERROR;
	  int ret=0;
	uint32_t *u32Pt = (uint32_t *)buff;
	uint8_t *u8pt,u8temp;
	int32_t i = 3000000,j = 0;
	uint32_t u32temp,timeOut = 20000000;
	uint8_t tryNum,exit;
	//SysInit();
	while(i--);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  Stat = STA_NOINIT;

  spiflashmem->spi =spi_setup_slave();

  spiflashmem =spi_flash_probe();

  if(spiflashmem != NULL)
  {
    Stat &= ~STA_NOINIT;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	u32Pt = (uint32_t *)buff;
//	SST25_Init();
//	SST25_Read(FIRMWARE_INFO_ADDR + FIRMWARE_STATUS_OFFSET*4,buff,4);
	HAL_FLASH_Unlock();

	ret=spiflashmem->read(spiflashmem, FIRMWARE_INFO_ADDR + FIRMWARE_STATUS_OFFSET*4, 4, buff);
	if(ret == 0)
			  res = RES_OK;

	if(*u32Pt != 0x5A5A5A5A)
	{
		HAL_FLASH_Lock();
		execute_user_code();
	}
	tryNum = 10;
	exit = 0;
	while(!exit && tryNum--)
	{
		//SST25_Read(FIRMWARE_INFO_ADDR + FIRMWARE_CRC_OFFSET*4,buff,4);
		ret|=spiflashmem->read(spiflashmem, FIRMWARE_INFO_ADDR + FIRMWARE_CRC_OFFSET*4, 4, buff);
		fileCrc = 	*u32Pt;
		//SST25_Read(FIRMWARE_INFO_ADDR + FIRMWARE_FILE_SIZE_OFFSET*4,buff,4);
		ret|=spiflashmem->read(spiflashmem, FIRMWARE_INFO_ADDR + FIRMWARE_FILE_SIZE_OFFSET*4, 4, buff);
		fileSize = 	*u32Pt;
		if(fileSize < FIRMWARE_MAX_SIZE)
		{
			flashCheckSum = 0;
			for(i = 0; i < fileSize;i += IFLASH_PAGESIZE)
			{
				//SST25_Read(i + FIRMWARE_BASE_ADDR,buff, IFLASH_PAGESIZE);
				ret|=spiflashmem->read(spiflashmem, i + FIRMWARE_BASE_ADDR, IFLASH_PAGESIZE, buff);
				for(j = 0 ; j < IFLASH_PAGESIZE;j++)
				{
					if(i + j < fileSize)
						flashCheckSum += buff[j];
					else
						break;
				}
				myFMC_Erase(USER_FLASH_START + i);
				FMC_ProgramPage(USER_FLASH_START + i,u32Pt);
				//SST25_Read(i + FIRMWARE_BASE_ADDR,buff, IFLASH_PAGESIZE);
				ret|=spiflashmem->read(spiflashmem, i + FIRMWARE_BASE_ADDR, IFLASH_PAGESIZE, buff);
			    memAddr = USER_FLASH_START + i;
				if(memcmp(buff, (void*)memAddr , IFLASH_PAGESIZE) != NULL)
					break;
			}
			if(flashCheckSum == fileCrc)//check if New Firmware is successfullly written to internal Flash
			{
					//SST25_Read(BASE_FIRMWARE_INFO_ADDR + FIRMWARE_STATUS_OFFSET*4,buff,4);
					ret|=spiflashmem->read(spiflashmem,BASE_FIRMWARE_INFO_ADDR + FIRMWARE_STATUS_OFFSET*4, 4, buff);
					if(*u32Pt != 0x5A5A5A5A)
					{
						tryNum = 10;
						while(tryNum--)//Copy all New firmware to Base Firmware
						{
							flashCheckSum = 0;
							for(i = 0; i < fileSize;i += 4096)
							{
								//SST25_Read(i + FIRMWARE_BASE_ADDR,buff, 256);
								ret|=spiflashmem->read(spiflashmem,i + FIRMWARE_BASE_ADDR, 4096, buff);
								//SST25_Write(i + BASE_FIRMWARE_BASE_ADDR,buff, 256);
								ret|=spiflashmem->erase(spiflashmem,i + BASE_FIRMWARE_BASE_ADDR, 4096);
								ret|=spiflashmem->write(spiflashmem,i + BASE_FIRMWARE_BASE_ADDR, 4096, buff);
								//SST25_Read(i + BASE_FIRMWARE_BASE_ADDR,flashBuff, 256);
								ret|=spiflashmem->read(spiflashmem,i + BASE_FIRMWARE_BASE_ADDR, 4096, flashBuff);
								//if(memcmp(buff,flashBuff,256) != NULL)
								if(memcmp(buff,flashBuff,4096) != NULL)
								{
										break;
								}
							}
							if(i >= fileSize)//check the checksum after copy done to Base frimware location
								for(i = 0; i < fileSize;i += 256)
								{
										//SST25_Read(i + BASE_FIRMWARE_BASE_ADDR,buff, 256);
										ret|=spiflashmem->read(spiflashmem,i + BASE_FIRMWARE_BASE_ADDR, 256, buff);
										for(j = 0 ; j < 256;j++)
										{
											if(i + j < fileSize)
												flashCheckSum += buff[j];
											else
												break;
										}
								}
							if(flashCheckSum == fileCrc)
							{
								//SST25_Read(FIRMWARE_INFO_ADDR,buff, 256);
								ret|=spiflashmem->read(spiflashmem,FIRMWARE_INFO_ADDR, 4096, buff);
								//SST25_Write(BASE_FIRMWARE_INFO_ADDR,buff, 256);
								ret|=spiflashmem->erase(spiflashmem,BASE_FIRMWARE_INFO_ADDR, 4096);
								ret|=spiflashmem->write(spiflashmem,BASE_FIRMWARE_INFO_ADDR, 4096, buff);
								//SST25_Read(BASE_FIRMWARE_INFO_ADDR,flashBuff, 256);
								ret|=spiflashmem->read(spiflashmem,i + BASE_FIRMWARE_BASE_ADDR, 4096, buff);
								if(memcmp(buff,flashBuff,4096) == NULL)
								{
										break;
								}
							}
						}
					}
					exit = 1;
				}
			}
		}
		if(exit == 0 && tryNum == 0xff)
		{
			tryNum = 10;
			exit = 0;
			while(!exit && tryNum--)
			{
				//SST25_Read(BASE_FIRMWARE_INFO_ADDR + FIRMWARE_CRC_OFFSET*4,buff,4);
				ret|=spiflashmem->read(spiflashmem, BASE_FIRMWARE_INFO_ADDR + FIRMWARE_CRC_OFFSET*4, 4, buff);
				fileCrc = 	*u32Pt;
				//SST25_Read(BASE_FIRMWARE_INFO_ADDR + FIRMWARE_FILE_SIZE_OFFSET*4,buff,4);
				ret|=spiflashmem->read(spiflashmem, BASE_FIRMWARE_INFO_ADDR + FIRMWARE_FILE_SIZE_OFFSET*4, 4, buff);
				fileSize = 	*u32Pt;
				if(fileSize < FIRMWARE_MAX_SIZE)
				{
					flashCheckSum = 0;
					for(i = 0; i < fileSize;i += IFLASH_PAGESIZE)
					{
						//SST25_Read(i + BASE_FIRMWARE_BASE_ADDR,buff, IFLASH_PAGESIZE);
						ret|=spiflashmem->read(spiflashmem, i + BASE_FIRMWARE_BASE_ADDR, IFLASH_PAGESIZE, buff);
						for(j = 0 ; j < IFLASH_PAGESIZE;j++)
						{
								if(i + j < fileSize)
									flashCheckSum += buff[j];
								else
									break;
						}
						myFMC_Erase(USER_FLASH_START + i);
						FMC_ProgramPage(USER_FLASH_START + i,u32Pt);
						//SST25_Read(i + BASE_FIRMWARE_BASE_ADDR,buff, IFLASH_PAGESIZE);
						ret|=spiflashmem->read(spiflashmem, i + BASE_FIRMWARE_BASE_ADDR, IFLASH_PAGESIZE, buff);
						memAddr = USER_FLASH_START + i;
						if(memcmp(buff, (void*)memAddr , IFLASH_PAGESIZE) != NULL)
							break;
					}
					if(flashCheckSum == fileCrc)
					{
							exit = 1;
					}
				}
			}
		}

	//SST25_Erase(FIRMWARE_INFO_ADDR,block4k);
	ret|=spiflashmem->erase(spiflashmem,FIRMWARE_INFO_ADDR, 4096);// Clear INFO ON FIRMWARE
	HAL_FLASH_Lock();
	execute_user_code();


  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void myFMC_Erase(uint32_t u32addr)
{
	uint16_t  FlashStatus;
	uint32_t InternalFlashPageNo=0;
//	FlashStatus = FLASH_ErasePage(u32addr);
	InternalFlashPageNo=u32addr-0x08000000;//since we passed abosulte address of Flash
	InternalFlashPageNo=InternalFlashPageNo/IFLASH_PAGESIZE;
	FLASH_PageErase(InternalFlashPageNo, 1);//pass as argument, page no and Bank 1
//	if(FLASH_COMPLETE != FlashStatus)
//	{
//		NVIC_SystemReset();
//	}
}

void FMC_ProgramPage(uint32_t u32startAddr, uint32_t * u32buff)
{
	uint16_t  FlashStatus;
    uint32_t i;
    //for (i = 0; i < IFLASH_PAGESIZE/8; i++)
    for (i = 0; i < IFLASH_PAGESIZE/4; i++)
    {
			//FlashStatus = FLASH_ProgramWord(u32startAddr + i*4, u32buff[i]);
    	  /* Check the parameters */
    	  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));

    	  /* Set PG bit */
    	  SET_BIT(FLASH->CR, FLASH_CR_PG);

    	  /* Program the word */
    	  *(__IO uint32_t*)(u32startAddr + i*4) = (uint32_t)u32buff[i];
			if(u32buff[i] != *(uint32_t*)(u32startAddr + i*4)) //check wrote data lower word
			{
				NVIC_SystemReset();
			}

//    	//HAL_FLASHEx_DATAEEPROM_Program(TYPEPROGRAM_FASTWORD,u32startAddr + i*4,(uint32_t *) u32buff[i]);
//    	    FlashStatus= HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, u32startAddr + i*8, *(uint64_t*)(u32buff+(2*i)))
//			if(HAL_OK != FlashStatus)
//			{
//				NVIC_SystemReset();
//			}
//			if(u32buff[2*i] != *(uint32_t*)(u32startAddr + i*8)) //check wrote data lower word
//			{
//				NVIC_SystemReset();
//			}
//			if(u32buff[(2*i)+1] != *(uint32_t*)(u32startAddr + (i*8)+4)) //check wrote data higher word
//			{
//				NVIC_SystemReset();
//			}
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
