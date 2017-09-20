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
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "spi_flash.h"
#include "stm32_hal_legacy.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//
static __RAM_FUNC updateBaseFirmware(void);
static __RAM_FUNC myFMC_Erase(uint32_t u32addr);
static __RAM_FUNC FMC_ProgramPage(uint32_t u32startAddr, uint32_t * u32buff);
void CHECK_AND_SET_FLASH_PROTECTION(void);
uint8_t writeSPIFlashSector(uint16_t sectorNum, uint16_t lenByte, void* buff);

#define CONF_MAX_STRING_LENGTH_13   	13
#define DFUVERSION_DATASIZE				CONF_MAX_STRING_LENGTH_13
/* Private variables ---------------------------------------------------------*/
/* Status of Disk Functions */
typedef unsigned char	BYTE;
typedef BYTE	DSTATUS;


	//Firmware Information
	typedef struct {
		uint32_t BaseFirmwareTag;
		uint32_t BaseFirmwareChecksum;
		uint32_t BaseFirmwareSize;
		uint8_t  BaseFirmwareVersion[DFUVERSION_DATASIZE];
		uint8_t  BaseFirmwareRunFlag; // 5A- flag that it has been run, any other value represent it is a first run
		uint32_t NewFirmwareTag;         // for bootloader
		uint32_t NewFirmwareChecksum;
		uint32_t NewFirmwareSize;
		uint8_t  NewFirmwareVersion[DFUVERSION_DATASIZE];// for bootloader
		uint8_t  NewFirmwareRunFlag; // 5A- flag that it has been run, any other value represent it is a first run
		uint8_t  BootloaderDoneFlag; // 5A- flag that it has been run successfully, any other value represent opposite
		uint8_t  RunningFirmwareVersion[DFUVERSION_DATASIZE];
		// for bootloader NOTE: Do not change this location above structure element it should be at first location
		//$$$$$$$$$##################IF YOU CHANGE BOOTLOADER AND FIRMWARE DFU will never work !!!!!!!!!!!!
	}__attribute__ ((packed))FirmwareInfoTypes;




FirmwareInfoTypes FirmwareInfo;


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

#define USER_FLASH_START		0x08006000
#define IFLASH_PAGESIZE         FLASH_PAGE_SIZE //1024  //for internal flash


#define SPIFLASHPAGESIZE		256

#define FLASHTOTALNOOFSECTOR				1024 //sectors
#define FLASHSECTORSIZEBYTES				4096 //bytes
#define FLASHPAGESIZEBYTES					256
#define FLASHNOOFPAGEINSECTOR				FLASHSECTORSIZEBYTES//

#define FLASHWRITE_RETRY			5

#define SUCCESS			0
#define FOUND			0
#define EMPTY			1
#define NOTFOUND		2

#define SPIFLASH_ERROR			255
//#define ERROR					255

#define CONFIGDATA_SECTORSTART				0 //NEVER CHANGE THIS, USED BY BOOTLOADER
#define CONFIGDATA_SECTORSIZE				4 //NEVER CHANGE THIS, USED BY BOOTLOADER
#define BASEFIRMWARE_SECTORSTART			(CONFIGDATA_SECTORSTART+CONFIGDATA_SECTORSIZE)
#define BASEFIRMWARE_SECTORSIZE				58
#define NEWFIRMWARE_SECTORSTART				(BASEFIRMWARE_SECTORSTART+BASEFIRMWARE_SECTORSIZE)
#define NEWFIRMWARE_SECTORSIZE				58

#define FIRMWARE_MAX_SIZE					0x3A000 //i.e 232K bytes (58 Sector of 4096 byte sector size)


#define SPIFLASH_SIZE_MAX	(4*1024*1024)//4Mbytes


#define FLASH_TOTAL_PAGES 128  // 258K/2K = 128 pages
#define NOOFBOOTLOADERPAGE 12   //24K/2K= 12 pages
#define ENABLE_BOOTLOADER_PROTECTION 0
#define FIRST_PAGE_TO_PROTECT 0
#define LAST_PAGE_TO_PROTECT (NOOFBOOTLOADERPAGE-1) //32K i.e 16 pages


#define FIRMWARE_STATUS_OFFSET			(1)
#define FIRMWARE_FILE_SIZE_OFFSET		(2)
#define FIRMWARE_CRC_OFFSET				(3)
#define FIRMWARE_START_ADDR_OFFSET		(4)
#define FIRMWARE_ERR_ADDR_OFFSET		(5)

uint8_t IFlashPageBuffer[4096];

uint32_t memAddr = 0;
uint32_t flashCheckSum;
uint8_t retn=0;


FLASH_OBProgramInitTypeDef FlashOBstatus;
uint32_t FlashOBreg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

uint8_t NVwriteConfigDB(FirmwareInfoTypes *Config,uint16_t size);
uint8_t NVreadConfigDB(FirmwareInfoTypes *Config,uint16_t size);


/* Private function prototypes -----------------------------------------------*/
uint8_t SPIFlashRead(uint32_t address, uint32_t byteLen, void *databuff)
{
	uint8_t *buffer;
	buffer=(uint8_t*)databuff;
	PRINTF("\n\rBOOTLOADER : SPIFlash Read @Address =%u , len=%u",address,byteLen);
	if(0==spiflashmem->read(spiflashmem, address,byteLen,buffer))
		return SUCCESS;
	else
		return SPIFLASH_ERROR;
}
uint8_t SPIFlashErase(uint32_t address, uint32_t byteLen)
{

	if(0==spiflashmem->erase(spiflashmem, address,byteLen))
		return SUCCESS;
	else
		return SPIFLASH_ERROR;
}
uint8_t SPIFlashWrite(uint32_t address, uint32_t byteLen, void *databuff)
{
	uint8_t *buffer;
	buffer=(uint8_t*)databuff;
	//PRINTF("\n\rBOOTLOADER : SPIFlash Write @Address =%u , len=%u",address,byteLen);
	if(0==spiflashmem->write(spiflashmem, address,byteLen,buffer))
		return SUCCESS;
	else
		return SPIFLASH_ERROR;
}

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
	uint32_t appAddress;
	appAddress=USER_FLASH_START;
	PRINTF("\n\rBOOTLOADER : Executing User Program....@ 0x%08X\n\n\n\n\n\n\r",appAddress);

	  /* Jump to user application */
	  JumpAddress = *(__IO uint32_t*) (appAddress + 4);
	  Jump_To_Application = (pFunction) JumpAddress;
	  /* Initialize user application's Stack Pointer */
	    // Relocate the vector table
	    SCB->VTOR = (uint32_t) appAddress;
	  __set_MSP(*(__IO uint32_t*) appAddress);
		HAL_DeInit();
	  Jump_To_Application();



}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	DRESULT res = RES_ERROR;
	int ret=0;

	int32_t i = 3000000,j = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  HAL_Delay(100);
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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

#if ENABLE_BOOTLOADER_PROTECTION
/* Ensures that the first sector of flash is write-protected preventing that the
bootloader is overwritten */
CHECK_AND_SET_FLASH_PROTECTION();
#endif

//HAL_FLASHEx_OBGetConfig(&FlashOBstatus);
//*((uint8_t*)0x1FFF7800)=0xFFEFF8AA; //Factory Default
//FlashOBreg=*((uint8_t*)0x1FFF7800);
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


	PRINTF("\n\rBOOTLOADER : Starting ......");
//	u32Pt = (uint32_t *)IFlashPageBuffer;
//	SST25_Init();
//	SST25_Read(FIRMWARE_INFO_ADDR + FIRMWARE_STATUS_OFFSET*4,IFlashPageBuffer,4);
	if(HAL_FLASH_Unlock()!=HAL_OK)
	{
		PRINTF("\n\rBOOTLOADER : Error to unlock Internal Flash");
		execute_user_code();
	}

	//Read the configuration data section where firmware member contains firmware Info
	ret=NVreadConfigDB(&FirmwareInfo,sizeof(FirmwareInfoTypes));
	if(ret == 0)
			  res = RES_OK;
	PRINTF("\n\rBOOTLOADER : New Firmware Tag= 0x%08X , BaseFirmwareTag=0x%08x",FirmwareInfo.NewFirmwareTag, FirmwareInfo.BaseFirmwareTag);





	//Check New Firmware is available or not , if not available
	if(FirmwareInfo.NewFirmwareTag != 0x5A5A5A5A)
	{
		HAL_FLASH_Lock();
		//proceed to check Base firmware check
	}
	else
	{

		PRINTF("\n\rBOOTLOADER : Flashing NEW FIRMWARE FROM SPI FLASH");
		PRINTF("\n\rBOOTLOADER : New Firmware Size-%u checksum-%u",FirmwareInfo.NewFirmwareSize,FirmwareInfo.NewFirmwareChecksum );

		flashCheckSum = 0;
		i=0;//FIRMWARE_MAX_SIZE
		//for(i = 0; i < FirmwareInfo.NewFirmwareSize;i += IFLASH_PAGESIZE)
		for(i = 0; i < FIRMWARE_MAX_SIZE;i += IFLASH_PAGESIZE)
		{
			//SST25_Read(i + FIRMWARE_BASE_ADDR,IFlashPageBuffer, IFLASH_PAGESIZE);
			//spiflashmem->read(spiflashmem, (64*FLASHSECTORSIZEBYTES),IFLASH_PAGESIZE,IFlashPageBuffer);
			PRINTF("\n\rBOOTLOADER : Program Page : %d of %d from SPIFLash adrss=%u",( i/IFLASH_PAGESIZE),(FirmwareInfo.NewFirmwareSize/IFLASH_PAGESIZE),(i + (NEWFIRMWARE_SECTORSTART*FLASHSECTORSIZEBYTES)) );

			ret=SPIFlashRead( (i + (NEWFIRMWARE_SECTORSTART*FLASHSECTORSIZEBYTES)), IFLASH_PAGESIZE, IFlashPageBuffer);
			for(j = 0 ; j < IFLASH_PAGESIZE;j++)
			{
				flashCheckSum += IFlashPageBuffer[j];
			}
			myFMC_Erase((uint32_t)(USER_FLASH_START + i));
			FMC_ProgramPage((USER_FLASH_START + i),(uint8_t*)IFlashPageBuffer);
			memAddr = (uint32_t)(USER_FLASH_START + i);//pointer/address of Internal just written
			if(memcmp((uint8_t*)IFlashPageBuffer, (uint8_t*)memAddr , IFLASH_PAGESIZE) != NULL)
			{
				PRINTF("\n\rBOOTLOADER :  $$%%% ERROR write @ %08x, index=0x%08x",memAddr,i);
				break;
			}
			HAL_Delay(10);
		}

		PRINTF("\n\rBOOTLOADER :  Calc Checksum=%d , StoreChecksum=%d",flashCheckSum ,FirmwareInfo.NewFirmwareChecksum);
		if(flashCheckSum == FirmwareInfo.NewFirmwareChecksum)
		{
			PRINTF("\n\rBOOTLOADER : Flashing NEW FIRMWARE FROM SPI FLASH DONE SUCCCESSFULLY");
			//clear the tag of New Firmware
			FirmwareInfo.NewFirmwareTag=0;
			for(int l=0;l<CONF_MAX_STRING_LENGTH_13;l++)
				FirmwareInfo.RunningFirmwareVersion[l]=FirmwareInfo.NewFirmwareVersion[l];
			FirmwareInfo.BootloaderDoneFlag=0x5A;// to flag user application that New Firmware programmed successfully
		}
		else // Fallback machenism to Base Firmware kick in here
		{
			PRINTF("\n\n\n\rBOOTLOADER :  ERROR MISMATCH CHECKSUM OR WRITE ERROR");//try flash Base firmware
			PRINTF("\n\n\rBOOTLOADER :  FALL BACK TO BASEFIRMWARE");//try flash Base firmware
			//FALLBACK TO BASE FRIMWARE
			if(FirmwareInfo.BaseFirmwareTag == 0x5A5A5A5A)//check presence of BaseFirmware
			{
				flashCheckSum = 0;

				i=0;//FIRMWARE_MAX_SIZE
				//for(i = 0; i < FirmwareInfo.NewFirmwareSize;i += IFLASH_PAGESIZE)
				for(i = 0; i < FIRMWARE_MAX_SIZE;i += IFLASH_PAGESIZE)
				{
					//SST25_Read(i + FIRMWARE_BASE_ADDR,IFlashPageBuffer, IFLASH_PAGESIZE);
					//spiflashmem->read(spiflashmem, (64*FLASHSECTORSIZEBYTES),IFLASH_PAGESIZE,IFlashPageBuffer);
					PRINTF("\n\rBOOTLOADER : Program Page : %d of %d",( i/IFLASH_PAGESIZE),(FirmwareInfo.BaseFirmwareSize/IFLASH_PAGESIZE) );
					ret=SPIFlashRead( (i + (BASEFIRMWARE_SECTORSTART*FLASHSECTORSIZEBYTES)), IFLASH_PAGESIZE, IFlashPageBuffer);

					for(j = 0 ; j < IFLASH_PAGESIZE;j++)//calculate checksum
					{
						if(i + j < FirmwareInfo.BaseFirmwareSize)
							flashCheckSum += IFlashPageBuffer[j];
						else
							break;
					}
					myFMC_Erase((uint32_t)(USER_FLASH_START + i));
					FMC_ProgramPage((USER_FLASH_START + i),(uint8_t*)IFlashPageBuffer);
					memAddr = (uint32_t)(USER_FLASH_START + i);//pointer/address of Internal just written
					if(memcmp((uint8_t*)IFlashPageBuffer, (uint8_t*)memAddr , IFLASH_PAGESIZE) != NULL)
					{
						PRINTF("\n\rBOOTLOADER :  $$%%% ERROR write @ %08x",memAddr);
						break;
					}
					HAL_Delay(50);
				}

				if(flashCheckSum == FirmwareInfo.BaseFirmwareChecksum)
					PRINTF("\n\rBOOTLOADER :  MATCHED CHECKSUM OF BASE FIRMWARE",memAddr);
				else
					PRINTF("\n\rBOOTLOADER :  ***** ???? MISMATCHED CHECKSUM OF BASE FIRMWARE",memAddr);
				for(int l=0;l<CONF_MAX_STRING_LENGTH_13;l++)
					FirmwareInfo.RunningFirmwareVersion[l]=FirmwareInfo.BaseFirmwareVersion[l];

			}
			else
				PRINTF("\n\n\rBOOTLOADER :  *** ???? NO BASEFIRMWARE");

			FirmwareInfo.BootloaderDoneFlag=0;// to flag user application that New Firmware NOT programmed
		}
	}


	//Check Base Tag : If no tag, programmed External Base Flash area with That running/new firmware
	if(FirmwareInfo.BaseFirmwareTag != 0x5A5A5A5A)// No tag on base Program the Base from Internal running firmware
	{
		PRINTF("\n\rBOOTLOADER : Copy Current Firmware to Base Firmware Copy on SPIFlash");


		//Copy running firmware to Base Firmware
		FirmwareInfo.BaseFirmwareTag=0x5A5A5A5A;//mark it has been done
		FirmwareInfo.BaseFirmwareChecksum=0;
		FirmwareInfo.BaseFirmwareSize=FIRMWARE_MAX_SIZE;
		FirmwareInfo.BaseFirmwareRunFlag=0x5A; //mark it has run
		for(int i=0;i<DFUVERSION_DATASIZE;i++)
			FirmwareInfo.BaseFirmwareVersion[i]=FirmwareInfo.RunningFirmwareVersion[i];

		//Copy firmware to external flash
		updateBaseFirmware();//checksum calculated inside this function

		//NVwriteConfigDB(&FirmwareInfo,sizeof(FirmwareInfoTypes));
		PRINTF("\n\n\n\rBOOTLOADER : Copy to BaseFirmware area in External SPI Flash Done ***");

	}


	// Store back updated FirmwareInformation to SPI FLash
	retn=NVwriteConfigDB(&FirmwareInfo,sizeof(FirmwareInfoTypes));
	PRINTF("\n\n\n\rBOOTLOADER : Config Write return = %d",retn);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
void vcom_Send( char *format, ... )
{
  //va_list args;
  __gnuc_va_list args;
  va_start(args, format);

  /*convert into string at buff[0] of length iw*/
  iw = vsprintf(&buff[0], format, args);

  HAL_UART_Transmit(&UART_VCOM_PORT,(uint8_t *)&buff[0], iw, 300);

  va_end(args);
}
__RAM_FUNC updateBaseFirmware(void)
{
	//Copy base firmware if need
	uint8_t * programPtr;
	uint8_t* rambuff;
	uint32_t bytecnt=0,extFlashSectorBytecnt=0;

	uint16_t SectorWriteCnt=0,retrn;
	rambuff=(uint8_t*)malloc(FLASHSECTORSIZEBYTES);
	programPtr=(uint8_t*)USER_FLASH_START;//point to start of application on internal flash
	for(bytecnt=0; bytecnt<FIRMWARE_MAX_SIZE;bytecnt++)
	{

		if(bytecnt!=0 && (bytecnt%FLASHSECTORSIZEBYTES==0))//sector alignment check
		{
			PRINTF("\n\rBOOTLOADER : SPI SECTOR ERASE WRITE @ %d",BASEFIRMWARE_SECTORSTART+SectorWriteCnt);
			//write the buffer to external flash Base Firmware area

			retrn=writeSPIFlashSector((BASEFIRMWARE_SECTORSTART+SectorWriteCnt), FLASHSECTORSIZEBYTES, rambuff);
			if(retrn!=SUCCESS)
			{
				PRINTF("\n\rBOOTLOADER : BASE FIRMWARE COPY ERROR ******");

				//reboot or trap the system here Note: TO DO
			}
			SectorWriteCnt++;
			extFlashSectorBytecnt+=FLASHSECTORSIZEBYTES;

		}
		//PRINTF(",%d:0x%02X",bytecnt,*(programPtr+bytecnt));
		rambuff[bytecnt-extFlashSectorBytecnt]=*(programPtr+bytecnt);
		FirmwareInfo.BaseFirmwareChecksum=FirmwareInfo.BaseFirmwareChecksum + *(programPtr+bytecnt);


	}
	free(rambuff);
}
__RAM_FUNC myFMC_Erase(uint32_t u32addr)
{
	uint16_t  FlashStatus;
	FLASH_EraseInitTypeDef eraseinstance;
	uint32_t PageErrorStatus;
	uint32_t InternalFlashPageNo=0;
//	FlashStatus = FLASH_ErasePage(u32addr);
	InternalFlashPageNo=u32addr-0x08000000;//since we passed abosulte address of Flash
	InternalFlashPageNo=InternalFlashPageNo/IFLASH_PAGESIZE;


	//FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGAERR|FLASH_FLAG_PROGERR);
	PRINTF("\n\rBOOTLOADER : Erasing Page %d of Bank1 @Address: 0x%08x",InternalFlashPageNo,u32addr );

	eraseinstance.Banks=1;
	eraseinstance.NbPages=1;
	eraseinstance.Page=InternalFlashPageNo;
	eraseinstance.TypeErase=FLASH_TYPEERASE_PAGES;
	HAL_FLASH_Unlock(); //Unlocks the flash memory
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PROGERR | FLASH_FLAG_PGSERR | FLASH_FLAG_SIZERR);
	//FLASH_FlushCaches();
	FLASH_FlushCaches();
	PageErrorStatus=0;
	FlashStatus= HAL_FLASHEx_Erase(&eraseinstance, &PageErrorStatus);
	HAL_FLASH_Lock();
	FlashStatus=0;
	//FLASH_PageErase(InternalFlashPageNo, 1);//pass as argument, page no and Bank 1
	//FlashStatus=FLASH_PageErase(InternalFlashPageNo, 1);//pass as argument, page no and Bank 1
//	if(HAL_OK != FlashStatus)
//	{
//		NVIC_SystemReset();
//	}
}


__RAM_FUNC FMC_ProgramPage(uint32_t u32startAddr, uint32_t * u32buff)
{
	uint16_t  FlashStatus;
    uint32_t i;
    for (i = 0; i < IFLASH_PAGESIZE/8; i++)
    //for (i = 0; i < IFLASH_PAGESIZE/4; i++)
    {
			//FlashStatus = FLASH_ProgramWord(u32startAddr + i*4, u32buff[i]);
    	  /* Check the parameters */
    	  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));

    	  /* Set PG bit */
    	  SET_BIT(FLASH->CR, FLASH_CR_PG);

//    	  /* Program the word */
//    	  *(__IO uint32_t*)(u32startAddr + i*4) = (uint32_t)u32buff[i];
//			if(u32buff[i] != *(uint32_t*)(u32startAddr + i*4)) //check wrote data lower word
//			{
//				NVIC_SystemReset();
//			}

    	//HAL_FLASHEx_DATAEEPROM_Program(TYPEPROGRAM_FASTWORD,u32startAddr + i*4,(uint32_t *) u32buff[i]);
    	  HAL_FLASH_Unlock();
    	    FlashStatus= HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, u32startAddr + i*8, *(uint64_t*)(u32buff+(2*i)));
    	    HAL_FLASH_Lock();
    	    if(HAL_OK != FlashStatus)
			{
				NVIC_SystemReset();
			}
			if(u32buff[2*i] != *(uint32_t*)(u32startAddr + i*8)) //check wrote data lower word
			{
				NVIC_SystemReset();
			}
			if(u32buff[(2*i)+1] != *(uint32_t*)(u32startAddr + (i*8)+4)) //check wrote data higher word
			{
				NVIC_SystemReset();
			}
    }
}

void CHECK_AND_SET_FLASH_PROTECTION(void)
{
	FLASH_OBProgramInitTypeDef obConfig;
	HAL_FLASHEx_OBGetConfig(&obConfig);
	/* If the first sector is not protected */
	if (obConfig.WRPStartOffset == FIRST_PAGE_TO_PROTECT && obConfig.WRPEndOffset == LAST_PAGE_TO_PROTECT)
	{
		HAL_FLASH_Unlock(); //Unlocks flash
		HAL_FLASH_OB_Unlock(); //Unlocks OB
		obConfig.WRPArea = OB_WRPAREA_BANK1_AREAA;
		obConfig.OptionType = OPTIONBYTE_WRP;
		obConfig.WRPStartOffset = FIRST_PAGE_TO_PROTECT;
		obConfig.WRPEndOffset = LAST_PAGE_TO_PROTECT;
		HAL_FLASHEx_OBProgram(&obConfig); //Programs the OB
		HAL_FLASH_OB_Launch(); //Ensures that the new configuration is saved in flash
		HAL_FLASH_OB_Lock(); //Locks OB
		HAL_FLASH_Lock(); //Locks flash
	}
}

uint8_t NVwriteConfigDB(FirmwareInfoTypes *Config,uint16_t size)
{
	uint8_t* buffptr;
	buffptr=(uint8_t*) Config;

	//debug
	PRINTF("\n\rBOOTLOADER : Write Function called : BaseFirmwareTag [0x%08x], NewFirmwareTag [0x%08x]",Config->BaseFirmwareTag,Config->NewFirmwareTag );

	if(size <= FLASHSECTORSIZEBYTES)
	{
		retn=writeSPIFlashSector(CONFIGDATA_SECTORSTART, size,buffptr);
		if(retn==0)
			return SUCCESS;
		else
			return SPIFLASH_ERROR;
	}
	else if(size <= FLASHSECTORSIZEBYTES*2)//it support manager data size more than 1 sector or >4096 bytes
	{
		retn=writeSPIFlashSector(CONFIGDATA_SECTORSTART, FLASHSECTORSIZEBYTES, buffptr);
		//retn|=writeSPIFlashSector((DBManager.SectorStart[RECORDMANAGER]+1),sizeOfmanager-DBManager.FlashSectorSize,bytebybyte+FLASHSECTORSIZEBYTES);
		retn|=writeSPIFlashSector((CONFIGDATA_SECTORSTART+1), size-FLASHSECTORSIZEBYTES, buffptr+FLASHSECTORSIZEBYTES);
		if(retn==0)
			return SUCCESS;
		else
			return SPIFLASH_ERROR;
	}
	else
		return SPIFLASH_ERROR;
}
uint8_t NVreadConfigDB(FirmwareInfoTypes *Config,uint16_t size)
{
	uint8_t* buffptr;
	buffptr=(uint8_t*) Config;
	retn=SPIFlashRead(CONFIGDATA_SECTORSTART*FLASHSECTORSIZEBYTES, size,buffptr);

	//debug
		PRINTF("\n\rBOOTLOADER : Read Function called return:%d ",retn);
	if(retn==0)
		return SUCCESS;
	else
		return SPIFLASH_ERROR;
}


uint8_t writeSPIFlashSector(uint16_t sectorNum, uint16_t lenByte, void* buff)
{

	uint8_t retrycnt=FLASHWRITE_RETRY; //Handle multiple retry of Sector Write
	uint16_t retn=0;
	uint32_t addrs= sectorNum*FLASHSECTORSIZEBYTES;
	uint8_t byteacessBuff[lenByte];



	while(retrycnt!=0)//try multiple write if not successful
	{
		retn=0;
		retn=SPIFlashErase(addrs,FLASHSECTORSIZEBYTES);
		retn|=SPIFlashWrite(addrs,lenByte,(uint8_t *)buff);
		retn|=SPIFlashRead(addrs,lenByte,byteacessBuff);

		if(retn==0 && memcmp((uint8_t *)buff,byteacessBuff,lenByte)==0)
		{

			//PRINTF("\n\rBOOTLOADER : Success Write @%d, len=%d, no of retry=%d \n\r",sectorNum,lenByte,(FLASHWRITE_RETRY-retrycnt));

			return SUCCESS;
		}
		retrycnt--;//decrement for next try if not success
	}


	PRINTF("\n\rBOOTLOADER : Sector Write SPIFLASH_ERROR @%d sector\n\r",sectorNum);

	return SPIFLASH_ERROR;//write fail on a sector
}

//#################### Utility function SECTION


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
