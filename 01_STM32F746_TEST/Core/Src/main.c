/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

//#include "stm32746g_discovery_sd.h"
#include "sdcard_dy.h"

#include <assert.h>
#include <errno.h>

#include "nnom.h"
#include "weights.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
unsigned int TimerDelay=0;
uint32_t TimerCount;
uint32_t TimerCountSec;

uint8_t CheckRetVal=0;


char TxBuf[100];
unsigned char TxBufLen;

unsigned char UartRxBuf[100];
unsigned char UartIndex6;
unsigned char rx6_byte;

unsigned char SD_isDetected;
unsigned char SD_isInit;
unsigned char SD_isWriteDone;


uint8_t workBuffer[_MAX_SS];

uint32_t byteswritten, bytesread;                     /* File write/read counts */
uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
uint8_t rtext[255];                                   /* File read buffer */

char TestSet[14] = "test_data.bin";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void SD_MspDeInit_dy(SD_HandleTypeDef *hsd, void *Params);
void SD_MspInit_dy(SD_HandleTypeDef *hsd, void *Params);
uint8_t SD_DeInit_dy(void);
uint8_t SD_Init_dy(void);

uint8_t SD_isDetected_dy(void);
uint8_t SD_Link_dy(void);
uint8_t SD_Mount_dy(void);
uint8_t SD_GetInfo_dy(void);
uint8_t SD_Format(void);

int8_t* load(const char* file, size_t * size);
int8_t* load_file(const char* filename, uint32_t* size);
nnom_status_t callback(nnom_model_t* m, nnom_layer_t* layer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#if 0
	if(htim->Instance == TIM6)
	{
		if(TimerDelay) TimerDelay--;
	}
#endif
	if(htim->Instance == TIM7)
	{
		if(TimerCount < 5000000)
		{
			TimerCount++;
		}
		else
		{
#if 0
			if(TimerCountSec < 60)
			{
				TimerCountSec++;
			}
			else
			{
				TimerCountSec=0;
			}
#endif
			TimerCount=0;
		}
	}
}
uint8_t SD_isDetected_dy(void)
{
	uint8_t status = HAL_ERROR;

	//status =
	if(HAL_GPIO_ReadPin(uSD_Detect_GPIO_Port, uSD_Detect_Pin) == GPIO_PIN_RESET)
	{
	  TxBufLen = sprintf(TxBuf, "[OK] SD detected \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
	  status = HAL_OK;
	}
	else
	{
	  TxBufLen = sprintf(TxBuf, "[ERROR] SD not detected \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
	  status = HAL_ERROR;
	}
	return status;
}
uint8_t SD_Link_dy(void)
{
	uint8_t status = HAL_ERROR;

	FATFS_UnLinkDriver(SDPath);
	HAL_Delay(100);
	if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
	{
	  TxBufLen = sprintf(TxBuf, "[OK] SD link driver \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
	  status = HAL_OK;
	}
	else
	{
	  TxBufLen = sprintf(TxBuf, "[ERROR] SD link driver \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
	  status = HAL_ERROR;
	}
	return status;
}
uint8_t SD_Mount_dy(void)
{
	uint8_t status = HAL_ERROR;

	if((f_mount(&SDFatFS, SDPath, 0)) == FR_OK)
	//if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) == FR_OK)
	{
	  TxBufLen = sprintf(TxBuf, "[OK] sd mount \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
	  status = HAL_OK;

	  //char SDPath[4];
	  //strcpy(SDPath, "0:/");
	}
	else
	{
	  TxBufLen = sprintf(TxBuf, "[ERROR] sd mount \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
	  status = HAL_ERROR;
	}
	return status;
}
uint8_t SD_GetInfo_dy(void)
{
	uint8_t status = HAL_ERROR;

	HAL_SD_CardInfoTypeDef cardInfo;
	HAL_SD_GetCardInfo(&hsd1, &cardInfo);

#if 0
	if (cardInfo.CardType != 0) {
		TxBufLen = sprintf(TxBuf, "[OK] sd info \r\n");
		HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
		HAL_Delay(100);
		status = HAL_OK;
	}
	else
	{
		TxBufLen = sprintf(TxBuf, "[ERROR] sd info - %d \r\n", status);
		HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
		HAL_Delay(100);
		status = HAL_ERROR;
	}
#endif
	return status;
}

uint8_t SD_Format(void)
{
	uint8_t status = HAL_ERROR;

    if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer)) == FR_OK)
    {
      /* FatFs Format Error */
		TxBufLen = sprintf(TxBuf, "[OK] sd format \r\n");
		HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
		HAL_Delay(100);
		status = HAL_OK;
    }
    else
    {
		TxBufLen = sprintf(TxBuf, "[ERROR] sd format \r\n");
		HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
		HAL_Delay(100);
		status = HAL_ERROR;
    }
    return status;
}

void SD_MspDeInit_dy(SD_HandleTypeDef *hsd, void *Params)
{
  static DMA_HandleTypeDef dma_rx_handle;
  static DMA_HandleTypeDef dma_tx_handle;

  /* Disable NVIC for DMA transfer complete interrupts */
  HAL_NVIC_DisableIRQ(SD_DMAx_Rx_IRQn);
  HAL_NVIC_DisableIRQ(SD_DMAx_Tx_IRQn);

  /* Deinitialize the stream for new transfer */
  dma_rx_handle.Instance = SD_DMAx_Rx_STREAM;
  HAL_DMA_DeInit(&dma_rx_handle);

  /* Deinitialize the stream for new transfer */
  dma_tx_handle.Instance = SD_DMAx_Tx_STREAM;
  HAL_DMA_DeInit(&dma_tx_handle);

  /* Disable NVIC for SDIO interrupts */
  HAL_NVIC_DisableIRQ(SDMMC1_IRQn);

  /* DeInit GPIO pins can be done in the application
     (by surcharging this __weak function) */

  /* Disable SDMMC1 clock */
  __HAL_RCC_SDMMC1_CLK_DISABLE();

  /* GPIO pins clock and DMA clocks can be shut down in the application
     by surcharging this __weak function */
}

void SD_MspInit_dy(SD_HandleTypeDef *hsd, void *Params)
{
  static DMA_HandleTypeDef dma_rx_handle;
  static DMA_HandleTypeDef dma_tx_handle;
  GPIO_InitTypeDef gpio_init_structure;

  /* Enable SDIO clock */
  __HAL_RCC_SDMMC1_CLK_ENABLE();

  /* Enable DMA2 clocks */
  __DMAx_TxRx_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Common GPIO configuration */
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = GPIO_AF12_SDMMC1;

  /* GPIOC configuration */
  gpio_init_structure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &gpio_init_structure);

  /* GPIOD configuration */
  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

  /* NVIC configuration for SDIO interrupts */
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 0x0E, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

  /* Configure DMA Rx parameters */
  dma_rx_handle.Init.Channel             = SD_DMAx_Rx_CHANNEL;
  dma_rx_handle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  dma_rx_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dma_rx_handle.Init.MemInc              = DMA_MINC_ENABLE;
  dma_rx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dma_rx_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dma_rx_handle.Init.Mode                = DMA_PFCTRL;
  dma_rx_handle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dma_rx_handle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dma_rx_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dma_rx_handle.Init.MemBurst            = DMA_MBURST_INC4;
  dma_rx_handle.Init.PeriphBurst         = DMA_PBURST_INC4;

  dma_rx_handle.Instance = SD_DMAx_Rx_STREAM;

  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmarx, dma_rx_handle);

  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&dma_rx_handle);

  /* Configure the DMA stream */
  HAL_DMA_Init(&dma_rx_handle);

  /* Configure DMA Tx parameters */
  dma_tx_handle.Init.Channel             = SD_DMAx_Tx_CHANNEL;
  dma_tx_handle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  dma_tx_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dma_tx_handle.Init.MemInc              = DMA_MINC_ENABLE;
  dma_tx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dma_tx_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dma_tx_handle.Init.Mode                = DMA_PFCTRL;
  dma_tx_handle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dma_tx_handle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dma_tx_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dma_tx_handle.Init.MemBurst            = DMA_MBURST_INC4;
  dma_tx_handle.Init.PeriphBurst         = DMA_PBURST_INC4;

  dma_tx_handle.Instance = SD_DMAx_Tx_STREAM;

  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmatx, dma_tx_handle);

  /* Deinitialize the stream for new transfer */
  HAL_DMA_DeInit(&dma_tx_handle);

  /* Configure the DMA stream */
  HAL_DMA_Init(&dma_tx_handle);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(SD_DMAx_Rx_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(SD_DMAx_Rx_IRQn);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(SD_DMAx_Tx_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(SD_DMAx_Tx_IRQn);
}
uint8_t SD_DeInit_dy(void)
{
  uint8_t sd_state = MSD_OK;

  hsd1.Instance = SDMMC1;

  /* HAL SD deinitialization */
  if(HAL_SD_DeInit(&hsd1) != HAL_OK)
  {
    sd_state = MSD_ERROR;
  }

  /* Msp SD deinitialization */
  hsd1.Instance = SDMMC1;
  SD_MspDeInit_dy(&hsd1, NULL);

  return  sd_state;
}
uint8_t SD_Init_dy(void)
{
	  uint8_t sd_state = MSD_OK;

	  /* uSD device interface configuration */
	  hsd1.Instance = SDMMC1;

	  hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
	  hsd1.Init.ClockBypass         = SDMMC_CLOCK_BYPASS_DISABLE;
	  hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	  hsd1.Init.BusWide             = SDMMC_BUS_WIDE_1B;
	  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	  hsd1.Init.ClockDiv            = SDMMC_TRANSFER_CLK_DIV;

	  SD_isDetected_dy();

	  /* Msp SD initialization */
	  SD_MspInit_dy(&hsd1, NULL);

	  /* HAL SD initialization */
	  if(HAL_SD_Init(&hsd1) != HAL_OK)
	  {
	    sd_state = MSD_ERROR;
	  }

	  /* Configure SD Bus width */
	  if(sd_state == MSD_OK)
	  {
	    /* Enable wide operation */
	    if(HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
	    {
	      sd_state = MSD_ERROR;
	    }
	    else
	    {
	      sd_state = MSD_OK;
	    }
	  }

	  return  sd_state;
}

int8_t* load(const char* file, size_t * size)
{
#if 0
	size_t sz;
	FILE* fp = fopen(file,"rb");
	int8_t* input;
	assert(fp);
	fseek(fp, 0, SEEK_END);
	sz = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	input = malloc(sz);
	fread(input, 1, sz, fp);
	fclose(fp);
	*size = sz;
	return input;
#endif
	size_t sz;
	uint8_t ret;
	ret = f_open(&SDFile, file, FA_READ);
	int8_t* input;

	//f_lseek(&SDFile, SEEK_END);
	sz = f_size(&SDFile);
	//f_lseek(&SDFile, SEEK_SET);
	input = malloc(sz);
	f_read(&SDFile, input, sz, (UINT*)&bytesread);
	f_close(&SDFile);
	*size = sz;
	return input;

}



int8_t* load_file(const char* filename, uint32_t* size)
{
#if 0
    FILE* fp;
    uint8_t* buffer;

    fp = fopen(filename, "rb");
    if (fp == NULL) {
        printf("Failed to open file %s.\n", filename);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    *size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    buffer = malloc(*size);
    if (buffer == NULL) {
        printf("Failed to allocate memory.\n");
        fclose(fp);
        return NULL;
    }
    fread(buffer, 1, *size, fp);

    fclose(fp);

    return buffer;
#endif
    uint8_t ret;
    int8_t* buffer;

    ret = f_open(&SDFile, TestSet, FA_READ);
    if (ret) {

        return NULL;
    }

    f_lseek(&SDFile, SEEK_END);
    *size = f_tell(&SDFile);
    f_lseek(&SDFile, SEEK_SET);

    buffer = malloc(*size);
    if (buffer == NULL) {
        //printf("Failed to allocate memory.\n");
        f_close(&SDFile);
        return NULL;
    }
    f_read(&SDFile, buffer, *size, (UINT*)&bytesread);

    f_close(&SDFile);

    return buffer;
}
nnom_status_t callback(nnom_model_t* m, nnom_layer_t* layer)
{
#if 0
	float scale = 1 << (layer->out->tensor->q_dec[0]);
	printf("\nOutput of Layer %s", default_layer_names[layer->type]);

	for (int i = 0; i < tensor_size(layer->out->tensor); i++)
	{
		if (i % 16 == 0)
			printf("\n");
		printf("%f ", (float)((int8_t*)layer->out->tensor->p_data)[i] / scale);
	}
	printf("\n");
	return NN_SUCCESS;
#endif
	float scale = 1 << (layer->out->tensor->q_dec[0]);
	TxBufLen = sprintf(TxBuf, "\nOutput of Layer %s ", default_layer_names[layer->type]);
	HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);

	for (int i = 0; i < tensor_size(layer->out->tensor); i++)
	{
		if (i % 16 == 0)
			HAL_UART_Transmit(&huart6, (uint8_t * )"\n", 2, 100);
		TxBufLen = sprintf(TxBuf, "%f ", (float)((int8_t*)layer->out->tensor->p_data)[i] / scale);
		HAL_UART_Transmit(&huart6, (uint8_t * ) TxBuf, TxBufLen, 100);
	}
	HAL_UART_Transmit(&huart6, (uint8_t * )"\n", 1, 100);
	return NN_SUCCESS;
}

#ifdef NNOM_USING_STATIC_MEMORY
uint8_t static_buf[1024 * 500];
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_TIM7_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_UART_Receive_IT(&huart6, &rx6_byte, 100);
  TimerCount=0;
  TimerCountSec=0;
  HAL_Delay(100);
  SD_Init_dy();

  HAL_SD_CardInfoTypeDef cardInfo;
  //HAL_SD_CardStatusTypeDef cardStatus;
  //uint32_t result_sd = 0;
  HAL_SD_GetCardInfo(&hsd1, &cardInfo);

  CheckRetVal = HAL_SD_GetState(&hsd1);

  TxBufLen = sprintf(TxBuf, "[START] Initialize is done \r\n");
  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
  HAL_Delay(100);

  CheckRetVal = SD_isDetected_dy();
  CheckRetVal = SD_Link_dy();
  CheckRetVal = SD_Mount_dy();
  //CheckRetVal = SD_Format();

  if(f_open(&SDFile, "test6.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
  {
	  TxBufLen = sprintf(TxBuf, "[OK] sd open \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
  }
  else
  {
	  TxBufLen = sprintf(TxBuf, "[ERROR] sd open -  \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
  }
  CheckRetVal = f_write(&SDFile, wtext, sizeof(wtext), (void *)&byteswritten);
  if((byteswritten == 0) || (CheckRetVal != FR_OK))
	{
	  TxBufLen = sprintf(TxBuf, "[ERROR] sd write -  \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
	}
	else
	{
	  TxBufLen = sprintf(TxBuf, "[OK] sd write -  \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
	  f_close(&SDFile);
	}
  if(f_open(&SDFile, "test6.TXT", FA_READ) != FR_OK)
  {
	  TxBufLen = sprintf(TxBuf, "[ERROR] sd re open -  \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
  }
  else
  {
	  TxBufLen = sprintf(TxBuf, "[OK] sd re open -  \r\n");
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(100);
	  CheckRetVal = f_read(&SDFile, rtext, sizeof(rtext), (UINT*)&bytesread);
      if((bytesread == 0) || (CheckRetVal != FR_OK))
      {
		  TxBufLen = sprintf(TxBuf, "[ERROR] sd read  -  \r\n");
		  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
		  HAL_Delay(100);
      }
      else
      {
    	  f_close(&SDFile);
		  TxBufLen = sprintf(TxBuf, "[OK] sd read  -  \r\n");
		  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
		  HAL_Delay(100);
    	  if((bytesread != byteswritten))
		  {
    		  TxBufLen = sprintf(TxBuf, "[ERROR] sd read and write matched -  \r\n");
    		  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
    		  HAL_Delay(100);
		  }
    	  else
    	  {
    		  TxBufLen = sprintf(TxBuf, "[OK] sd read and write matched -  \r\n");
    		  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
    		  HAL_Delay(100);
    	  }
      }
  }
  f_close(&SDFile);

  TxBufLen = sprintf(TxBuf, "[DONE] init-functions are done \r\n");
  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
  HAL_Delay(100);


  TxBufLen = sprintf(TxBuf, "[DONE] Timer test \r\n");
  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
  HAL_Delay(1000);
  TimerCount = 0;
  for(int i=0;i<10;i++)
  {
	  TxBufLen = sprintf(TxBuf, "[TEST] Timer count test - %ld \r\n", TimerCount);
	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	  HAL_Delay(500);
  }

  for(int i=0;i<10;i++)
  {
	  PrintTimeStamp();

	  HAL_Delay(500);
  }





  char udata[100];
  unsigned int udata_len;

  uint32_t timestamp;
  uint32_t duration_cal;


  TimerCount = 0;
  htim7.Instance->CNT = 0;
  timestamp = (1000*TimerCount) + htim7.Instance->CNT;


  udata_len = sprintf(udata, "create begin nnom--> timestart %ld \r\n", timestamp);
  HAL_UART_Transmit(&huart6, (uint8_t * ) udata, udata_len, 100);

    //FILE* fp;
  	nnom_model_t* model;
  	nnom_predict_t * pre;
  	//
  	int8_t* input;
  	float prob;
  	uint32_t label;
  	size_t size = 0;


    CheckRetVal = SD_isDetected_dy();
    CheckRetVal = SD_Link_dy();
    CheckRetVal = SD_Mount_dy();
    input = load("test_data.bin", &size);
    if((f_open(&SDFile, "result.csv", FA_CREATE_ALWAYS | FA_WRITE)) == FR_OK)
    {
  	  TxBufLen = sprintf(TxBuf, "[OK] sd open \r\n");
  	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
  	  HAL_Delay(100);
    }
    else
    {
  	  TxBufLen = sprintf(TxBuf, "[ERROR] sd open -  \r\n");
  	  HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
  	  HAL_Delay(100);
    }
    f_printf(&SDFile, "label, prob\n");

	TxBufLen = sprintf(TxBuf, "validation size: %ld\n", (uint32_t)size);
	HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
	HAL_Delay(100);

    //input =
#if 0
  	int8_t* input = (int8_t*) calloc(756, sizeof(int8_t));
  	for (int i = 0; i < 756; i++) {
  	    input[i] = 0;
  	}

  	size = 755;
#endif
  	//printf("validation size: %d\n", (uint32_t)size);


  #ifdef NNOM_USING_STATIC_MEMORY
  	// when use static memory buffer, we need to set it before create
  	nnom_set_static_buf(static_buf, sizeof(static_buf));
  #endif

  	model = nnom_model_create();				// create NNoM model
  	pre = prediction_create(model, nnom_output_data, sizeof(nnom_output_data), 4); // mnist, 10 classes, get top-4
  	//model_set_callback(model, callback);

	// now takes label and data from the file and data
	for(size_t seek=0; seek < size;)
	{
		// labels
		uint8_t true_label[128];
		memcpy(true_label, input + seek, 128);
		seek += 128;
		// data
		for(int i=0; i < 128; i++)
		{
			if(seek >= size)
				break;
			memcpy(nnom_input_data, input + seek, sizeof(nnom_input_data));
			seek += sizeof(nnom_input_data);

			//nnom_predict(model, &label, &prob);				// this will work independently
			prediction_run(pre, true_label[i], &label, &prob);  // this provide more infor but requires prediction API

			// save results
			f_printf(&SDFile, "%d,%f\n", label, prob);
			TxBufLen = sprintf(TxBuf, "%ld,%f\n", label, prob);
			HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);

		}

		TxBufLen = sprintf(TxBuf, "Processing %ld%%\n", (uint32_t)(seek * 100 / size));
		HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);
		HAL_Delay(100);
	}
	f_close(&SDFile);

  	// print prediction result
  	prediction_end(pre);
  	prediction_summary(pre);
  	prediction_delete(pre);

  	// model
  	model_stat(model);
  	model_delete(model);



  duration_cal = ((1000*TimerCount) + htim7.Instance->CNT) - timestamp;
  udata_len = sprintf(udata, "create end nnom--> duration %ld us \r\n", duration_cal);
  HAL_UART_Transmit(&huart6, (uint8_t * ) udata, udata_len, 100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  HAL_Delay(5000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_B0_Pin */
  GPIO_InitStruct.Pin = LCD_B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(LCD_B0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_NBL1_Pin FMC_NBL0_Pin FMC_D5_Pin FMC_D6_Pin
                           FMC_D8_Pin FMC_D11_Pin FMC_D4_Pin FMC_D7_Pin
                           FMC_D9_Pin FMC_D12_Pin FMC_D10_Pin */
  GPIO_InitStruct.Pin = FMC_NBL1_Pin|FMC_NBL0_Pin|FMC_D5_Pin|FMC_D6_Pin
                          |FMC_D8_Pin|FMC_D11_Pin|FMC_D4_Pin|FMC_D7_Pin
                          |FMC_D9_Pin|FMC_D12_Pin|FMC_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin|ARDUINO_SDA_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D6_Pin DCMI_D7_Pin */
  GPIO_InitStruct.Pin = DCMI_D6_Pin|DCMI_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_SDNCAS_Pin FMC_SDCLK_Pin FMC_A11_Pin FMC_A10_Pin
                           FMC_BA1_Pin FMC_BA0_Pin */
  GPIO_InitStruct.Pin = FMC_SDNCAS_Pin|FMC_SDCLK_Pin|FMC_A11_Pin|FMC_A10_Pin
                          |FMC_BA1_Pin|FMC_BA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_B1_Pin LCD_B2_Pin LCD_B3_Pin LCD_G4_Pin
                           LCD_G1_Pin LCD_G3_Pin LCD_G0_Pin LCD_G2_Pin
                           LCD_R7_Pin LCD_R5_Pin LCD_R6_Pin LCD_R4_Pin
                           LCD_R3_Pin LCD_R1_Pin LCD_R2_Pin */
  GPIO_InitStruct.Pin = LCD_B1_Pin|LCD_B2_Pin|LCD_B3_Pin|LCD_G4_Pin
                          |LCD_G1_Pin|LCD_G3_Pin|LCD_G0_Pin|LCD_G2_Pin
                          |LCD_R7_Pin|LCD_R5_Pin|LCD_R6_Pin|LCD_R4_Pin
                          |LCD_R3_Pin|LCD_R1_Pin|LCD_R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_D2_Pin FMC_D3_Pin FMC_D1_Pin FMC_D15_Pin
                           FMC_D0_Pin FMC_D14_Pin FMC_D13_Pin */
  GPIO_InitStruct.Pin = FMC_D2_Pin|FMC_D3_Pin|FMC_D1_Pin|FMC_D15_Pin
                          |FMC_D0_Pin|FMC_D14_Pin|FMC_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin OTG_FS_ID_Pin */
  GPIO_InitStruct.Pin = OTG_FS_P_Pin|OTG_FS_N_Pin|OTG_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
  GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DE_Pin LCD_B7_Pin LCD_B6_Pin LCD_B5_Pin
                           LCD_G6_Pin LCD_G7_Pin LCD_G5_Pin */
  GPIO_InitStruct.Pin = LCD_DE_Pin|LCD_B7_Pin|LCD_B6_Pin|LCD_B5_Pin
                          |LCD_G6_Pin|LCD_G7_Pin|LCD_G5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_B4_Pin */
  GPIO_InitStruct.Pin = LCD_B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
  HAL_GPIO_Init(LCD_B4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SDB_Pin */
  GPIO_InitStruct.Pin = SAI2_SDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_D5_Pin */
  GPIO_InitStruct.Pin = DCMI_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_A0_Pin FMC_A1_Pin FMC_A2_Pin FMC_A3_Pin
                           FMC_A4_Pin FMC_A5_Pin FMC_A6_Pin FMC_A9_Pin
                           FMC_A7_Pin FMC_A8_Pin FMC_SDNRAS_Pin */
  GPIO_InitStruct.Pin = FMC_A0_Pin|FMC_A1_Pin|FMC_A2_Pin|FMC_A3_Pin
                          |FMC_A4_Pin|FMC_A5_Pin|FMC_A6_Pin|FMC_A9_Pin
                          |FMC_A7_Pin|FMC_A8_Pin|FMC_SDNRAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_HSYNC_Pin LCD_VSYNC_Pin LCD_R0_Pin LCD_CLK_Pin */
  GPIO_InitStruct.Pin = LCD_HSYNC_Pin|LCD_VSYNC_Pin|LCD_R0_Pin|LCD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_VSYNC_Pin */
  GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_SCK_D13_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCK_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(ARDUINO_SCK_D13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_PWR_EN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D4_Pin DCMI_D3_Pin DCMI_D0_Pin DCMI_D2_Pin
                           DCMI_D1_Pin */
  GPIO_InitStruct.Pin = DCMI_D4_Pin|DCMI_D3_Pin|DCMI_D0_Pin|DCMI_D2_Pin
                          |DCMI_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_CS_D5_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(ARDUINO_PWM_CS_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_TX_Pin */
  GPIO_InitStruct.Pin = VCP_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D10_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(ARDUINO_PWM_D10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_SDNME_Pin FMC_SDNE0_Pin */
  GPIO_InitStruct.Pin = FMC_SDNME_Pin|FMC_SDNE0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_A4_Pin ARDUINO_A5_Pin ARDUINO_A1_Pin ARDUINO_A2_Pin
                           ARDUINO_A3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A4_Pin|ARDUINO_A5_Pin|ARDUINO_A1_Pin|ARDUINO_A2_Pin
                          |ARDUINO_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : FMC_SDCKE0_Pin */
  GPIO_InitStruct.Pin = FMC_SDCKE0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(FMC_SDCKE0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_A0_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARDUINO_A0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_HSYNC_Pin PA6 */
  GPIO_InitStruct.Pin = DCMI_HSYNC_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_SCL_Pin LCD_SDA_Pin */
  GPIO_InitStruct.Pin = LCD_SCL_Pin|LCD_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D6_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(ARDUINO_PWM_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
  GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
