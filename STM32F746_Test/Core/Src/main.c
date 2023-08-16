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
//#include "weights1.h"
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

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
unsigned int TimerDelay=0;
uint64_t TimerCount;
uint64_t TimerCountSec;

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
uint8_t wtext[] = "Checking time"; /* File write buffer */
uint8_t rtext[255];                                   /* File read buffer */

char TestSet[14] = "test_data.bin";//"test_data.bin";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM6_Init(void);
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
	unsigned char input_data_third[7968] =
			{
					7,	2,	1,	0,	4,	1,	4,	9,	5,	9,	0,	6,	9,	0,	1,	5,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,

					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	42,	92,	79,	75,	30,	18,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	111,	127,	127,	127,	127,	120,	99,	99,	99,	99,	99,	99,	99,	99,	85,	26,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	33,	57,	36,	57,	81,	113,	127,	112,	127,	127,	127,	125,	114,	127,	127,	70,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	8,	33,	7,	33,	33,	33,	29,	10,	118,	127,	53,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	41,	126,	104,	9,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	11,	116,	127,	41,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	64,	127,	119,	22,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	29,	124,	127,	31,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	66,	127,	93,	2,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	4,	102,	124,	29,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	63,	127,	91,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	37,	125,	120,	28,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	9,	110,	127,	83,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	101,	127,	109,	17,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	19,	127,	127,	38,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	15,	112,	127,	57,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	66,	127,	127,	26,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	30,	121,	127,	127,	26,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	60,	127,	127,	109,	20,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	60,	127,	103,	9,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,


					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	58,	62,	85,	127,	127,	75,	46,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	84,	126,	126,	126,	126,	126,	126,	109,	15,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	84,	126,	126,	126,	106,	71,	88,	126,	126,	61,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	26,	125,	126,	105,	16,	6,	0,	3,	103,	126,	70,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	38,	125,	105,	12,	0,	0,	0,	61,	124,	126,	32,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	15,	9,	0,	0,	0,	0,	104,	126,	126,	32,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	58,	123,	126,	99,	5,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	38,	123,	126,	115,	31,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	64,	126,	126,	72,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	88,	123,	126,	79,	6,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	12,	117,	126,	116,	17,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	99,	126,	126,	70,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	39,	124,	126,	94,	6,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	9,	100,	126,	126,	70,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	67,	126,	126,	86,	6,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	124,	126,	126,	12,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	124,	126,	126,	21,	10,	10,	10,	10,	2,	0,	2,	10,	10,	18,	75,	75,	75,	73,	5,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	124,	126,	126,	126,	126,	126,	126,	126,	84,	71,	83,	126,	126,	126,	126,	126,	126,	126,	61,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	87,	126,	126,	126,	126,	126,	126,	126,	126,	126,	126,	126,	124,	123,	123,	84,	58,	58,	28,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	59,	61,	61,	61,	83,	126,	126,	126,	77,	61,	61,	20,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,


					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	19,	127,	54,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	43,	126,	41,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	67,	120,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	22,	122,	75,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	42,	127,	31,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	101,	111,	5,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	16,	127,	108,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	47,	127,	97,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	70,	127,	38,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	28,	118,	102,	4,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	62,	127,	82,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	85,	127,	40,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	12,	116,	107,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	60,	127,	79,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	75,	127,	71,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	114,	127,	33,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	30,	125,	127,	33,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	70,	127,	102,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	5,	107,	127,	60,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	2,	99,	88,	5,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,


					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	5,	75,	126,	101,	15,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	18,	125,	125,	126,	53,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	10,	98,	125,	125,	126,	53,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	55,	95,	125,	125,	125,	126,	84,	54,	31,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	126,	125,	125,	125,	125,	126,	125,	125,	110,	25,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	91,	127,	126,	126,	126,	126,	117,	111,	126,	126,	126,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	31,	110,	126,	125,	125,	125,	73,	38,	31,	64,	125,	125,	52,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	16,	115,	125,	126,	125,	110,	68,	5,	0,	0,	15,	115,	125,	121,	56,	2,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	18,	125,	125,	126,	94,	10,	0,	0,	0,	0,	0,	54,	125,	126,	125,	17,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	18,	125,	125,	100,	15,	0,	0,	0,	0,	0,	0,	15,	100,	126,	125,	17,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	18,	126,	126,	0,	0,	0,	0,	0,	0,	0,	0,	16,	101,	127,	126,	82,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	70,	125,	125,	0,	0,	0,	0,	0,	0,	0,	0,	54,	125,	126,	125,	17,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	108,	125,	125,	0,	0,	0,	0,	0,	0,	10,	31,	115,	125,	126,	115,	15,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	108,	125,	125,	0,	0,	0,	0,	0,	0,	72,	125,	125,	125,	110,	30,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	108,	125,	125,	0,	0,	0,	0,	0,	91,	110,	125,	125,	125,	90,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	109,	126,	126,	36,	36,	114,	126,	126,	127,	126,	126,	126,	126,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	56,	125,	125,	126,	125,	125,	125,	125,	126,	125,	125,	125,	73,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	15,	115,	125,	126,	125,	125,	125,	125,	126,	115,	94,	17,	5,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	31,	71,	126,	125,	125,	125,	125,	126,	53,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	36,	87,	125,	86,	35,	36,	15,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,


					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	25,	112,	0,	0,	0,	0,	0,	0,	0,	35,	14,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	60,	115,	0,	0,	0,	0,	0,	0,	0,	74,	84,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	2,	97,	115,	0,	0,	0,	0,	0,	0,	0,	48,	105,	5,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	34,	126,	67,	0,	0,	0,	0,	0,	0,	0,	57,	126,	10,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	22,	118,	108,	6,	0,	0,	0,	0,	0,	0,	0,	96,	126,	10,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	84,	123,	26,	0,	0,	0,	0,	0,	0,	0,	9,	127,	126,	10,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	42,	121,	105,	0,	0,	0,	0,	0,	0,	0,	0,	70,	126,	94,	2,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	84,	126,	53,	0,	0,	0,	0,	0,	0,	0,	16,	116,	125,	33,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	7,	112,	126,	0,	0,	0,	0,	0,	0,	0,	0,	67,	126,	105,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	11,	126,	82,	0,	0,	0,	0,	0,	0,	0,	0,	84,	126,	83,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	4,	102,	104,	9,	0,	0,	0,	0,	0,	0,	11,	126,	126,	53,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	84,	126,	99,	42,	42,	42,	42,	64,	82,	97,	126,	126,	53,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	20,	85,	122,	126,	126,	126,	126,	116,	115,	125,	126,	126,	4,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	24,	42,	42,	42,	42,	0,	0,	80,	126,	126,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	63,	126,	126,	22,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	64,	126,	126,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	63,	126,	126,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	67,	126,	122,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	116,	118,	55,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	89,	33,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,


					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	38,	127,	53,	1,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	9,	113,	127,	127,	4,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	40,	127,	127,	82,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	3,	101,	127,	127,	36,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	26,	127,	127,	125,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	67,	127,	127,	90,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	98,	127,	124,	24,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	29,	127,	127,	118,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	55,	127,	127,	66,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	81,	127,	119,	14,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	30,	126,	127,	111,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	39,	127,	127,	77,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	81,	127,	119,	26,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	14,	126,	127,	105,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	43,	127,	127,	65,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	52,	127,	117,	10,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	87,	127,	102,	2,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	2,	105,	127,	98,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	79,	127,	80,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	13,	78,	53,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,


					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	11,	96,	67,	16,	0,	0,	0,	0,	0,	0,	0,	0,	7,	38,	2,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	8,	117,	125,	84,	0,	0,	0,	0,	0,	0,	0,	0,	7,	110,	120,	18,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	10,	94,	126,	73,	0,	0,	0,	0,	0,	0,	0,	0,	0,	69,	126,	50,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	35,	126,	126,	10,	0,	0,	0,	0,	0,	0,	0,	0,	21,	127,	86,	6,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	11,	76,	126,	48,	0,	0,	0,	0,	0,	0,	0,	0,	21,	115,	127,	46,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	81,	127,	102,	5,	0,	0,	0,	0,	0,	0,	0,	0,	52,	127,	79,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	81,	126,	89,	2,	0,	0,	0,	0,	0,	0,	4,	65,	118,	126,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	81,	126,	126,	95,	87,	35,	35,	35,	35,	66,	98,	126,	126,	84,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	25,	114,	126,	126,	127,	126,	126,	126,	126,	127,	126,	126,	109,	17,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	8,	32,	68,	127,	116,	68,	68,	68,	22,	126,	126,	80,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	17,	127,	103,	10,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	80,	126,	34,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	42,	127,	120,	25,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	79,	127,	82,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	115,	122,	25,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	52,	127,	116,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	104,	126,	78,	0,	6,	15,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	104,	126,	77,	45,	102,	80,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	104,	126,	127,	126,	77,	14,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	30,	95,	64,	11,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,


					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	7,	74,	96,	2,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	45,	112,	126,	126,	9,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	14,	117,	127,	126,	126,	83,	9,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	72,	126,	127,	126,	126,	126,	119,	57,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	15,	120,	126,	104,	92,	126,	126,	126,	115,	12,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	39,	127,	96,	0,	4,	49,	109,	127,	127,	100,	9,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	43,	126,	40,	0,	0,	0,	91,	126,	127,	95,	6,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	87,	126,	77,	0,	0,	0,	117,	126,	127,	67,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	43,	126,	104,	20,	42,	83,	125,	118,	127,	118,	21,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	9,	119,	126,	127,	126,	126,	92,	18,	108,	126,	76,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	34,	120,	127,	127,	72,	4,	0,	67,	127,	111,	17,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	34,	79,	71,	6,	0,	0,	4,	87,	126,	80,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	44,	126,	113,	9,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	83,	126,	63,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	24,	122,	126,	19,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	57,	127,	86,	4,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	10,	109,	127,	23,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	15,	127,	82,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	93,	122,	21,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	7,	111,	39,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,


					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	8,	23,	23,	23,	8,	64,	42,	23,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	37,	76,	108,	126,	126,	126,	107,	123,	126,	126,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	17,	71,	122,	126,	126,	126,	126,	126,	126,	126,	126,	126,	126,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	31,	126,	126,	126,	126,	126,	126,	126,	106,	85,	85,	85,	85,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	10,	66,	36,	0,	28,	119,	113,	119,	84,	62,	34,	10,	5,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	5,	103,	126,	39,	0,	0,	16,	0,	15,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	3,	88,	126,	66,	5,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	6,	66,	126,	116,	7,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	46,	126,	111,	14,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	75,	126,	87,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	117,	126,	123,	63,	24,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	127,	126,	126,	126,	125,	73,	45,	60,	42,	21,	21,	42,	14,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	69,	126,	126,	126,	126,	126,	126,	126,	126,	126,	126,	126,	116,	84,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	1,	26,	109,	111,	125,	126,	126,	126,	126,	126,	126,	126,	126,	126,	62,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	33,	36,	100,	126,	126,	126,	126,	126,	126,	126,	87,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	60,	126,	124,	76,	25,	82,	126,	126,	87,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	25,	126,	126,	126,	94,	126,	126,	126,	74,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	4,	83,	126,	126,	126,	126,	125,	87,	5,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	11,	90,	115,	126,	110,	64,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	46,	74,	11,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,


					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	18,	28,	68,	100,	99,	47,	18,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	22,	76,	117,	127,	127,	127,	127,	127,	125,	105,	75,	3,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	23,	76,	120,	127,	127,	113,	83,	66,	125,	100,	127,	114,	112,	52,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	76,	117,	127,	127,	93,	71,	4,	0,	0,	95,	20,	99,	123,	111,	126,	10,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	4,	63,	126,	127,	116,	64,	5,	0,	0,	0,	0,	105,	21,	35,	127,	127,	127,	10,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	36,	121,	127,	114,	27,	0,	0,	0,	0,	1,	16,	58,	112,	121,	127,	127,	81,	2,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	37,	120,	127,	111,	54,	69,	89,	89,	84,	105,	125,	115,	127,	127,	127,	116,	19,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	4,	87,	12,	126,	127,	127,	127,	125,	127,	127,	127,	127,	127,	126,	85,	12,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	8,	68,	97,	88,	73,	76,	100,	127,	127,	127,	127,	75,	8,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	81,	127,	127,	120,	49,	1,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	59,	125,	127,	127,	45,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	50,	121,	127,	127,	105,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	27,	120,	127,	127,	121,	29,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	65,	127,	127,	122,	32,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	6,	124,	127,	127,	76,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	6,	114,	127,	127,	104,	4,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	39,	127,	127,	127,	33,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	104,	127,	127,	68,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	113,	127,	116,	12,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	56,	127,	54,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
					0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0
			};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /*
  	uint64_t timer_start;
    uint64_t timer_end,timer_end2;
    uint64_t SysTickCnt_start;
    uint64_t SysTickCnt_end;
    uint64_t SysTickCnt_ms;
    uint64_t us_SysTickCnt_start;
    uint64_t us_SysTickCnt_end;
    uint64_t SysTickCnt_us;

 	 HAL_TIM_Base_Start(&htim6);
	char uart_buf[50];
  	int uart_buf_len;
	uint16_t timer_val,timer_val1;

	timer_val = __HAL_TIM_GET_COUNTER(&htim6);

	// 50ms
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	// SAVE TIME
	timer_val1 = __HAL_TIM_GET_COUNTER(&htim6) - timer_val;


	// SHOW TIME
	uart_buf_len = sprintf(uart_buf, "%u us\r\n", timer_val1);
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);

	void delay_us(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);              // ???��머�?? 0?���?????? 초기?��
	while((__HAL_TIM_GET_COUNTER(&htim1))<time);   // ?��?��?�� ?��간까�?????? ??�??????
	}

	HAL_TIM_Base_Start(&htim1);  // delay_ms() ?��?���?????? ?��?�� ???���?????? ?��?��

	GPIOA->BRR = GPIO_PIN_10;
	delay_us(10);
	GPIOA->BSRR = GPIO_PIN_10;

  */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_USART6_UART_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_TIM11_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  char uart_buf[50];
  int uart_buf_len=0;
  HAL_TIM_Base_Start(&htim11);
  HAL_TIM_Base_Start(&htim6);
  //uint64_t timer_val,timer_val1=0;

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

  //timer_val = TimerCount
  //timer_val = __HAL_TIM_GET_COUNTER(&htim6);

  	  //timer_val = __HAL_TIM_GET_COUNTER(&htim11);
  //timer_val1 = __HAL_TIM_GET_COUNTER(&htim11) - timer_val;
  	  //uart_buf_len = sprintf(uart_buf, "%u us\r\n", timer_val1);
  	  //HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, uart_buf_len, 100);
  __HAL_TIM_SET_COUNTER(&htim6,0);

  TimerCount = 0;
  for(int i=0;i<10;i++)
  {
	  PrintTimeStamp();
//	  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
//	  HAL_Delay(100);
//	  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);
//	  HAL_Delay(100);
  }
/*
  timer_val = __HAL_TIM_GET_COUNTER(&htim16);

 	  // 50ms
 	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
 	  HAL_Delay(50);
 	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

 	  // SAVE TIME
 	  timer_val1 = __HAL_TIM_GET_COUNTER(&htim16) - timer_val;


 	  // SHOW TIME
 	  uart_buf_len = sprintf(uart_buf, "%u us\r\n", timer_val1);
 	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);

 	  //
 	  HAL_Delay(1000);
 	  for(int i=0; i<5; i++)
 	  {
 		  HAL_Delay(2000);
 			nnom_model_t* model;
 			nnom_predict_t * pre;
 			//int8_t* input;
 			float prob;
 			uint32_t label;
 			uint64_t size = 7968; 	////912 // 1696	// 7968
 			uint8_t buf[100];
 			uint8_t buf1[100];
 			uint8_t buf2[100];
 			int TxBufferlen,TxBufferlen1,TxBufferlen2;
 			uint64_t timer_start;
 			uint64_t timer_end,timer_end2;
 			long long SysTickCnt_ms;
 			long long SysTickCnt_start, SysTickCnt_end;
 			*/

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
  	//size_t size1 = 7968;

    CheckRetVal = SD_isDetected_dy();
    CheckRetVal = SD_Link_dy();
    CheckRetVal = SD_Mount_dy();

  	//input = load("test_data.txt", &size);
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



  #if 0
  	int8_t* input = (int8_t*) calloc(756, sizeof(int8_t));
  	for (int i = 0; i < 756; i++) {
  	    input[i] = 0;
  	}

  	size = 755;
  #endif

  #ifdef NNOM_USING_STATIC_MEMORY
  	// when use static memory buffer, we need to set it before create
  	nnom_set_static_buf(static_buf, sizeof(static_buf));
  #endif

  	model = nnom_model_create();				// create NNoM model
  	pre = prediction_create(model, nnom_output_data, sizeof(nnom_output_data), 4); // mnist, 10 classes, get top-4
  	//model_set_callback(model, callback);

	// now takes label and data from the file and data
  	//PrintTimeStamp();
  	//HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
  	//HAL_Delay(100);
  	//HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);
  	//HAL_Delay(100);
  	uint64_t timer_val,timer_val1=0;
	for(size_t seek=0; seek < size;)
	{
		// labels
		uint8_t true_label[128];
		memcpy(true_label, input + seek, 128);
		seek += 128;
		// data
		for(int i=0; i < 128; i++)
		{
			TimerCount=0;
			timer_val = TimerCount;
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);		////	LSR

			if(seek >= size)
				break;
			memcpy(nnom_input_data, input + seek, sizeof(nnom_input_data));
			seek += sizeof(nnom_input_data);

			//nnom_predict(model, &label, &prob);				// this will work independently
			prediction_run(pre, true_label[i], &label, &prob);  // this provide more infor but requires prediction API

			timer_val1 = TimerCount;

			// save results
			f_printf(&SDFile, "%d,%f,%f\n", label, prob, prob);
			TxBufLen = sprintf(TxBuf, "ture : %ld, pre : %ld, pre_accuracy : %f\n", true_label[i], label, prob);
			HAL_UART_Transmit(&huart6, (uint8_t * )TxBuf, TxBufLen, 100);

			uart_buf_len = sprintf(uart_buf, "%u ms\r\n", (timer_val1-timer_val));
			HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, uart_buf_len, 100);

			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);	//// LSR
			HAL_Delay(100);
		}	//input_data_third

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


	//  HAL_Delay(5000);
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 100-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  htim7.Init.Prescaler = 100-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 50-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
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
  HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D8_Pin|LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|D1_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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

  /*Configure GPIO pin : D3_Pin */
  GPIO_InitStruct.Pin = D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D8_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D8_Pin|LCD_DISP_Pin;
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

  /*Configure GPIO pins : ARDUINO_D4_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : D1_Pin */
  GPIO_InitStruct.Pin = D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(D1_GPIO_Port, &GPIO_InitStruct);

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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
