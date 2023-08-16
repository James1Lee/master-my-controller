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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include <nnom.h>
#include <weights.h>
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
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t* load(const char* file, size_t * size)
{
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
}

nnom_status_t callback(nnom_model_t* m, nnom_layer_t* layer)
{
	float scale = 1 << (layer->out->tensor->q_dec[0]);
	printf("\nOutput of Layer %s", default_layer_names[layer->type]);
	for (int i = 0; i < tensor_size(layer->out->tensor); i++)
	{
		if (i % 16 == 0)
			printf("\n");
		//printf("%f ", (float)((int8_t*)layer->out->tensor->p_data)[i] / scale);
	}
	printf("\n");
	return NN_SUCCESS;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char uart_buf[50];
	int uart_buf_len=0;
	uint16_t timer_val,timer_val1=0;

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uart_buf_len = sprintf(uart_buf, "TIMER TEST\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);
  //uint8_t str[] = "Hello, World!\n\r";
  HAL_TIM_Base_Start(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //while (1)
  //{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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



		  	  		model = nnom_model_create();				// create NNoM model
		  	  	  	pre = prediction_create(model, nnom_output_data, sizeof(nnom_output_data), 4); // mnist, 10 classes, get top-4
		  	  	  	//model_set_callback(model, callback);

		  	  		// now takes label and data from the file and data
		  	  	  	for(uint64_t seek=0; seek < size;)
		  	  	  	{
		  	  	  		// labels

		  	  	  		uint8_t true_label[128];
		  	  	  		memcpy(true_label, input_data_third + seek, 128);
		  	  	  		seek += 128;
		  	  	  		// data
		  	  	  		//SysTickCnt_start = SysTickCnt;
		  	  	  		for(uint64_t i=0; i < 10; i++)
		  	  	  		{

		  	  	  			//timer_start = HAL_TIM_Base_Start(&htim6);
		  	  	  			if(seek >= size)
		  	  	  				break;

		  	  	  			memcpy(nnom_input_data, input_data_third + seek, sizeof(nnom_input_data));
		  	  	  			seek += sizeof(nnom_input_data);

		  	  	  			//nnom_predict(model, &label, &prob);				// this will work independently
		  	  	  			prediction_run(pre, true_label[i], &label, &prob);  // this provide more infor but requires prediction API

		  	  	  			// save results
		  	  	  			//fprintf(fp, "%d,%f\n", label, prob);
		  	  	  			//timer_end = HAL_GetTick();
		  	  	  			TxBufferlen1 = sprintf(buf1,"Pre_Label : %d, Pre_accuracy : %f \n",label,prob);
		  	  	  			HAL_UART_Transmit(&huart1, buf1, TxBufferlen1, 100);
		  	  	  		}
		  	  	  		TxBufferlen = sprintf(buf,"Processing %d%%\n", (uint32_t)(seek * 100 / size));
		  	  	  		HAL_UART_Transmit(&huart1, buf, TxBufferlen, 1000);
		  	  	  	}
		  	  	  	prediction_matrix(pre);
		  	  	  	//model_stat(pre);
	  }
  //}
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 80-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65536-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
