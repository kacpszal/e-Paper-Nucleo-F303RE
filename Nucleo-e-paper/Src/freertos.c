/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "time.h"
#include "gpio.h"
#define CMD_BUFFER_SIZE 128
#define LEN 3

//position of the time and date on e-paper
#define    posXTime32DotFont      350
#define    posYTime32DotFont      200
#define    posXDate32DotFont      335
#define    posYDate32DotFont      300
#define    posXTime64DotFont	  300
#define    posYTime64DotFont      200
#define    posXDate64DotFont	  260
#define    posYDate64DotFont	  300

/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
static unsigned char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t font = 0;
uint16_t posXTime = posXTime32DotFont;
uint16_t posYTime = posYTime32DotFont;
uint16_t posXDate = posXDate32DotFont;
uint16_t posYDate = posYDate32DotFont;
RTC_HandleTypeDef hrtc;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

// getting time and date

void RTC_SetTime(RTC_TimeTypeDef sTime);
void RTC_GetTime(RTC_TimeTypeDef *sTime);
void RTC_SetDate(RTC_DateTypeDef sDate);
void RTC_GetDate(RTC_DateTypeDef *sDate);

// end of getting time and date

unsigned char verifyParity(const void * ptr, int n);
void updateScreen();
void clearScreen();
void displayExampleText();
void displayString();
void set64DotFont();
void set32DotFont();
void EXTI15_10_IRQHandler_Config(void);

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	EXTI15_10_IRQHandler_Config();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
	RTC_TimeTypeDef rtc_time;
	RTC_DateTypeDef rtc_date;
	char timeBuffer[9];
	char dateBuffer[11];
	char hours[LEN];
	char minutes[LEN];
	char seconds[LEN];
	char dayString[LEN];
	char monthString[LEN];
	char yearString[5];
	RTC_SetTime(rtc_time);
	RTC_SetDate(rtc_date);

	rtc_time.Hours = 23;
	rtc_time.Minutes = 59;
	rtc_time.Seconds = 55;
	rtc_date.Date = 8;
	rtc_date.Month = 6;
	rtc_date.Year = 18;
	RTC_SetTime(rtc_time);
	RTC_SetDate(rtc_date);

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  while(1337)
  {
	  RTC_GetTime(&rtc_time);
	  RTC_GetDate(&rtc_date);

	  // rtc time
	  snprintf(hours, LEN, "%d", rtc_time.Hours);
	  snprintf(minutes, LEN, "%d", rtc_time.Minutes);
	  snprintf(seconds, LEN, "%d", rtc_time.Seconds);
	  if(rtc_time.Hours < 10) {
		  strcpy(timeBuffer, "0");
		  strcat(timeBuffer, hours);
	  } else {
		  strcpy(timeBuffer, hours);
	  }
	  strcat(timeBuffer, ":");
	  if(rtc_time.Minutes < 10) {
		  strcat(timeBuffer, "0");
	  }
	  strcat(timeBuffer, minutes);

	  strcat(timeBuffer, ":");
	  if(rtc_time.Seconds < 10) {
		  strcat(timeBuffer, "0");
	  }
	  strcat(timeBuffer, seconds);

	  // rtc_date
	  snprintf(dayString, LEN, "%d", rtc_date.Date);
	  snprintf(monthString, LEN, "%d", rtc_date.Month);
	  int yearInt = (int)rtc_date.Year + 2000;
	  snprintf(yearString, 5, "%d", yearInt);

	  strcpy(dateBuffer, yearString);
	  strcat(dateBuffer, "-");
	  if(rtc_date.Month < 10) {
		  strcat(dateBuffer, "0");
	  }
	  strcat(dateBuffer, monthString);
	  strcat(dateBuffer, "-");
	  if(rtc_date.Date < 10) {
		  strcat(dateBuffer, "0");
	  }
	  strcat(dateBuffer, dayString);

	  // displaying
	  displayString(timeBuffer, posXTime, posYTime);
	  displayString(dateBuffer, posXDate, posYDate);
	  updateScreen();
	  clearScreen();

	  osDelay(5000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */

/**
* @brief Configures EXTI lines 10 to 15 (connected to PC.13 pin) in interrupt mode
* @param None
* @retval None
*/
void EXTI15_10_IRQHandler_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOC clock */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Configure PC.13 pin as input floating */
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Enable and set EXTI lines 10 to 15 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

/**
* @brief EXTI line detection callbacks
* @param GPIO_Pin: Specifies the pins connected EXTI line
* @retval None
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		if(font == 0) {
			set64DotFont();
			font = 1;
			posXTime = posXTime64DotFont;
			posYTime = posYTime64DotFont;
			posXDate = posXDate64DotFont;
			posYDate = posYDate64DotFont;
		} else {
			set32DotFont();
			font = 0;
			posXTime = posXTime32DotFont;
			posYTime = posYTime32DotFont;
			posXDate = posXDate32DotFont;
			posYDate = posYDate32DotFont;
		}
	}
}

void updateScreen() {
	unsigned char tab[9] = {0xA5, 0x00, 0x09, 0x0A, 0xCC, 0x33, 0xC3, 0x3C, 0xA6};
	HAL_UART_Transmit(&huart5, tab, 9, 1);
}

void clearScreen() {
	unsigned char tab[9] = {0xA5, 0x00, 0x09, 0x2E, 0xCC, 0x33, 0xC3, 0x3C, 0x82};
	HAL_UART_Transmit(&huart5, tab, 9, 1);
}

void displayExampleText() {
	unsigned char tab[23] = {0xA5, 0x00, 0x17, 0x30, 0x00, 0x0A, 0x00, 0x0A, 0xC4,
			0xE3, 0xBA, 0xC3, 0x57, 0x6F, 0x72, 0x6C, 0x64, 0x00, 0xCC, 0x33, 0xC3, 0x3C, 0x9E};
	HAL_UART_Transmit(&huart5, tab, 23, 1);
}

void set64DotFont() {
	unsigned char tab[10] = {0xA5, 0x00, 0x0A, 0x1F, 0x03, 0xCC, 0x33, 0xC3, 0x3C, 0xB3};
	HAL_UART_Transmit(&huart5, tab, 10, 1);
}

void set32DotFont() {
	unsigned char tab[10] = {0xA5, 0x00, 0x0A, 0x1F, 0x01, 0xCC, 0x33, 0xC3, 0x3C};
	tab[9] = verifyParity(tab, 10);
	HAL_UART_Transmit(&huart5, tab, 10, 1);
}

unsigned char verifyParity(const void * ptr, int n)
{
	int i;
	unsigned char * p = (unsigned char *)ptr;
	unsigned char result;

	for(i = 0, result = 0; i < n; i++)
	{
		result ^= p[i];
	}

	return result;
}

void displayString(const void * p, int x, int y)
{
	int string_size;
	unsigned char * ptr = (unsigned char *)p;

	string_size = strlen((const char *)ptr);
	string_size += 14;

	cmdBuffer[0] = 0xA5;

	cmdBuffer[1] = (string_size >> 8) & 0xFF;
	cmdBuffer[2] = string_size & 0xFF;

	cmdBuffer[3] = 0x30;

	cmdBuffer[4] = (x >> 8) & 0xFF;
	cmdBuffer[5] = x & 0xFF;
	cmdBuffer[6] = (y >> 8) & 0xFF;
	cmdBuffer[7] = y & 0xFF;

	strcpy((char *)(&cmdBuffer[8]), (const char *)ptr);

	string_size -= 5;

	cmdBuffer[string_size] = 0xCC;
	cmdBuffer[string_size + 1] = 0x33;
	cmdBuffer[string_size + 2] = 0xC3;
	cmdBuffer[string_size + 3] = 0x3C;
	cmdBuffer[string_size + 4] = verifyParity(cmdBuffer, string_size + 4);

	HAL_UART_Transmit(&huart5, cmdBuffer, string_size + 5, 1);
}

void RTC_SetTime(RTC_TimeTypeDef sTime) {
	if(HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
}

void RTC_GetTime(RTC_TimeTypeDef *sTime) {
	RTC_DateTypeDef sDate;

	if(HAL_RTC_GetTime(&hrtc, sTime, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}

	if(HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
}

void RTC_SetDate(RTC_DateTypeDef sDate) {
	if(HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
}

void RTC_GetDate(RTC_DateTypeDef *sDate) {
	RTC_TimeTypeDef sTime;

		if(HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}

		if(HAL_RTC_GetDate(&hrtc, sDate, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
