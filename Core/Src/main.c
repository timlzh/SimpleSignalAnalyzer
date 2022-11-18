/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "ws2812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM_FREQ 72
#define TIM2_PRESCALER 21
#define SQUARE 0
#define SINE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Tx_str1[] = "Hello world!\r\n";
uint8_t Tx_str2[] = "LED Open!\r\n";
uint8_t Tx_str3[] = "LED Close!\r\n";
uint8_t str_buff[64];
uint8_t Rx_char = 0;
uint32_t capture_Buf[5] = {0};
uint8_t capture_Cnt = 0;
float high_time, low_time, total_time, duty_cycle, freq, rms, vp, adjVp, preFreq;
int wave_type = 0;
uint32_t adc_buf[1024];
volatile int adcBufFullFlag = 0;
uint16_t adc_value = 0, adc_volt = 0;
uint16_t ws2812_sine[] = {25, 18, 11, 4, 5, 14, 23, 32};
uint16_t ws2812_square[] = {25, 26, 27, 19, 11, 3, 4, 5, 6, 14, 22, 30, 31, 32};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_SendInfo(void) {
    sprintf((char *)str_buff, wave_type == SINE ? "Sine" : "Square");
    HAL_UART_Transmit(&huart1, str_buff, strlen((char *)str_buff), 100);
    sprintf((char *)str_buff, " Vp=%.1f rms=%.1f freq=%.1f duty=%.1f\n", adjVp,
            rms, freq, duty_cycle);
    HAL_UART_Transmit(&huart1, str_buff, strlen((char *)str_buff), 100);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        adcBufFullFlag = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        UART_SendInfo();
        HAL_UART_Receive_IT(&huart1, &Rx_char, 1);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        switch (capture_Cnt) {
            case 1:
                capture_Buf[0] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1,
                                              TIM_ICPOLARITY_FALLING);
                capture_Cnt++;
                break;
            case 2:
                capture_Buf[1] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1,
                                              TIM_ICPOLARITY_RISING);
                capture_Cnt++;
                break;
            case 3:
                capture_Buf[2] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
                HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
                capture_Cnt++;
                break;
        }
    }
}

float to_microsecond(uint32_t cnt, uint32_t PSC) {
    return (float)cnt * (float)(PSC + 1) / TIM_FREQ;
}

void cal_arr_and_psc(float SamplingRate, TIM_TypeDef *TIMx) {
    uint32_t totalDashed = TIM_FREQ * 1000000 / SamplingRate;
    uint32_t sqrtedTotal = (uint32_t)sqrt(totalDashed);

    for (uint32_t Prescaler = 2; Prescaler <= sqrtedTotal || Prescaler < 0xffff;
         Prescaler++) {
        if (totalDashed % Prescaler == 0 && Prescaler > 1 &&
            totalDashed / Prescaler > 1) {
            if (totalDashed / Prescaler <= 0xffff) {
                TIMx->ARR = totalDashed / Prescaler - 1;
                TIMx->PSC = Prescaler - 1;
            }
        }
    }
    totalDashed += 1;
    for (uint32_t Prescaler = 2; Prescaler <= sqrtedTotal || Prescaler < 0xffff;
         Prescaler++) {
        if (totalDashed % Prescaler == 0 && Prescaler > 1 &&
            totalDashed / Prescaler > 1) {
            if (totalDashed / Prescaler <= 0xffff) {
                TIMx->ARR = totalDashed / Prescaler - 1;
                TIMx->PSC = Prescaler - 1;
            }
        }
    }
}

int get_wave_type() {
    return (vp / sqrt(2) + vp) / 2 - rms <= 0 ? SQUARE : SINE;
}

float adj_vp() { return wave_type == SINE ? rms * 1.414 : rms; }

void ws2812_show_freq() {
    WS2812_clear();
    // if(freq < 5500){
    //     WS2812_display_RGB(50, 0, 0, 1);
    // }else if(freq < 10500){
    //     WS2812_display_RGB(0, 50, 0, 1);
    // }else{
    //     WS2812_display_RGB(0, 0, 50, 1);
    // }
    int R = 50, G = 0, B = 0;
    if (freq <= 3000) {
        R = 50, G = 0, B = 0;
    } else if (freq <= 16500) {
        R = 50 * (16500 - freq) / 13500;
        G = 50 * (freq - 3000) / 13500;
        B = 0;
    } else if (freq < 33000) {
        R = 0;
        G = 50 * (33000 - freq) / 16500;
        B = 50 * (freq - 16500) / 16500;
    } else {
        R = 0, G = 0, B = 50;
    }
    // WS2812_display_RGB(R, G, B, 17);
    if(wave_type == SINE){
        for (int i = 0; i < 8; i++) {
            WS2812_display_RGB(R, G, B, ws2812_sine[i]);
        }
    }else{
        for (int i = 0; i < 14; i++) {
            WS2812_display_RGB(R, G, B, ws2812_square[i]);
        }
    }
    
    WS2812_display();
}
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_UART_Receive_IT(&huart1, &Rx_char, 1);
    HAL_UART_Transmit(&huart1, Tx_str1, sizeof(Tx_str1), 10000);
    cal_arr_and_psc(500000, TIM3);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, adc_buf, 1024);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        switch (capture_Cnt) {
            case 0:
                capture_Cnt++;
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1,
                                              TIM_INPUTCHANNELPOLARITY_RISING);
                HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
                break;
            case 4:
                high_time = to_microsecond(capture_Buf[1] - capture_Buf[0], TIM2_PRESCALER);
                low_time = to_microsecond(capture_Buf[2] - capture_Buf[1], TIM2_PRESCALER);
                total_time = high_time + low_time;
                duty_cycle = (high_time * 100.0) / total_time;
                preFreq = freq;
                freq = 1000000.0 / total_time;
                capture_Cnt = 0;
                break;
        }

        if (adcBufFullFlag) {
            float volt, vMax = 0, vMin = 9999, vSum = 0;
            for (int i = 0; i < 1024; i++) {
                volt = (adc_buf[i] * 3300.0 / 4096.0 - 1836.0) * 1.6339869281;
                if (volt > vMax)
                    vMax = volt;
                if (volt < vMin)
                    vMin = volt;
                vSum += (volt / 1000.0) * (volt / 1000.0);
            }
            rms = sqrt(vSum / 1024.0) * 1000;
            // rms = rms * 1.0097888768426757 - 36.011380740475425;
            vp = (vMax - vMin) / 2;
            wave_type = get_wave_type();
            adjVp = adj_vp();
            UART_SendInfo();
            ws2812_show_freq();
            // ws2812_show_freq();
            adcBufFullFlag = 0;
        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
    while (1) {
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
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
