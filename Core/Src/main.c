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
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "8x8led.h"
#include "dac8830.h"
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
#define ABS(x) ((x) > 0 ? (x) : -(x))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t str_buff[64];
uint8_t Rx_char = 0;
uint32_t capture_Buf[5] = {0};
uint8_t capture_Cnt = 0;
float high_time, low_time, total_time, duty_cycle, freq, rms, vp, adjVp, preFreq, vdc;
int wave_type = 0;
uint32_t adc_buf[1000];
volatile int adcBufFullFlag = 0;
uint16_t adc_value = 0, adc_volt = 0;
uint16_t ws2812_sine[] = {24, 17, 10, 3, 4, 13, 22, 31};
uint16_t ws2812_square[] = {24, 25, 26, 18, 10, 2, 3, 4, 5, 13, 21, 29, 30, 31};
float brightness = 0;
int flag = 1;
float volt_buf[1024];
int points = 100;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

int fgetc(FILE *f) {
    uint8_t ch = 0;
    HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
    return ch;
}

void UART_SendInfo(void) {
    printf(wave_type == SINE ? "Sine" : "Square");
    printf(" Vp=%.1f rms=%.1f freq=%.1f duty=%.1f Vp=%.1f\n", adjVp, rms, freq, duty_cycle, vp);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        adcBufFullFlag = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // UART_SendInfo();
        if (Rx_char == '+') points += 100;
        if (Rx_char == '-') points -= 100;
        points = points < 10 ? 10 : points;
        points = points > 900 ? 900 : points;
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
    int R = 80, G = 0, B = 0;
    if (freq <= 1000) {
        R = 80, G = 0, B = 0;
    } else if (freq <= 5500) {
        R = 80 * (5500 - freq) / 4500;
        G = 80 * (freq - 1000) / 4500;
        B = 0;
    } else if (freq < 15500) {
        R = 0;
        G = 80 * (15500 - freq) / 10000;
        B = 80 * (freq - 5500) / 10000;
    } else {
        R = 0, G = 0, B = 80;
    }
    // WS2812_display_RGB(R, G, B, 17);
    brightness += freq * 0.002 * flag;
    if (brightness >= 100) {
        brightness = 100;
        flag = -1;
    } else if (brightness <= 0) {
        brightness = 0;
        flag = 1;
    }
    R = R * brightness / 100;
    G = G * brightness / 100;
    B = B * brightness / 100;
    if (wave_type == SINE) {
        for (int i = 0; i < 8; i++) {
            WS2812_display_RGB(R, G, B, ws2812_sine[i]);
        }
    } else {
        for (int i = 0; i < 14; i++) {
            WS2812_display_RGB(R, G, B, ws2812_square[i]);
        }
    }

    WS2812_display();
}

void led_show_freq() {
    int f = (int)(freq / 1000.0 + 0.5);
    int row = 1;
    int dat = 0;
    while (f && row <= 8) {
        if (f >= 8) {
            dat = 0xff;
            f -= 8;
        } else {
            dat = (1 << f) - 1;
            dat <<= 8 - f;
            f = 0;
        }
        Write_Max7219(row, dat);
        row++;
    }
    while (row <= 8) {
        Write_Max7219(row, 0);
        row++;
    }
}

void led_show_vpp() {
    int vpp = (int)(adjVp*2/50 + 0.5); 
    int row = 1;
    int dat = 0;
    while (vpp && row <= 8) {
        if (vpp >= 8) {
            dat = 0xff;
            vpp -= 8;
        } else {
            dat = (1 << vpp) - 1;
            dat <<= 8 - vpp;
            vpp = 0;
        }
        Write_Max7219(row, dat);
        row++;
    }
    while (row <= 8) {
        Write_Max7219(row, 0);
        row++;
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        ws2812_show_freq();
        led_show_vpp();
        dac8830_write_volt(vdc / 1000);
    }
}

void uart_HMI_show() {
    printf("\xff\xff\xff");
    if (wave_type == SINE)
        printf("t0.txt=\" Wave: Sine Freq: %.1fkHz \r\n Vp: %.0fmV Vdc: %.0fV\"", (freq / 1000.0), adjVp, vdc);
    else
        printf("t0.txt=\" Wave: Square Freq: %.1fkHz Vp: %.0fmV \r\n Duty: %.0f%% Vdc: %.0fmV\"", (freq / 1000.0), adjVp, duty_cycle, vdc);
    printf("\xff\xff\xff");
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
    MX_TIM4_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */
    Init_Max7219();
    dac8830_write_volt(3);
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_UART_Receive_IT(&huart1, &Rx_char, 1);
    cal_arr_and_psc(500000, TIM3);
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 1000);

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

                if (ABS(1000000.0 / total_time - preFreq) > 1000) {
                    HAL_TIM_Base_Stop(&htim3);
                    HAL_ADC_Stop_DMA(&hadc1);
                    if (freq >= 1000)
                        cal_arr_and_psc(10 * freq, TIM3);
                    else
                        cal_arr_and_psc(500000, TIM3);
                    HAL_TIM_Base_Start(&htim3);
                    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 1000);
                }
                preFreq = freq;
                freq = 1000000.0 / total_time;
                capture_Cnt = 0;
                break;
        }

        if (adcBufFullFlag) {
            float volt, vMax = 0, vMin = 9999, vSquareSum = 0, vSum = 0;

            for (int i = 0; i < 1000; i++) {
                volt_buf[i] = (adc_buf[i] * 3300.0 / 4096.0) * 2;
                if (volt_buf[i] > vMax) vMax = volt_buf[i];
                if (volt_buf[i] < vMin) vMin = volt_buf[i];
                vSum += volt_buf[i];
            }
            // vdc = vSum / 1000;
            // vdc += vdc / 1000 * 4; // 校准
            // if (wave_type == SQUARE && (duty_cycle < 48 || duty_cycle > 52)){
            vdc = vMax * 0.5 + vMin * 0.5;
            //}

            for (int i = 0; i < 1000; i++) {
                volt = volt_buf[i] - vdc;
                vSquareSum += (volt / 1000) * (volt / 1000);
            }

            uart_HMI_show();
            printf("\xff\xff\xff");
            printf("addt 1,0,50\xff\xff\xff");
            for (int i = 0; i < 70; i++) {
                printf("%c", (int)(volt_buf[i] * 255 / 5000));
            }

            rms = sqrt(vSquareSum / 1000.0) * 1000;
            // rms = rms * 1.0097888768426757 - 36.011380740475425;
            vp = (vMax - vMin) / 2;
            wave_type = get_wave_type();
            adjVp = adj_vp();
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
void SystemClock_Config(void) {
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
