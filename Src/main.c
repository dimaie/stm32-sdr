/**
 ******************************************************************************
 * @file    Audio/SDR/Src/main.c
 * @author  MCD Application Team
 * @brief   Audio playback and record main file.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "stm32f7xx_hal.h"
#define ARM_MATH_CM7
#include "arm_math.h"  // For Q15 operations
#include "main.h"
#include "math.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery.h"
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
AUDIO_ApplicationTypeDef appli_state = APPLICATION_IDLE;

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void SystemClock_Config(void);
static void AUDIO_InitApplication(void);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/

#define AUDIO_BUFFER_SIZE   480
#define AUDIO_FREQUENCY     48000
#define VOLUME              10

// Sweep parameters
float phase = 0.0f;
float freq_step = 2.0f;
int16_t audio_buffer[AUDIO_BUFFER_SIZE * 2]; // stereo
volatile uint8_t lpf_enabled = 1;

#define SINE_TABLE_SIZE 1024
q15_t sine_table[SINE_TABLE_SIZE];

// Initialize sine table (call once in main)
void init_sine_table(void) {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    	sine_table[i] = (q15_t)(32767.0f * sinf(2 * PI * i / SINE_TABLE_SIZE));
    }
}

// Phase accumulator
uint32_t phase_acc = 0;
uint32_t phase_inc = (uint32_t)((float)SINE_TABLE_SIZE * 200 / AUDIO_FREQUENCY); // initial freq 200 Hz

const q15_t alpha_q15 = 2621; // alpha = 0.08 in Q15 (0.08*32768≈2621)

int direction = 1;  // sweep direction
float freq = 200.0f; // current frequency

// LPF state: int32_t accumulator in Q15
int32_t lpf_out = 0;

void FillBuffer(int16_t *buf, int length) {
    for (int n = 0; n < length; n++) {
        // Top 10 bits of phase_acc for 1024-point sine table
        uint32_t index = phase_acc >> 22;
        int16_t raw_sample = sine_table[index];  // Q15

        int16_t output_sample;
        if (lpf_enabled) {
            // LPF in Q15
            int32_t delta = (int32_t)raw_sample - lpf_out; // delta in Q15
            lpf_out += (alpha_q15 * delta) >> 15;          // LPF accumulator in Q15
            output_sample = (int16_t)lpf_out;              // cast to int16_t
        } else {
            output_sample = raw_sample;
        }

        // Stereo output
        buf[2*n]   = output_sample;
        buf[2*n+1] = output_sample;

        // Advance phase accumulator
        phase_acc += phase_inc;
    }

    // Update sweep frequency
    freq += direction * freq_step;
    if (freq >= 5000.0f) { freq = 5000.0f; direction = -1; }
    if (freq <= 200.0f)  { freq = 200.0f;  direction = 1; }

    // Recalculate phase increment
    //phase_inc = (uint32_t)((freq / AUDIO_FREQUENCY) * 4294967296.0f);
    phase_inc = (uint32_t)((freq * (1ULL << 32)) / AUDIO_FREQUENCY);
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	// refill first half of buffer
	FillBuffer(&audio_buffer[0], AUDIO_BUFFER_SIZE / 2);
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	// refill second half of buffer
	FillBuffer(&audio_buffer[AUDIO_BUFFER_SIZE], AUDIO_BUFFER_SIZE / 2);
}
/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
	/* Configure the MPU attributes */
	MPU_Config();

	/* Enable the CPU Cache */
	CPU_CACHE_Enable();

	/* STM32F7xx HAL library initialization:
	 - Configure the Flash ART accelerator on ITCM interface
	 - Configure the Systick to generate an interrupt each 1 msec
	 - Set NVIC Group Priority to 4
	 - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();

	/* Configure the system clock to 200 MHz */
	SystemClock_Config();
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

	/* Init Audio Application */
	AUDIO_InitApplication();

	/* Init TS module */
	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

	init_sine_table();
	FillBuffer(audio_buffer, AUDIO_BUFFER_SIZE);

	// Start playing in circular mode
	BSP_AUDIO_OUT_Play((uint16_t*) audio_buffer, sizeof(audio_buffer));

	uint8_t last_button_state = BSP_PB_GetState(BUTTON_KEY);

	while (1) {
		uint8_t current_button_state = BSP_PB_GetState(BUTTON_KEY);

		// Detect falling edge (button pressed)
		if (!current_button_state && last_button_state) {
			lpf_enabled = !lpf_enabled;

			if (lpf_enabled)
				LCD_UsrLog("LPF: ON\n");
			else
				LCD_UsrLog("LPF: OFF\n");
		}

		last_button_state = current_button_state;

		HAL_Delay(10); // debounce
	}
}

/*******************************************************************************
 Static Function
 *******************************************************************************/

/**
 * @brief  Audio Application Init.
 * @param  None
 * @retval None
 */
static void AUDIO_InitApplication(void) {
	/* Configure LED1 */
	BSP_LED_Init(LED1);

	/* Initialize the LCD */
	BSP_LCD_Init();

	/* LCD Layer Initialization */
	BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);

	/* Select the LCD Layer */
	BSP_LCD_SelectLayer(1);

	/* Enable the display */
	BSP_LCD_DisplayOn();

	/* Init the LCD Log module */
	LCD_LOG_Init();

	LCD_LOG_SetHeader((uint8_t*) "SDR Project");
	BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, VOLUME, AUDIO_FREQUENCY);
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 200000000
 *            HCLK(Hz)                       = 200000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 25000000
 *            PLL_M                          = 25
 *            PLL_N                          = 432
 *            PLL_P                          = 2
 *            PLL_Q                          = 9
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 7
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 8;

	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	if (ret != HAL_OK) {
		while (1) {
			;
		}
	}

	/* Activate the OverDrive to reach the 200 MHz Frequency */
	ret = HAL_PWREx_EnableOverDrive();
	if (ret != HAL_OK) {
		while (1) {
			;
		}
	}

	/* Select PLLSAI output as USB clock source */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
	ret = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
	if (ret != HAL_OK) {
		while (1) {
			;
		}
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
	if (ret != HAL_OK) {
		while (1) {
			;
		}
	}
}

/**
 * @brief  Clock Config.
 * @param  hsai: might be required to set audio peripheral predivider if any.
 * @param  AudioFreq: Audio frequency used to play the audio stream.
 * @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
 *         Being __weak it can be overwritten by the application
 * @retval None
 */
void BSP_AUDIO_OUT_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t AudioFreq,
		void *Params) {
	RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;

	HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);

	/* Set the PLL configuration according to the audio frequency */
	if ((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K)
			|| (AudioFreq == AUDIO_FREQUENCY_44K)) {
		/* Configure PLLSAI prescalers */
		/* PLLI2S_VCO: VCO_429M
		 SAI_CLK(first level) = PLLI2S_VCO/PLLSAIQ = 429/2 = 214.5 Mhz
		 SAI_CLK_x = SAI_CLK(first level)/PLLI2SDivQ = 214.5/19 = 11.289 Mhz */
		RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
		RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
		RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
		RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 429;
		RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 2;
		RCC_ExCLKInitStruct.PLLI2SDivQ = 19;
		HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
	} else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K), AUDIO_FREQUENCY_96K */
	{
		/* SAI clock config
		 PLLI2S_VCO: VCO_344M
		 SAI_CLK(first level) = PLLI2S_VCO/PLLSAIQ = 344/7 = 49.142 Mhz
		 SAI_CLK_x = SAI_CLK(first level)/PLLI2SDivQ = 49.142/1 = 49.142 Mhz */
		RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
		RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
		RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
		RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 344;
		RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 7;
		RCC_ExCLKInitStruct.PLLI2SDivQ = 1;
		HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
	}
}

/**
 * @brief  Clock Config.
 * @param  hltdc: LTDC handle
 * @note   This API is called by BSP_LCD_Init()
 * @retval None
 */
void BSP_LCD_ClockConfig(LTDC_HandleTypeDef *hltdc, void *Params) {
	static RCC_PeriphCLKInitTypeDef periph_clk_init_struct;

	/* RK043FN48H LCD clock configuration */
	/* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
	/* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
	/* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/5 = 38.4 Mhz */
	/* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_4 = 38.4/4 = 9.6Mhz */
	periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	periph_clk_init_struct.PLLSAI.PLLSAIN = 192;
	periph_clk_init_struct.PLLSAI.PLLSAIR = RK043FN48H_FREQUENCY_DIVIDER;
	periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
	HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
}

/**
 * @brief  Configure the MPU attributes
 * @param  None
 * @retval None
 */
static void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct;

	/* Disable the MPU */
	HAL_MPU_Disable();

	/* Configure the MPU as Strongly ordered for not defined regions */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress = 0x00;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/* Configure the MPU attributes as WT for SDRAM */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress = 0xC0000000;
	MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER1;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.SubRegionDisable = 0x00;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/* Configure the MPU attributes FMC control registers */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress = 0xA0000000;
	MPU_InitStruct.Size = MPU_REGION_SIZE_8KB;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER2;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.SubRegionDisable = 0x0;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/* Enable the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
 * @brief  CPU L1-Cache enable.
 * @param  None
 * @retval None
 */
static void CPU_CACHE_Enable(void) {
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

