#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "stm32f7xx_hal.h"
#include "arm_math.h"
#include "main.h"
#include "math.h"
#include "si5351.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery.h"
#include "stm32f7xx_hal_i2c.h"

// I2C for SI5351
I2C_HandleTypeDef hi2c1;

#define AUDIO_BUFFER_SIZE   512
#define AUDIO_FREQUENCY     AUDIO_FREQUENCY_48K
#define BLOCK_SIZE          (AUDIO_BUFFER_SIZE / 4)
#define NUM_TAPS            63
#define LPF_TAPS            64
#define SINE_TABLE_SIZE     512
#define NCO_TABLE_SIZE      436 // Samples for 11 kHz at 48 kHz
#define AUDIO_FS            48000.0f
#define NCO_FREQ            11000.0f
#define TONE_FREQ           1500.0f
#define PHASE_TO_INDEX      ((float)SINE_TABLE_SIZE / (2.0f * M_PI))

int16_t output_buffer[AUDIO_BUFFER_SIZE];
int16_t input_buffer[AUDIO_BUFFER_SIZE];

// Hilbert filter coefficients for I and Q (63 taps, ±45° phase shift)
static const float32_t hilbert_i_coeffs[NUM_TAPS] = { 0.0011617f, 0.0f,
		0.00138798f, 0.0f, 0.00195534f, 0.0f, 0.00292511f, 0.0f, 0.00436605f,
		0.0f, 0.00635959f, 0.0f, 0.00900884f, 0.0f, 0.01245464f, 0.0f,
		0.0169049f, 0.0f, 0.02269144f, 0.0f, 0.030389f, 0.0f, 0.04109288f, 0.0f,
		0.05717193f, 0.0f, 0.08482775f, 0.0f, 0.14688722f, 0.0f, 0.44909573f,
		0.70710678f, -0.44909573f, 0.0f, -0.14688722f, 0.0f, -0.08482775f, 0.0f,
		-0.05717193f, 0.0f, -0.04109288f, 0.0f, -0.030389f, 0.0f, -0.02269144f,
		0.0f, -0.0169049f, 0.0f, -0.01245464f, 0.0f, -0.00900884f, 0.0f,
		-0.00635959f, 0.0f, -0.00436605f, 0.0f, -0.00292511f, 0.0f,
		-0.00195534f, 0.0f, -0.00138798f, 0.0f, -0.0011617f };

static const float32_t hilbert_q_coeffs[NUM_TAPS] = { -0.0011617f, 0.0f,
		-0.00138798f, 0.0f, -0.00195534f, 0.0f, -0.00292511f, 0.0f,
		-0.00436605f, 0.0f, -0.00635959f, 0.0f, -0.00900884f, 0.0f,
		-0.01245464f, 0.0f, -0.0169049f, 0.0f, -0.02269144f, 0.0f, -0.030389f,
		0.0f, -0.04109288f, 0.0f, -0.05717193f, 0.0f, -0.08482775f, 0.0f,
		-0.14688722f, 0.0f, -0.44909573f, 0.70710678f, 0.44909573f, 0.0f,
		0.14688722f, 0.0f, 0.08482775f, 0.0f, 0.05717193f, 0.0f, 0.04109288f,
		0.0f, 0.030389f, 0.0f, 0.02269144f, 0.0f, 0.0169049f, 0.0f, 0.01245464f,
		0.0f, 0.00900884f, 0.0f, 0.00635959f, 0.0f, 0.00436605f, 0.0f,
		0.00292511f, 0.0f, 0.00195534f, 0.0f, 0.00138798f, 0.0f, 0.0011617f };

// LPF coefficients (64 taps, 3 kHz cutoff, Hamming window)
static const float32_t lpf_coeffs[LPF_TAPS] = { -0.000035f, -0.000071f,
		-0.000108f, -0.000146f, -0.000185f, -0.000223f, -0.000260f, -0.000294f,
		-0.000323f, -0.000346f, -0.000360f, -0.000364f, -0.000357f, -0.000337f,
		-0.000302f, -0.000253f, -0.000189f, -0.000110f, -0.000017f, 0.000089f,
		0.000216f, 0.000355f, 0.000506f, 0.000669f, 0.000842f, 0.001024f,
		0.001215f, 0.001412f, 0.001614f, 0.001819f, 0.002024f, 0.002227f,
		0.002426f, 0.002619f, 0.002803f, 0.002976f, 0.003135f, 0.003279f,
		0.003405f, 0.003512f, 0.003599f, 0.003664f, 0.003707f, 0.003726f,
		0.003721f, 0.003691f, 0.003637f, 0.003559f, 0.003457f, 0.003332f,
		0.003185f, 0.003017f, 0.002829f, 0.002623f, 0.002400f, 0.002162f,
		0.001911f, 0.001649f, 0.001378f, 0.001100f, 0.000818f, 0.000534f,
		0.000250f, -0.000033f };

arm_fir_instance_f32 fir_i, fir_q, fir_lpf;
float32_t fir_i_state[NUM_TAPS + BLOCK_SIZE - 1];
float32_t fir_q_state[NUM_TAPS + BLOCK_SIZE - 1];
float32_t fir_lpf_state[LPF_TAPS + BLOCK_SIZE - 1];

static float32_t sine_table[SINE_TABLE_SIZE];
float32_t input_rms = 0.0f;
float32_t output_rms = 0.0f;
uint32_t callback_count = 0;
uint8_t button_pressed = 0;
float32_t output_gain = 1.0f; // Adjustable gain
uint8_t demod_mode = 0; // 0=USB, 1=LSB

// Global persistent phase accumulator
static float32_t nco_phase = 0.0f;
static const float32_t nco_inc = 2.0f * M_PI * 11000.0f / AUDIO_FS;

// Common NCO generation function
static void generate_nco_block(float32_t *nco_buf, uint32_t size) {
	for (uint32_t i = 0; i < size; i++) {
		uint32_t idx = (uint32_t) (nco_phase * PHASE_TO_INDEX) % SINE_TABLE_SIZE;
		nco_buf[i] = sine_table[idx];
		nco_phase += nco_inc;
		if (nco_phase >= 2.0f * M_PI)
			nco_phase -= 2.0f * M_PI;
	}
}

void init_sine_table(void) {
	for (int i = 0; i < SINE_TABLE_SIZE; i++) {
		sine_table[i] = 0.5f * sinf(2.0f * M_PI * i / SINE_TABLE_SIZE);
	}
}
/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void SystemClock_Config(void);
static void AUDIO_InitApplication(void);
static void CPU_CACHE_Enable(void);
static void MX_I2C1_Init(void);

/* Audio Callbacks ----------------------------------------------------------*/
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	/* No-op */
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	/* No-op */
}

/* Audio Callbacks */
/* Audio Callbacks */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
    float32_t i_in[BLOCK_SIZE], q_in[BLOCK_SIZE], nco[BLOCK_SIZE], sum_out[BLOCK_SIZE], mixed_out[BLOCK_SIZE];

    // Deinterleave input buffer
    for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
        i_in[i] = (float32_t)input_buffer[2 * i] / 32768.0f;      // Left: I
        q_in[i] = (float32_t)input_buffer[2 * i + 1] / 32768.0f;  // Right: Q
    }

    // Compute input RMS
    arm_rms_f32(i_in, BLOCK_SIZE, &input_rms);

    // Sum I and Q
    arm_add_f32(i_in, q_in, sum_out, BLOCK_SIZE);

    // Generate 11 kHz NCO
    generate_nco_block(nco, BLOCK_SIZE);

    // Mix (multiply) I + Q with NCO
    arm_mult_f32(sum_out, nco, mixed_out, BLOCK_SIZE);

    // Compute output RMS
    arm_rms_f32(mixed_out, BLOCK_SIZE, &output_rms);

    // Output mixed signal to both channels
    for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
        int16_t sample = (int16_t)(mixed_out[i] * 32767.0f * output_gain);
        if (sample > 32767) sample = 32767;
        if (sample < -32768) sample = -32768;
        output_buffer[2 * i] = sample;     // Left: Mixed signal
        output_buffer[2 * i + 1] = sample; // Right: Mixed signal
    }

    callback_count++;
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void) {
    float32_t i_in[BLOCK_SIZE], q_in[BLOCK_SIZE], nco[BLOCK_SIZE], sum_out[BLOCK_SIZE], mixed_out[BLOCK_SIZE];

    // Deinterleave input buffer
    for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
        i_in[i] = (float32_t)input_buffer[AUDIO_BUFFER_SIZE / 2 + 2 * i] / 32768.0f;      // Left: I
        q_in[i] = (float32_t)input_buffer[AUDIO_BUFFER_SIZE / 2 + 2 * i + 1] / 32768.0f;  // Right: Q
    }

    // Compute input RMS
    arm_rms_f32(i_in, BLOCK_SIZE, &input_rms);

    // Sum I and Q
    arm_add_f32(i_in, q_in, sum_out, BLOCK_SIZE);

    // Generate 11 kHz NCO
    generate_nco_block(nco, BLOCK_SIZE);

    // Mix (multiply) I + Q with NCO
    arm_mult_f32(sum_out, nco, mixed_out, BLOCK_SIZE);

    // Compute output RMS
    arm_rms_f32(mixed_out, BLOCK_SIZE, &output_rms);

    // Output mixed signal to both channels
    for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
        int16_t sample = (int16_t)(mixed_out[i] * 32767.0f * output_gain);
        if (sample > 32767) sample = 32767;
        if (sample < -32768) sample = -32768;
        output_buffer[AUDIO_BUFFER_SIZE / 2 + 2 * i] = sample;     // Left: Mixed signal
        output_buffer[AUDIO_BUFFER_SIZE / 2 + 2 * i + 1] = sample; // Right: Mixed signal
    }

    callback_count++;
}
/* Main --------------------------------------------------------------------*/
int main(void) {
	MPU_Config();
	CPU_CACHE_Enable();
	HAL_Init();
	SystemClock_Config();
	BSP_LED_Init(LED1);
	BSP_LED_Toggle(LED1);
	HAL_Delay(1000);
	BSP_LED_Toggle(LED1);

	MX_I2C1_Init();
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

	// Initialize buffers
	for (uint32_t i = 0; i < AUDIO_BUFFER_SIZE; i++) {
		input_buffer[i] = 0;
		output_buffer[i] = 0;
	}
	memset(fir_i_state, 0, sizeof(fir_i_state));
	memset(fir_q_state, 0, sizeof(fir_q_state));
	memset(fir_lpf_state, 0, sizeof(fir_lpf_state));

	init_sine_table();
	arm_fir_init_f32(&fir_i, NUM_TAPS, (float32_t*) hilbert_i_coeffs,
			fir_i_state, BLOCK_SIZE);
	arm_fir_init_f32(&fir_q, NUM_TAPS, (float32_t*) hilbert_q_coeffs,
			fir_q_state, BLOCK_SIZE);
	arm_fir_init_f32(&fir_lpf, LPF_TAPS, (float32_t*) lpf_coeffs, fir_lpf_state,
			BLOCK_SIZE);

	AUDIO_InitApplication();
	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

	uint8_t last_button_state = BSP_PB_GetState(BUTTON_KEY);
	const int32_t correction = 978;
	si5351_Init(correction);
	si5351_SetupCLK1(28000000, SI5351_DRIVE_STRENGTH_4MA);
	si5351_EnableOutputs(1 << 1);

	char lcd_text[50];
	while (1) {
		uint8_t current_button_state = BSP_PB_GetState(BUTTON_KEY);
		if (!current_button_state && last_button_state) {
			button_pressed = !button_pressed;
			BSP_LED_Toggle(LED1);
		}
		last_button_state = current_button_state;

		snprintf(lcd_text, sizeof(lcd_text), "In: %.4f Out: %.4f CB: %lu",
				input_rms, output_rms, callback_count);
		LCD_LOG_SetFooter((uint8_t*) lcd_text);

		HAL_Delay(100);
	}
}

static void AUDIO_InitApplication(void) {
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);
	BSP_LCD_SelectLayer(1);
	BSP_LCD_DisplayOn();
	LCD_LOG_Init();
	LCD_LOG_SetHeader((uint8_t*) "SDR Project");

	BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1,
	OUTPUT_DEVICE_HEADPHONE, AUDIO_FREQUENCY,
	DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);

	BSP_AUDIO_IN_Record((uint16_t*) input_buffer, AUDIO_BUFFER_SIZE);
	BSP_AUDIO_OUT_Play((uint16_t*) output_buffer, AUDIO_BUFFER_SIZE * 2);
	BSP_AUDIO_OUT_SetVolume(70); // Set to 50 to avoid clipping
}

/* I2C1 Init for SI5351 -----------------------------------------------------*/
static void MX_I2C1_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x20404768;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		while (1)
			;
	}
}

/* System Clock Configuration -----------------------------------------------*/
static void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	if (ret != HAL_OK)
		while (1)
			;

	ret = HAL_PWREx_EnableOverDrive();
	if (ret != HAL_OK)
		while (1)
			;

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
	ret = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
	if (ret != HAL_OK)
		while (1)
			;

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
	if (ret != HAL_OK)
		while (1)
			;
}

/* LCD Clock Configuration --------------------------------------------------*/
void BSP_LCD_ClockConfig(LTDC_HandleTypeDef *hltdc, void *Params) {
	static RCC_PeriphCLKInitTypeDef periph_clk_init_struct;
	periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	periph_clk_init_struct.PLLSAI.PLLSAIN = 192;
	periph_clk_init_struct.PLLSAI.PLLSAIR = RK043FN48H_FREQUENCY_DIVIDER;
	periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
	HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
}

/* MPU Configuration --------------------------------------------------------*/
static void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct;
	HAL_MPU_Disable();

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

	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* Assert Failed Handler -----------------------------------------------------*/
#ifdef USE_FULL_ASSERT
   void assert_failed(uint8_t* file, uint32_t line) {
       while (1);
   }
   #endif

/* CPU Cache Enable ---------------------------------------------------------*/
static void CPU_CACHE_Enable(void) {
	SCB_EnableICache();
	SCB_EnableDCache();
}
