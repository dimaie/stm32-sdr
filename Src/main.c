#define DEBUG

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32f7xx_hal.h"
#include "arm_math.h"
#include "main.h"
#include "si5351.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32f7xx_hal_i2c.h"
#include "filters.h"

// I2C for SI5351
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

#define FFT_GRID_FREQUENCY_FACTOR   20
#define FFT_FREQUENCY_FACTOR   		25
#define AUDIO_BUFFER_SIZE   512
#define AUDIO_FREQUENCY     AUDIO_FREQUENCY_44K
#define BLOCK_SIZE          (AUDIO_BUFFER_SIZE / 4)
#define SINE_TABLE_SIZE     512
#define AUDIO_FS            44100.0f
#define NCO_FREQ            11000.0f
#define PHASE_TO_INDEX      ((float)SINE_TABLE_SIZE / (2.0f * M_PI))
#define FFT_SIZE            512
#define LCD_WIDTH           480
#define LCD_HEIGHT          272
#define SPECTRUM_HEIGHT     200
#define SPECTRUM_Y_OFFSET   20
#define MAX_MAGNITUDE       4.0f
#define FFT_SMOOTH_ALPHA    0.3f // Smoothing factor for EMA
#define ENABLE_FFT_SMOOTHING 1 // Set to 1 to enable EMA smoothing, 0 to disable

float32_t fft_magnitude[FFT_SIZE / 2];
float32_t fft_magnitude_smoothed[FFT_SIZE / 2] = { 0 }; // Smoothed magnitudes
int16_t output_buffer[AUDIO_BUFFER_SIZE];
int16_t input_buffer[AUDIO_BUFFER_SIZE];
float32_t fft_input[FFT_SIZE];
float32_t fft_input_ready[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
static float32_t window[FFT_SIZE];
static volatile uint32_t fft_buffer_index = 0;
static volatile uint8_t fft_buffer_full = 0;

arm_fir_instance_f32 fir_i, fir_q, fir_bpf, fir_lpf;
float32_t fir_i_state[HILBERT_TAPS + BLOCK_SIZE - 1];
float32_t fir_q_state[HILBERT_TAPS + BLOCK_SIZE - 1];
float32_t fir_bpf_state[BPF_TAPS + BLOCK_SIZE - 1];
float32_t fir_lpf_state[LPF_TAPS + BLOCK_SIZE - 1];

static float32_t sine_table[SINE_TABLE_SIZE];
float32_t input_rms = 0.0f;
float32_t output_rms = 0.0f;
float32_t i_filt_rms = 0.0f;
float32_t q_filt_rms = 0.0f;
float32_t sum_out_rms = 0.0f;
uint32_t callback_count = 0;
uint8_t button_pressed = 0;
float32_t output_gain = 1.0f;
uint8_t demod_mode = 0; // 0=USB

static float32_t nco_phase = 0.0f;
static const float32_t nco_inc = 2.0f * M_PI * NCO_FREQ / AUDIO_FS;

static arm_rfft_fast_instance_f32 fft_instance;

static void generate_nco_block(float32_t *nco_buf, uint32_t size) {
	for (uint32_t i = 0; i < size; i++) {
		uint32_t idx = (uint32_t) (nco_phase * PHASE_TO_INDEX) % SINE_TABLE_SIZE;
		nco_buf[i] = sine_table[idx];
		nco_phase += nco_inc;
		if (nco_phase >= 2.0f * M_PI)
			nco_phase -= 2.0f * M_PI;
	}
}

static void init_sine_table(void) {
	for (int i = 0; i < SINE_TABLE_SIZE; i++) {
		sine_table[i] = sinf(2.0f * M_PI * i / SINE_TABLE_SIZE);
	}
	// Generate Hann window
	for (int i = 0; i < FFT_SIZE; i++) {
		window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_SIZE - 1)));
	}
}

static void process_audio_block(int16_t *input_buf, int16_t *output_buf) {
	float32_t i_in[BLOCK_SIZE], q_in[BLOCK_SIZE], i_filt[BLOCK_SIZE],
			q_filt[BLOCK_SIZE];
	float32_t sum_out[BLOCK_SIZE], bpf_out[BLOCK_SIZE], mixed_out[BLOCK_SIZE],
			lpf_out[BLOCK_SIZE];
	float32_t nco[BLOCK_SIZE];

	// Deinterleave input buffer (I: left, Q: right)
	for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
		i_in[i] = (float32_t) input_buf[2 * i] / 32768.0f;
		q_in[i] = (float32_t) input_buf[2 * i + 1] / 32768.0f;
	}

	// Compute input RMS
	arm_rms_f32(i_in, BLOCK_SIZE, &input_rms);

	// Apply Hilbert filters
	arm_fir_f32(&fir_i, i_in, i_filt, BLOCK_SIZE);
	arm_fir_f32(&fir_q, q_in, q_filt, BLOCK_SIZE);

	// Subtract for USB image rejection (I - Q)
	arm_sub_f32(i_filt, q_filt, sum_out, BLOCK_SIZE);

	// Accumulate for FFT
	if (!fft_buffer_full && fft_buffer_index + BLOCK_SIZE <= FFT_SIZE) {
		memcpy(&fft_input[fft_buffer_index], sum_out,
				BLOCK_SIZE * sizeof(float32_t));
		fft_buffer_index += BLOCK_SIZE;
		if (fft_buffer_index >= FFT_SIZE) {
			fft_buffer_full = 1;
		}
	}
	arm_rms_f32(i_filt, BLOCK_SIZE, &i_filt_rms);
	arm_rms_f32(q_filt, BLOCK_SIZE, &q_filt_rms);
	arm_rms_f32(sum_out, BLOCK_SIZE, &sum_out_rms);

	// Apply USB BPF
	arm_fir_f32(&fir_bpf, sum_out, bpf_out, BLOCK_SIZE);

	// Generate 11 kHz NCO
	generate_nco_block(nco, BLOCK_SIZE);

	// Mix with NCO
	arm_mult_f32(bpf_out, nco, mixed_out, BLOCK_SIZE);

	// Apply LPF
	arm_fir_f32(&fir_lpf, mixed_out, lpf_out, BLOCK_SIZE);

	// Compute output RMS
	arm_rms_f32(lpf_out, BLOCK_SIZE, &output_rms);

	// Output to both channels
	for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
		int16_t sample = (int16_t) (lpf_out[i] * 32767.0f * output_gain);
		sample = (sample > 32767) ? 32767 : (sample < -32768) ? -32768 : sample;
		output_buf[2 * i] = sample;
		output_buf[2 * i + 1] = sample;
	}

	callback_count++;
}

static void update_spectrum_display(void) {
    static int16_t fft_update_counter = 0;
    static int16_t fft_grid_update_counter = 0;
    static int prev_bar_heights[FFT_SIZE / 2] = {0}; // Store previous bar heights

    if (!fft_buffer_full)
        return;

    fft_update_counter++;
    fft_grid_update_counter++; // Increment grid counter only when processing FFT buffer

    // Draw grid every FFT_GRID_FREQUENCY_FACTOR calls (~232 ms)
    if (fft_grid_update_counter % FFT_GRID_FREQUENCY_FACTOR == 0) {
        BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
        for (int i = 1; i < 5; i++) {
            BSP_LCD_DrawHLine(0, SPECTRUM_Y_OFFSET + (SPECTRUM_HEIGHT / 5 * i), LCD_WIDTH);
        }
        for (int i = 1; i < 5; i++) {
            int freq = i * (AUDIO_FS / 2) / 5;
            int x = (int)((float)freq / (AUDIO_FS / 2) * LCD_WIDTH);
            BSP_LCD_DrawVLine(x, SPECTRUM_Y_OFFSET, SPECTRUM_HEIGHT);
        }
        fft_grid_update_counter = 0; // Reset grid counter to prevent overflow
    }

    // Update spectrum only every FFT_FREQUENCY_FACTOR calls (~290 ms)
    if (fft_update_counter % FFT_FREQUENCY_FACTOR != 0) {
        return;
    }

    // Copy FFT input and reset flag
    memcpy(fft_input_ready, fft_input, FFT_SIZE * sizeof(float32_t));
    fft_buffer_full = 0;
    fft_buffer_index = 0;

    // Apply Hann window
    for (int i = 0; i < FFT_SIZE; i++) {
        fft_input_ready[i] *= window[i];
    }

    // Compute FFT
    arm_rfft_fast_f32(&fft_instance, fft_input_ready, fft_output, 0);

    // Compute magnitude
    arm_cmplx_mag_f32(fft_output, fft_magnitude, FFT_SIZE / 2);

#if ENABLE_FFT_SMOOTHING
    // Apply exponential moving average (EMA) smoothing
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        fft_magnitude_smoothed[i] = FFT_SMOOTH_ALPHA * fft_magnitude[i] +
                                   (1.0f - FFT_SMOOTH_ALPHA) * fft_magnitude_smoothed[i];
    }
#endif

    // Clear previous bars
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float freq = i * (AUDIO_FS / FFT_SIZE);
        if (freq > 20000.0f) continue; // Clip at 20 kHz
        int x = (int)((float)i / (FFT_SIZE / 2) * LCD_WIDTH);
        int bar_width = (LCD_WIDTH + (FFT_SIZE / 2 - 1)) / (FFT_SIZE / 2);
        int prev_height = prev_bar_heights[i];
        if (prev_height > 0) {
            int y = SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT - prev_height;
            BSP_LCD_FillRect(x, y, bar_width, prev_height);
        }
    }

    // Draw new bars and store heights
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float freq = i * (AUDIO_FS / FFT_SIZE);
        if (freq > 20000.0f) {
            prev_bar_heights[i] = 0; // No bar drawn, reset height
            continue;
        }
        int x = (int)((float)i / (FFT_SIZE / 2) * LCD_WIDTH);
        int bar_width = (LCD_WIDTH + (FFT_SIZE / 2 - 1)) / (FFT_SIZE / 2);
#if ENABLE_FFT_SMOOTHING
        float32_t scaled_mag = fft_magnitude_smoothed[i] / MAX_MAGNITUDE;
#else
        float32_t scaled_mag = fft_magnitude[i] / MAX_MAGNITUDE;
#endif
        scaled_mag = (scaled_mag > 1.0f) ? 1.0f : scaled_mag;
        int bar_height = (int)(scaled_mag * SPECTRUM_HEIGHT);
        int y = SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT - bar_height;
        BSP_LCD_FillRect(x, y, bar_width, bar_height);
        prev_bar_heights[i] = bar_height; // Store new height
    }

#ifdef DEBUG
    // Send debug info to UART terminal
    char debug_text[64];
    snprintf(debug_text, sizeof(debug_text),
             "In:%.2f I:%.2f Q:%.2f Sum:%.2f Out:%.2f CB:%lu\r\n",
             input_rms, i_filt_rms, q_filt_rms, sum_out_rms, output_rms, callback_count);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_text, strlen(debug_text), HAL_MAX_DELAY);
#endif
}
/* Private function prototypes */
static void MPU_Config(void);
static void SystemClock_Config(void);
static void AUDIO_InitApplication(void);
static void CPU_CACHE_Enable(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
}
void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
	process_audio_block(&input_buffer[0], &output_buffer[0]);
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void) {
	process_audio_block(&input_buffer[AUDIO_BUFFER_SIZE / 2],
			&output_buffer[AUDIO_BUFFER_SIZE / 2]);
}

int main(void) {
	MPU_Config();
	CPU_CACHE_Enable();
	HAL_Init();
	SystemClock_Config();
	MX_USART1_UART_Init();
	BSP_LED_Init(LED1);
	BSP_LED_Toggle(LED1);
	HAL_Delay(1000);
	BSP_LED_Toggle(LED1);

	MX_I2C1_Init();
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

	// Initialize buffers
	memset(input_buffer, 0, sizeof(input_buffer));
	memset(output_buffer, 0, sizeof(output_buffer));
	memset(fir_i_state, 0, sizeof(fir_i_state));
	memset(fir_q_state, 0, sizeof(fir_q_state));
	memset(fir_bpf_state, 0, sizeof(fir_bpf_state));
	memset(fir_lpf_state, 0, sizeof(fir_lpf_state));
	memset(fft_input, 0, sizeof(fft_input));
	memset(fft_input_ready, 0, sizeof(fft_input_ready));

	init_sine_table();
	arm_fir_init_f32(&fir_i, HILBERT_TAPS, (float32_t*) hilbert_i_coeffs,
			fir_i_state, BLOCK_SIZE);
	arm_fir_init_f32(&fir_q, HILBERT_TAPS, (float32_t*) hilbert_q_coeffs,
			fir_q_state, BLOCK_SIZE);
	arm_fir_init_f32(&fir_bpf, BPF_TAPS, (float32_t*) bpf_coeffs, fir_bpf_state,
			BLOCK_SIZE);
	arm_fir_init_f32(&fir_lpf, LPF_TAPS, (float32_t*) lpf_coeffs, fir_lpf_state,
			BLOCK_SIZE);

	// Initialize FFT
	arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

	AUDIO_InitApplication();
	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

	// Initial LCD setup
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0, 0, (uint8_t*) "RF Spectrum (0-20 kHz)",
			CENTER_MODE);

	uint8_t last_button_state = BSP_PB_GetState(BUTTON_KEY);
	const int32_t correction = 978;
	si5351_Init(correction);
	si5351_SetupCLK1(28000000, SI5351_DRIVE_STRENGTH_4MA);
	si5351_EnableOutputs(1 << 1);

	while (1) {
		uint8_t current_button_state = BSP_PB_GetState(BUTTON_KEY);
		if (!current_button_state && last_button_state) {
			button_pressed = !button_pressed;
			BSP_LED_Toggle(LED1);
		}
		last_button_state = current_button_state;

		if (fft_buffer_full) {
			update_spectrum_display();
		}

		HAL_Delay(2);
	}
}

static void MX_USART1_UART_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // USART1 TX: PA9, RX: PA10 (STM32F746G-DISCO)
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        while (1); // Error handling
    }
}

static void AUDIO_InitApplication(void) {
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);
	BSP_LCD_SelectLayer(1);
	BSP_LCD_DisplayOn();
	LCD_LOG_Init();
	LCD_LOG_SetHeader((uint8_t*) "SDR Project");

	BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1, OUTPUT_DEVICE_HEADPHONE,
	AUDIO_FREQUENCY, DEFAULT_AUDIO_IN_BIT_RESOLUTION,
			DEFAULT_AUDIO_IN_CHANNEL_NBR);
	BSP_AUDIO_IN_Record((uint16_t*) input_buffer, AUDIO_BUFFER_SIZE);
	BSP_AUDIO_OUT_Play((uint16_t*) output_buffer, AUDIO_BUFFER_SIZE * 2);
	BSP_AUDIO_OUT_SetVolume(70);
}

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

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48
			| RCC_PERIPHCLK_SAI2;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
	ret = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
	if (ret != HAL_OK)
		while (1)
			;

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
	RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
	if (ret != HAL_OK)
		while (1)
			;
}

void BSP_LCD_ClockConfig(LTDC_HandleTypeDef *hltdc, void *Params) {
	static RCC_PeriphCLKInitTypeDef periph_clk_init_struct;
	periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	periph_clk_init_struct.PLLSAI.PLLSAIN = 192;
	periph_clk_init_struct.PLLSAI.PLLSAIR = RK043FN48H_FREQUENCY_DIVIDER;
	periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
	HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
}

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

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
    while (1);
}
#endif

static void CPU_CACHE_Enable(void) {
	SCB_EnableICache();
	SCB_EnableDCache();
}
