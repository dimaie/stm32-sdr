#include "dsp.h"
#include "si5351.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stm32f7xx_hal.h"
#include  "lcd_log.h"
#include <string.h>

// I2C for SI5351
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

uint8_t button_pressed = 0;

// Audio callback functions
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
    DSP_ProcessAudioBlock(&dsp, &dsp.input_buffer[0], &dsp.output_buffer[0]);
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void) {
    DSP_ProcessAudioBlock(&dsp, &dsp.input_buffer[AUDIO_BUFFER_SIZE / 2],
                         &dsp.output_buffer[AUDIO_BUFFER_SIZE / 2]);
}

// Private function prototypes
static void MPU_Config(void);
static void SystemClock_Config(void);
static void AUDIO_InitApplication(void);
static void CPU_CACHE_Enable(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

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

    // Initialize DSP context
    DSP_Init(&dsp);

    AUDIO_InitApplication();
    BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

    // Initial LCD setup
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(0, 0, (uint8_t*)"RF Spectrum (0-20 kHz)", CENTER_MODE);

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

        if (dsp.fft_buffer_full) {
            update_spectrum_display(&dsp);
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
    BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1, OUTPUT_DEVICE_HEADPHONE,
                          AUDIO_FREQUENCY_44K, DEFAULT_AUDIO_IN_BIT_RESOLUTION,
                          DEFAULT_AUDIO_IN_CHANNEL_NBR);
    BSP_AUDIO_IN_Record((uint16_t*)dsp.input_buffer, AUDIO_BUFFER_SIZE);
    BSP_AUDIO_OUT_Play((uint16_t*)dsp.output_buffer, AUDIO_BUFFER_SIZE * 2);
    BSP_AUDIO_OUT_SetVolume(70);
}

static void MX_I2C1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
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
        while (1);
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
        while (1);

    ret = HAL_PWREx_EnableOverDrive();
    if (ret != HAL_OK)
        while (1);

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48 | RCC_PERIPHCLK_SAI2;
    PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
    PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
    PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4;
    PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
    ret = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    if (ret != HAL_OK)
        while (1);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
    if (ret != HAL_OK)
        while (1);
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
