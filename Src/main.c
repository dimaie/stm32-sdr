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

// I2C for SI5351
I2C_HandleTypeDef hi2c1;

#define AUDIO_BUFFER_SIZE   512
#define AUDIO_FREQUENCY     AUDIO_FREQUENCY_44K  // 44.1 kHz to match Python
#define BLOCK_SIZE          (AUDIO_BUFFER_SIZE / 4)  // 128 samples
#define HILBERT_TAPS        100  // Matches Python (100 elements)
#define BPF_TAPS            100  // Matches Python (100 elements)
#define LPF_TAPS            54   // Matches Python (54 elements)
#define SINE_TABLE_SIZE     512
#define AUDIO_FS            44100.0f
#define NCO_FREQ            11000.0f  // 11 kHz LO
#define PHASE_TO_INDEX      ((float)SINE_TABLE_SIZE / (2.0f * M_PI))
#define FFT_SIZE            512  // FFT size for spectrum display
#define LCD_WIDTH           480
#define LCD_HEIGHT          272
#define SPECTRUM_HEIGHT     200  // Height of spectrum plot area
#define SPECTRUM_Y_OFFSET   20   // Y offset for spectrum plot

int16_t output_buffer[AUDIO_BUFFER_SIZE];
int16_t input_buffer[AUDIO_BUFFER_SIZE];

// Hilbert filter coefficients for +45° (100 taps, from Python)
static const float32_t hilbert_i_coeffs[HILBERT_TAPS] = {
    4.288698225779810E-18f, 0.000866962256721964f, 6.694578973884190E-18f, 0.000723768779644489f,
    6.251950335026000E-18f, 0.000491948708636800f, 9.329793308652210E-18f, 0.000162753915054888f,
    1.206877891916540E-18f, -0.000267138492850143f, 3.686242120344090E-18f, -0.000794106419837781f,
    -3.350841016622350E-18f, -0.001406158205346857f, 5.086725057380440E-18f, -0.002081653046072215f,
    5.254639122859110E-18f, -0.002788189076768316f, 1.997158538844780E-15f, -0.003481638971364546f,
    1.521786931189910E-15f, -0.004105248229979659f, 1.999269938576540E-15f, -0.004588612972905914f,
    1.642012534044480E-15f, -0.004846199063150177f, 2.321256832370990E-15f, -0.004774806025839937f,
    1.472463356633780E-15f, -0.004248919032053459f, 3.256022898511980E-15f, -0.003112012423934955f,
    2.885150851939090E-15f, -0.001160058329106086f, 4.963134919021790E-15f, 0.001890534040148042f,
    7.256657980479030E-15f, 0.006468126105579311f, 9.877273082232760E-15f, 0.013288374303952424f,
    9.998257606955500E-15f, 0.023701141617936898f, 1.304841388605350E-14f, 0.040681358385972734f,
    1.316584420291990E-14f, 0.072465913204806887f, 1.832157501742890E-14f, 0.153651468095950122f,
    4.504853770616490E-14f, 0.872643185009882805f, -1.249775599534200E-13f, -0.320482045163527962f,
    -2.183135851203050E-14f, -0.147353662629853055f, -1.325350771085950E-14f, -0.098220446790738603f,
    -9.869993202619540E-15f, -0.073799181987711918f, -4.594216166446000E-15f, -0.058469211844324492f,
    -3.304347777777260E-15f, -0.047522100992294508f, 1.045764716612020E-16f, -0.039075967510059088f,
    1.357742476364510E-15f, -0.032247860767888339f, 1.609834995034210E-15f, -0.026578009579594275f,
    3.029110018978720E-18f, -0.021807474489618426f, 1.269751717122650E-15f, -0.017779413498920069f,
    -2.455156751402480E-16f, -0.014390646510711955f, 4.318593097590070E-16f, -0.011566204186358037f,
    -2.073528334337070E-17f, -0.009245418095997261f, 4.939755466124440E-16f, -0.007374279468676965f,
    2.513569241845640E-16f, -0.005901425606391064f, 1.464223979968000E-15f, -0.004776322329186264f,
    6.971003972531060E-16f, -0.003948802940774603f, 8.141476939065410E-16f, -0.003369432187206015f,
    -3.161136311572910E-16f, -0.002990334964793745f, -9.052589606482270E-17f, -0.002766233025766330f,
    -6.964043762789310E-16f, -0.002655502116741105f, -3.256402058849020E-17f, -0.002621113453607914f,
    -8.694556986426840E-17f, -0.002631365320034318f, 8.518103595547740E-17f, -0.002660346625915069f
};

// Hilbert filter coefficients for -45° (100 taps, from Python)
static const float32_t hilbert_q_coeffs[HILBERT_TAPS] = {
    -0.002660346625914681f, -1.425615451960800E-16f, -0.002631365320034742f, 7.239460181599930E-17f,
    -0.002621113453608740f, 3.656625573314080E-16f, -0.002655502116741606f, -2.681406471145040E-16f,
    -0.002766233025766440f, 5.229276890266240E-16f, -0.002990334964794087f, -6.070059499422990E-16f,
    -0.003369432187205082f, -7.323460201465080E-17f, -0.003948802940773903f, -4.764056590717670E-16f,
    -0.004776322329185343f, -3.129479342217880E-16f, -0.005901425606389074f, -5.992210655172320E-16f,
    -0.007374279468676095f, -2.575055438396270E-16f, -0.009245418095996334f, -1.432636145077280E-15f,
    -0.011566204186357842f, 1.855776728505920E-16f, -0.014390646510711451f, -1.223169894210750E-15f,
    -0.017779413498919743f, -0.021807474489616407f, -0.026578009579593605f, -0.032247860767886542f,
    -0.039075967510061899f, 2.312619114749650E-15f, -0.047522100992295140f, 5.508313753159310E-16f,
    -0.058469211844333152f, 3.653449213341470E-15f, -0.073799181987724130f, 5.543566341328550E-15f,
    -0.098220446790761515f, 7.591005706981040E-15f, -0.147353662629886223f, 2.175090500727820E-14f,
    -0.320482045163644980f, 1.294663938604830E-13f, 0.872643185009816635f, -4.886088790795220E-14f,
    0.153651468095996002f, -2.334181949236510E-14f, 0.072465913204831797f, -1.273860359379920E-14f,
    0.040681358385996375f, -9.967403549770980E-15f, 0.023701141617955505f, -8.913611311437650E-15f,
    0.013288374303971109f, -5.120441418271150E-15f, 0.006468126105590283f, -6.713751021471820E-15f,
    0.001890534040160306f, -4.214988052861760E-15f, -0.001160058329098612f, -3.133467475213760E-15f,
    -0.003112012423928077f, -3.051927453298240E-15f, -0.004248919032049460f, -2.728321242007810E-15f,
    -0.004774806025835775f, -1.212682271632930E-15f, -0.004846199063146594f, -2.473499615604520E-15f,
    -0.004588612972902101f, -7.056223879923200E-16f, -0.004105248229976594f, -1.187392338537970E-15f,
    -0.003481638971361474f, -7.417226001046070E-16f, -0.002788189076767050f, -8.039975667293950E-16f,
    -0.002081653046071019f, -4.058558361122810E-16f, -0.001406158205345744f, -7.324771536664670E-16f,
    -0.000794106419837717f, 2.037380889287510E-16f, -0.000267138492849360f, -7.918767863321210E-16f,
    0.000162753915055431f, -9.916131906296190E-17f, 0.000491948708637530f, -6.080720937020530E-16f,
    0.000723768779645844f, -5.864325834209240E-16f, 0.000866962256723557f, -3.337536727527920E-16f
};

// USB BPF coefficients (100 taps, centered at 12.5 kHz, from Python)
static const float32_t bpf_coeffs[BPF_TAPS] = {
    0.000077220197574123f, -1.694364742523110E-6f, -0.000247376363569229f, 0.000172349071145931f,
    0.000442532978562303f, -0.000587921806885401f, -0.000490796464995067f, 0.001245366873040035f,
    0.000130200340800485f, -0.001862166817785674f, 0.000703051894342210f, 0.002007465586182165f,
    -0.001685724919682768f, -0.001407920138837247f, 0.002116180748678689f, 0.000372547212250720f,
    -0.001360627435518710f, 0.000104411160061938f, -0.000488583862643263f, 0.001171803660774852f,
    0.002230271567546280f, -0.004622738899125535f, -0.001988597303309669f, 0.009091087568736349f,
    -0.001507388998028951f, -0.012022955491491360f, 0.007518507241432481f, 0.010969367705405847f,
    -0.012943254772900497f, -0.005724754564656228f, 0.013800759554407906f, -0.000413209911619933f,
    -0.008117888769062576f, 0.001659860856017020f, -0.001521029232922352f, 0.006868472137034775f,
    0.007858709633466776f, -0.025100680515366711f, -0.002212548908150258f, 0.046023064611360000f,
    -0.020182170398250168f, -0.057755470091797719f, 0.055819871330333046f, 0.049337804826868066f,
    -0.092667374657917745f, -0.017292461343810531f, 0.114978094867627500f, -0.030993743432389236f,
    -0.111032169995744168f, 0.079554595102602668f, 0.079554595102602668f, -0.111032169995744168f,
    -0.030993743432389236f, 0.114978094867627500f, -0.017292461343810531f, -0.092667374657917745f,
    0.049337804826868066f, 0.055819871330333046f, -0.057755470091797719f, -0.020182170398250168f,
    0.046023064611360000f, -0.002212548908150258f, -0.025100680515366711f, 0.007858709633466776f,
    0.006868472137034775f, -0.001521029232922352f, 0.001659860856017020f, -0.008117888769062576f,
    -0.000413209911619933f, 0.013800759554407906f, -0.005724754564656228f, -0.012943254772900497f,
    0.010969367705405847f, 0.007518507241432481f, -0.012022955491491360f, -0.001507388998028951f,
    0.009091087568736349f, -0.001988597303309669f, -0.004622738899125535f, 0.002230271567546280f,
    0.001171803660774852f, -0.000488583862643263f, 0.000104411160061938f, -0.001360627435518710f,
    0.000372547212250720f, 0.002116180748678689f, -0.001407920138837247f, -0.001685724919682768f,
    0.002007465586182165f, 0.000703051894342210f, -0.001862166817785674f, 0.000130200340800485f,
    0.001245366873040035f, -0.000490796464995067f, -0.000587921806885401f, 0.000442532978562303f,
    0.000172349071145931f, -0.000247376363569229f, -1.694364742523110E-6f, 0.000077220197574123f
};

// LPF coefficients (54 taps, 0-2700 Hz, from Python)
static const float32_t lpf_coeffs[LPF_TAPS] = {
    -0.007202148437500f, -0.017944335937500f, -0.011901855468750f, -0.017517089843750f,
    -0.013946533203125f, -0.010192871093750f, -0.002105712890625f, 0.007202148437500f,
    0.016906738281250f, 0.024322509765625f, 0.027465820312500f, 0.024597167968750f,
    0.015075683593750f, -0.000366210937500f, -0.019317626953125f, -0.038238525390625f,
    -0.052551269531250f, -0.057739257812500f, -0.049926757812500f, -0.026977539062500f,
    0.011047363281250f, 0.061584472656250f, 0.119720458984375f, 0.178955078125000f,
    0.231933593750000f, 0.271911621093750f, 0.293151855468750f, 0.293151855468750f,
    0.271911621093750f, 0.231933593750000f, 0.178955078125000f, 0.119720458984375f,
    0.061584472656250f, 0.011047363281250f, -0.026977539062500f, -0.049926757812500f,
    -0.057739257812500f, -0.052551269531250f, -0.038238525390625f, -0.019317626953125f,
    -0.000366210937500f, 0.015075683593750f, 0.024597167968750f, 0.027465820312500f,
    0.024322509765625f, 0.016906738281250f, 0.007202148437500f, -0.002105712890625f,
    -0.010192871093750f, -0.013946533203125f, -0.017517089843750f, -0.011901855468750f,
    -0.017944335937500f, -0.007202148437500f
};

arm_fir_instance_f32 fir_i, fir_q, fir_bpf, fir_lpf;
float32_t fir_i_state[HILBERT_TAPS + BLOCK_SIZE - 1];
float32_t fir_q_state[HILBERT_TAPS + BLOCK_SIZE - 1];
float32_t fir_bpf_state[BPF_TAPS + BLOCK_SIZE - 1];
float32_t fir_lpf_state[LPF_TAPS + BLOCK_SIZE - 1];

static float32_t sine_table[SINE_TABLE_SIZE];
float32_t input_rms = 0.0f;
float32_t output_rms = 0.0f;
uint32_t callback_count = 0;
uint8_t button_pressed = 0;
float32_t output_gain = 1.0f;
uint8_t demod_mode = 0; // 0=USB

static float32_t nco_phase = 0.0f;
static const float32_t nco_inc = 2.0f * M_PI * NCO_FREQ / AUDIO_FS;

static arm_rfft_fast_instance_f32 fft_instance;
static float32_t fft_input[FFT_SIZE];
static float32_t fft_input_ready[FFT_SIZE];  // Secondary buffer for main loop
static float32_t fft_output[FFT_SIZE];
static float32_t fft_magnitude[FFT_SIZE / 2];  // Half size for positive frequencies
static volatile uint32_t fft_buffer_index = 0;
static volatile uint8_t fft_buffer_full = 0;

static void generate_nco_block(float32_t *nco_buf, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        uint32_t idx = (uint32_t)(nco_phase * PHASE_TO_INDEX) % SINE_TABLE_SIZE;
        nco_buf[i] = sine_table[idx];
        nco_phase += nco_inc;
        if (nco_phase >= 2.0f * M_PI) nco_phase -= 2.0f * M_PI;
    }
}

void init_sine_table(void) {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        sine_table[i] = sinf(2.0f * M_PI * i / SINE_TABLE_SIZE);
    }
}

static void process_audio_block(int16_t *input_buf, int16_t *output_buf) {
    float32_t i_in[BLOCK_SIZE], q_in[BLOCK_SIZE], i_filt[BLOCK_SIZE], q_filt[BLOCK_SIZE];
    float32_t sum_out[BLOCK_SIZE], bpf_out[BLOCK_SIZE], mixed_out[BLOCK_SIZE], lpf_out[BLOCK_SIZE];
    float32_t nco[BLOCK_SIZE];

    // Deinterleave input buffer (I: left, Q: right)
    for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
        i_in[i] = (float32_t)input_buf[2 * i] / 32768.0f;
        q_in[i] = (float32_t)input_buf[2 * i + 1] / 32768.0f;
    }

    // Compute input RMS
    arm_rms_f32(i_in, BLOCK_SIZE, &input_rms);

    // Apply Hilbert filters
    arm_fir_f32(&fir_i, i_in, i_filt, BLOCK_SIZE);
    arm_fir_f32(&fir_q, q_in, q_filt, BLOCK_SIZE);

    // Subtract for USB image rejection (I - Q)
    arm_sub_f32(i_filt, q_filt, sum_out, BLOCK_SIZE);

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
        int16_t sample = (int16_t)(lpf_out[i] * 32767.0f * output_gain);
        sample = (sample > 32767) ? 32767 : (sample < -32768) ? -32768 : sample;
        output_buf[2 * i] = sample;
        output_buf[2 * i + 1] = sample;
    }

    // Accumulate for FFT (use sum_out for wide RF band view)
    if (!fft_buffer_full && fft_buffer_index + BLOCK_SIZE <= FFT_SIZE) {
        memcpy(&fft_input[fft_buffer_index], sum_out, BLOCK_SIZE * sizeof(float32_t));
        fft_buffer_index += BLOCK_SIZE;
        if (fft_buffer_index >= FFT_SIZE) {
            fft_buffer_full = 1;  // Signal main loop to process FFT
        }
    }

    callback_count++;
}

/* Add this define near other constants (e.g., after SPECTRUM_Y_OFFSET) */
#define MAX_MAGNITUDE    4.0f  // Fixed upper limit for FFT magnitude scaling

static void update_spectrum_display(void) {
    if (!fft_buffer_full) return;  // Wait until buffer is ready

    // Copy FFT input to secondary buffer and reset flag
    memcpy(fft_input_ready, fft_input, FFT_SIZE * sizeof(float32_t));
    fft_buffer_full = 0;
    fft_buffer_index = 0;

    // Compute FFT
    arm_rfft_fast_f32(&fft_instance, fft_input_ready, fft_output, 0);

    // Compute magnitude (only first FFT_SIZE/2 bins)
    arm_cmplx_mag_f32(fft_output, fft_magnitude, FFT_SIZE / 2);

    // Clear LCD spectrum area
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(0, SPECTRUM_Y_OFFSET, LCD_WIDTH, SPECTRUM_HEIGHT);

    // Draw grid (horizontal lines for magnitude, vertical for frequency)
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    for (int i = 1; i < 5; i++) {
        BSP_LCD_DrawHLine(0, SPECTRUM_Y_OFFSET + (SPECTRUM_HEIGHT / 5 * i), LCD_WIDTH);
    }
    for (int i = 1; i < 5; i++) {
        int freq = i * (AUDIO_FS / 2) / 5;  // Mark 4.41, 8.82, 13.23, 17.64 kHz
        int x = (int)((float)freq / (AUDIO_FS / 2) * LCD_WIDTH);
        BSP_LCD_DrawVLine(x, SPECTRUM_Y_OFFSET, SPECTRUM_HEIGHT);
    }

    // Draw vertical bars with fixed magnitude scaling
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        int x = (int)((float)i / (FFT_SIZE / 2) * LCD_WIDTH);
        int bar_width = (LCD_WIDTH + (FFT_SIZE / 2 - 1)) / (FFT_SIZE / 2);  // ~1.875 pixels
        float32_t scaled_mag = fft_magnitude[i] / MAX_MAGNITUDE;
        scaled_mag = (scaled_mag > 1.0f) ? 1.0f : scaled_mag;  // Clip to max
        int bar_height = (int)(scaled_mag * SPECTRUM_HEIGHT);
        int y = SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT - bar_height;
        BSP_LCD_FillRect(x, y, bar_width, bar_height);
    }

    // Update footer with RMS and callback count
    char lcd_text[50];
    snprintf(lcd_text, sizeof(lcd_text), "In: %.4f Out: %.4f CB: %lu", input_rms, output_rms, callback_count);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(0, SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT, LCD_WIDTH, LCD_HEIGHT - (SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT));
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(0, SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT + 10, (uint8_t*)lcd_text, LEFT_MODE);
}

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void SystemClock_Config(void);
static void AUDIO_InitApplication(void);
static void CPU_CACHE_Enable(void);
static void MX_I2C1_Init(void);

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {}
void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
    process_audio_block(&input_buffer[0], &output_buffer[0]);
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void) {
    process_audio_block(&input_buffer[AUDIO_BUFFER_SIZE / 2], &output_buffer[AUDIO_BUFFER_SIZE / 2]);
}

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
    memset(input_buffer, 0, sizeof(input_buffer));
    memset(output_buffer, 0, sizeof(output_buffer));
    memset(fir_i_state, 0, sizeof(fir_i_state));
    memset(fir_q_state, 0, sizeof(fir_q_state));
    memset(fir_bpf_state, 0, sizeof(fir_bpf_state));
    memset(fir_lpf_state, 0, sizeof(fir_lpf_state));
    memset(fft_input, 0, sizeof(fft_input));
    memset(fft_input_ready, 0, sizeof(fft_input_ready));

    init_sine_table();
    arm_fir_init_f32(&fir_i, HILBERT_TAPS, (float32_t*)hilbert_i_coeffs, fir_i_state, BLOCK_SIZE);
    arm_fir_init_f32(&fir_q, HILBERT_TAPS, (float32_t*)hilbert_q_coeffs, fir_q_state, BLOCK_SIZE);
    arm_fir_init_f32(&fir_bpf, BPF_TAPS, (float32_t*)bpf_coeffs, fir_bpf_state, BLOCK_SIZE);
    arm_fir_init_f32(&fir_lpf, LPF_TAPS, (float32_t*)lpf_coeffs, fir_lpf_state, BLOCK_SIZE);

    // Initialize FFT
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

    AUDIO_InitApplication();
    BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

    // Initial LCD setup for spectrum
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayStringAt(0, 0, (uint8_t*)"RF Spectrum (0-22.05 kHz)", CENTER_MODE);

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

        // Update spectrum display if new FFT data is ready
        if (fft_buffer_full) {
            update_spectrum_display();
        }

        HAL_Delay(10);  // Reduced delay to improve display responsiveness
    }
}

static void AUDIO_InitApplication(void) {
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);
    BSP_LCD_SelectLayer(1);
    BSP_LCD_DisplayOn();
    LCD_LOG_Init();
    LCD_LOG_SetHeader((uint8_t*)"SDR Project");

    BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1, OUTPUT_DEVICE_HEADPHONE, AUDIO_FREQUENCY,
                          DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
    BSP_AUDIO_IN_Record((uint16_t*)input_buffer, AUDIO_BUFFER_SIZE);
    BSP_AUDIO_OUT_Play((uint16_t*)output_buffer, AUDIO_BUFFER_SIZE * 2);
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
    if (ret != HAL_OK) while (1);

    ret = HAL_PWREx_EnableOverDrive();
    if (ret != HAL_OK) while (1);

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48 | RCC_PERIPHCLK_SAI2;
    PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
    PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
    PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4;
    PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
    ret = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    if (ret != HAL_OK) while (1);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
    if (ret != HAL_OK) while (1);
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
