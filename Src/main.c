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
static const float32_t hilbert_i_coeffs[HILBERT_TAPS] = { 0.000198724236183671,
		0.000254683831862001, 0.000315458144123207, 0.000368610207103309,
		0.000465506783939857, 0.000511928039934757, 0.000647299422206719,
		0.000698074642452268, 0.000854171713729817, 0.000945871414257390,
		0.001076408944312920, 0.001275920792277492, 0.001306915989978013,
		0.001703306484332476, 0.001550515101469794, 0.002227298340949841,
		0.001835463144518919, 0.002820293580739281, 0.002224274854022750,
		0.003419621857599474, 0.002819842043365285, 0.003926578130419233,
		0.003762668283074692, 0.004216679139836836, 0.005216147587106480,
		0.004163484463570267, 0.007339230162553567, 0.003675607102120641,
		0.010249150284403673, 0.002743452714859422, 0.013980372783435091,
		0.001489894753953287, 0.018448562889854227, 0.000219023228827659,
		0.023429266754326966, -0.000538612578294005, 0.028559517413246797,
		0.000026682439276827, 0.033366738095610159, 0.003106342852277801,
		0.037323748655479140, 0.010557712182430569, 0.039922600621868351,
		0.025820858117389513, 0.040754864597457946, 0.057491275979955001,
		0.039583257572973069, 0.140431345936612995, 0.036390015738706309,
		0.865063715035993996, 0.031391300797281890, -0.327903573861813014,
		0.025013429083301980, -0.148898771042744293, 0.017834389903486098,
		-0.094047715083500544, 0.010501189429832865, -0.064304366997204127,
		0.003638357049891278, -0.044513820587759587, -0.002235624940045236,
		-0.030331681178903912, -0.006769427152121139, -0.020066155186087276,
		-0.009808208167348372, -0.012834161617157508, -0.011388784109210482,
		-0.008006187994283959, -0.011705147912663404, -0.005030666033447775,
		-0.011054188692668034, -0.003395620019522265, -0.009774147564620667,
		-0.002641217575854609, -0.008188042301640656, -0.002384731896854369,
		-0.006561415369602986, -0.002339199541964768, -0.005079300499545814,
		-0.002318061252603545, -0.003842514275124937, -0.002225239010723332,
		-0.002879421464156411, -0.002034553450037827, -0.002166978296109626,
		-0.001764401144908235, -0.001654383125037512, -0.001453535333182090,
		-0.001283804403268615, -0.001142233712330263, -0.001004797547016575,
		-0.000860873307921386, -0.000781417508646928, -0.000625726502729441,
		-0.000593026532750594, -0.000440206606932182, -0.000430981422416596,
		-0.000299086415167579, -0.000293666182366421, -0.000193345055588135 };

// Hilbert filter coefficients for -45° (100 taps, from Python)
static const float32_t hilbert_q_coeffs[HILBERT_TAPS] = { -0.000104828886628755,
		-0.000117719664105387, -0.000179320525624148, -0.000205669060801986,
		-0.000270286475558104, -0.000332604968214234, -0.000386819028473551,
		-0.000493164616992682, -0.000552187708486295, -0.000674078778415017,
		-0.000787897902733834, -0.000880670533595121, -0.001080992707313235,
		-0.001168160302446560, -0.001365286530613688, -0.001636783384052048,
		-0.001559963634144742, -0.002356313266829644, -0.001678217635730416,
		-0.003240524231127448, -0.001945858691780062, -0.003970012170510779,
		-0.002804622726618263, -0.004094681623970159, -0.004691992111496490,
		-0.003369737266986447, -0.007627097490107733, -0.002195191592412418,
		-0.010835260483481341, -0.001845290685708202, -0.012766336781617968,
		-0.004153071025321154, -0.011756607386998292, -0.010556794294806793,
		-0.007228189577834320, -0.020846529514028617, -0.000887005875166711,
		-0.032318023634858514, 0.002824775637078142, -0.040051810693587799,
		-0.002513661566225675, -0.038562584732477104, -0.023692311071245833,
		-0.024291462351993202, -0.067204321144781837, 0.002221169819425520,
		-0.145832963504399560, 0.035705388989502325, -0.352113137784848706,
		0.067646258294101688, 0.819224519212560343, 0.089389932910583753,
		0.083898046820836417, 0.095487826549608398, 0.005097439529971089,
		0.085709969555230026, -0.010424760352139743, 0.064811294048651949,
		-0.003898907826858565, 0.040265585480685734, 0.008749537601820631,
		0.019155494269048438, 0.018678233958939739, 0.005700076331359550,
		0.022203747632268241, 0.000410110361091821, 0.019752962364774186,
		0.000933775296421688, 0.014070217079109496, 0.003844223553408773,
		0.008207357923911553, 0.006337891834889198, 0.004112629333540725,
		0.007109267624208683, 0.002222779781756527, 0.006264069949833283,
		0.001893005138071308, 0.004640304124501774, 0.002154680835501082,
		0.003076655796887437, 0.002307279081475436, 0.002003764398968816,
		0.002117456102702203, 0.001427355620025981, 0.001691149314908232,
		0.001139213958168955, 0.001232148667634720, 0.000938062422309971,
		0.000871368830893603, 0.000732817577251988, 0.000628578254366384,
		0.000526829579234758, 0.000462087337619918, 0.000353466739325737,
		0.000329895504555281, 0.000229266229860308, 0.000216471955570351,
		0.000146341941125966, 0.000125349498762248, 0.000088059239423163 };

// USB BPF coefficients (100 taps, centered at 12.5 kHz, from Python)
static const float32_t bpf_coeffs[BPF_TAPS] = { 0.000077220197574123f,
		-1.694364742523110E-6f, -0.000247376363569229f, 0.000172349071145931f,
		0.000442532978562303f, -0.000587921806885401f, -0.000490796464995067f,
		0.001245366873040035f, 0.000130200340800485f, -0.001862166817785674f,
		0.000703051894342210f, 0.002007465586182165f, -0.001685724919682768f,
		-0.001407920138837247f, 0.002116180748678689f, 0.000372547212250720f,
		-0.001360627435518710f, 0.000104411160061938f, -0.000488583862643263f,
		0.001171803660774852f, 0.002230271567546280f, -0.004622738899125535f,
		-0.001988597303309669f, 0.009091087568736349f, -0.001507388998028951f,
		-0.012022955491491360f, 0.007518507241432481f, 0.010969367705405847f,
		-0.012943254772900497f, -0.005724754564656228f, 0.013800759554407906f,
		-0.000413209911619933f, -0.008117888769062576f, 0.001659860856017020f,
		-0.001521029232922352f, 0.006868472137034775f, 0.007858709633466776f,
		-0.025100680515366711f, -0.002212548908150258f, 0.046023064611360000f,
		-0.020182170398250168f, -0.057755470091797719f, 0.055819871330333046f,
		0.049337804826868066f, -0.092667374657917745f, -0.017292461343810531f,
		0.114978094867627500f, -0.030993743432389236f, -0.111032169995744168f,
		0.079554595102602668f, 0.079554595102602668f, -0.111032169995744168f,
		-0.030993743432389236f, 0.114978094867627500f, -0.017292461343810531f,
		-0.092667374657917745f, 0.049337804826868066f, 0.055819871330333046f,
		-0.057755470091797719f, -0.020182170398250168f, 0.046023064611360000f,
		-0.002212548908150258f, -0.025100680515366711f, 0.007858709633466776f,
		0.006868472137034775f, -0.001521029232922352f, 0.001659860856017020f,
		-0.008117888769062576f, -0.000413209911619933f, 0.013800759554407906f,
		-0.005724754564656228f, -0.012943254772900497f, 0.010969367705405847f,
		0.007518507241432481f, -0.012022955491491360f, -0.001507388998028951f,
		0.009091087568736349f, -0.001988597303309669f, -0.004622738899125535f,
		0.002230271567546280f, 0.001171803660774852f, -0.000488583862643263f,
		0.000104411160061938f, -0.001360627435518710f, 0.000372547212250720f,
		0.002116180748678689f, -0.001407920138837247f, -0.001685724919682768f,
		0.002007465586182165f, 0.000703051894342210f, -0.001862166817785674f,
		0.000130200340800485f, 0.001245366873040035f, -0.000490796464995067f,
		-0.000587921806885401f, 0.000442532978562303f, 0.000172349071145931f,
		-0.000247376363569229f, -1.694364742523110E-6f, 0.000077220197574123f };

// LPF coefficients (54 taps, 0-2700 Hz, from Python)
static const float32_t lpf_coeffs[LPF_TAPS] = { -0.007202148437500f,
		-0.017944335937500f, -0.011901855468750f, -0.017517089843750f,
		-0.013946533203125f, -0.010192871093750f, -0.002105712890625f,
		0.007202148437500f, 0.016906738281250f, 0.024322509765625f,
		0.027465820312500f, 0.024597167968750f, 0.015075683593750f,
		-0.000366210937500f, -0.019317626953125f, -0.038238525390625f,
		-0.052551269531250f, -0.057739257812500f, -0.049926757812500f,
		-0.026977539062500f, 0.011047363281250f, 0.061584472656250f,
		0.119720458984375f, 0.178955078125000f, 0.231933593750000f,
		0.271911621093750f, 0.293151855468750f, 0.293151855468750f,
		0.271911621093750f, 0.231933593750000f, 0.178955078125000f,
		0.119720458984375f, 0.061584472656250f, 0.011047363281250f,
		-0.026977539062500f, -0.049926757812500f, -0.057739257812500f,
		-0.052551269531250f, -0.038238525390625f, -0.019317626953125f,
		-0.000366210937500f, 0.015075683593750f, 0.024597167968750f,
		0.027465820312500f, 0.024322509765625f, 0.016906738281250f,
		0.007202148437500f, -0.002105712890625f, -0.010192871093750f,
		-0.013946533203125f, -0.017517089843750f, -0.011901855468750f,
		-0.017944335937500f, -0.007202148437500f };

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
static float32_t fft_magnitude[FFT_SIZE / 2]; // Half size for positive frequencies
static volatile uint32_t fft_buffer_index = 0;
static volatile uint8_t fft_buffer_full = 0;

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
		sine_table[i] = sinf(2.0f * M_PI * i / SINE_TABLE_SIZE);
	}
}

/* Ensure these global variables are added near other globals (e.g., after output_rms), if not already present */
float32_t i_filt_rms = 0.0f;
float32_t q_filt_rms = 0.0f;
float32_t sum_out_rms = 0.0f;

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

	// Accumulate for FFT (use sum_out for wide RF band view)
	if (!fft_buffer_full && fft_buffer_index + BLOCK_SIZE <= FFT_SIZE) {
		memcpy(&fft_input[fft_buffer_index], sum_out,
				BLOCK_SIZE * sizeof(float32_t));
		fft_buffer_index += BLOCK_SIZE;
		if (fft_buffer_index >= FFT_SIZE) {
			fft_buffer_full = 1;  // Signal main loop to process FFT
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

/* Add this define near other constants (e.g., after SPECTRUM_Y_OFFSET) */
#define MAX_MAGNITUDE    4.0f  // Fixed upper limit for FFT magnitude scaling

/* Modified update_spectrum_display function */
static void update_spectrum_display(void) {
	if (!fft_buffer_full)
		return;  // Wait until buffer is ready

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
		BSP_LCD_DrawHLine(0, SPECTRUM_Y_OFFSET + (SPECTRUM_HEIGHT / 5 * i),
				LCD_WIDTH);
	}
	for (int i = 1; i < 5; i++) {
		int freq = i * (AUDIO_FS / 2) / 5;  // Mark 4.41, 8.82, 13.23, 17.64 kHz
		int x = (int) ((float) freq / (AUDIO_FS / 2) * LCD_WIDTH);
		BSP_LCD_DrawVLine(x, SPECTRUM_Y_OFFSET, SPECTRUM_HEIGHT);
	}

	// Draw vertical bars with fixed magnitude scaling
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	for (int i = 0; i < FFT_SIZE / 2; i++) {
		int x = (int) ((float) i / (FFT_SIZE / 2) * LCD_WIDTH);
		int bar_width = (LCD_WIDTH + (FFT_SIZE / 2 - 1)) / (FFT_SIZE / 2); // ~1.875 pixels
		float32_t scaled_mag = fft_magnitude[i] / MAX_MAGNITUDE;
		scaled_mag = (scaled_mag > 1.0f) ? 1.0f : scaled_mag;  // Clip to max
		int bar_height = (int) (scaled_mag * SPECTRUM_HEIGHT);
		int y = SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT - bar_height;
		BSP_LCD_FillRect(x, y, bar_width, bar_height);
	}

	// Update footer with RMS and callback count
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font12);  // Use smallest font to ensure fit
	BSP_LCD_FillRect(0, SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT, LCD_WIDTH,
			LCD_HEIGHT - (SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT));
	char lcd_text[64];
	snprintf(lcd_text, sizeof(lcd_text),
			"In:%.2f I:%.2f Q:%.2f Sum:%.2f Out:%.2f CB:%lu", input_rms,
			i_filt_rms, q_filt_rms, sum_out_rms, output_rms, callback_count);
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_DisplayStringAt(0, SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT + 4,
			(uint8_t*) lcd_text, LEFT_MODE);
}

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void SystemClock_Config(void);
static void AUDIO_InitApplication(void);
static void CPU_CACHE_Enable(void);
static void MX_I2C1_Init(void);

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

	// Initial LCD setup for spectrum
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0, 0, (uint8_t*) "RF Spectrum (0-22.05 kHz)",
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
	LCD_LOG_SetHeader((uint8_t*) "SDR Project");

	BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1, OUTPUT_DEVICE_HEADPHONE,
			AUDIO_FREQUENCY,
			DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
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

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
