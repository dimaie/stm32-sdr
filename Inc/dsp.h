#ifndef DSP_H
#define DSP_H

#include "arm_math.h"
#include "stm32f7xx_hal.h"
#include "filters.h"

// Constants (duplicated from main code to avoid dependency on main.c)
#define FFT_SIZE            512
#define AUDIO_BUFFER_SIZE   512
#define BLOCK_SIZE          (AUDIO_BUFFER_SIZE / 4)
#define SINE_TABLE_SIZE     512
#define AUDIO_FS            44100.0f
#define NCO_FREQ            11000.0f
#define PHASE_TO_INDEX      ((float)SINE_TABLE_SIZE / (2.0f * M_PI))
#define FFT_SMOOTH_ALPHA    0.3f
#define ENABLE_FFT_SMOOTHING 1
#define FFT_FREQUENCY_FACTOR 25
#define FFT_GRID_FREQUENCY_FACTOR 20
#define LCD_WIDTH           480
#define SPECTRUM_HEIGHT     200
#define SPECTRUM_Y_OFFSET   20
#define MAX_MAGNITUDE       4.0f

// DSP Context structure
typedef struct {
    // Buffers
    float32_t fft_magnitude[FFT_SIZE / 2];
    float32_t fft_magnitude_smoothed[FFT_SIZE / 2];
    int16_t output_buffer[AUDIO_BUFFER_SIZE];
    int16_t input_buffer[AUDIO_BUFFER_SIZE];
    float32_t fft_input[FFT_SIZE];
    float32_t fft_input_ready[FFT_SIZE];
    float32_t fft_output[FFT_SIZE];
    float32_t window[FFT_SIZE];
    float32_t sine_table[SINE_TABLE_SIZE];
    float32_t fir_i_state[HILBERT_TAPS + BLOCK_SIZE - 1];
    float32_t fir_q_state[HILBERT_TAPS + BLOCK_SIZE - 1];
    float32_t fir_bpf_state[BPF_TAPS + BLOCK_SIZE - 1];
    float32_t fir_lpf_state[LPF_TAPS + BLOCK_SIZE - 1];

    // Filter instances
    arm_fir_instance_f32 fir_i;
    arm_fir_instance_f32 fir_q;
    arm_fir_instance_f32 fir_bpf;
    arm_fir_instance_f32 fir_lpf;

    // FFT instance
    arm_rfft_fast_instance_f32 fft_instance;

    // State variables
    volatile uint32_t fft_buffer_index;
    volatile uint8_t fft_buffer_full;
    float32_t nco_phase;
    float32_t input_rms;
    float32_t output_rms;
    float32_t i_filt_rms;
    float32_t q_filt_rms;
    float32_t sum_out_rms;
    uint32_t callback_count;
    float32_t output_gain;
    uint8_t demod_mode;
    uint8_t button_pressed;

    float32_t nco_inc;
} DSPContext;

extern DSPContext dsp;
// Public DSP functions
void DSP_Init(DSPContext *dsp);
void DSP_ProcessAudioBlock(DSPContext *dsp, int16_t *input_buf, int16_t *output_buf);
void update_spectrum_display(DSPContext *dsp);

#endif // DSP_H
