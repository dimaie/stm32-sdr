#include "dsp.h"
#include "filters.h"
#include "stm32746g_discovery_lcd.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define BIN_TO_COMPENSATE_ATTENUATION 220

// Global DSP context
DSPContext dsp;

// Initialize DSP context
void DSP_Init(DSPContext *dsp) {
	// Initialize buffers
	memset(dsp->input_buffer, 0, sizeof(dsp->input_buffer));
	memset(dsp->output_buffer, 0, sizeof(dsp->output_buffer));
	memset(dsp->fir_i_state, 0, sizeof(dsp->fir_i_state));
	memset(dsp->fir_q_state, 0, sizeof(dsp->fir_q_state));
	memset(dsp->fir_bpf_state, 0, sizeof(dsp->fir_bpf_state));
	memset(dsp->fir_lpf_state, 0, sizeof(dsp->fir_lpf_state));
	memset(dsp->fft_input, 0, sizeof(dsp->fft_input));
	memset(dsp->fft_input_ready, 0, sizeof(dsp->fft_input_ready));
	memset(dsp->fft_magnitude_smoothed, 0, sizeof(dsp->fft_magnitude_smoothed));

	// Initialize state variables
	dsp->fft_buffer_index = 0;
	dsp->fft_buffer_full = 0;
	dsp->nco_phase = 0.0f;
	dsp->input_rms = 0.0f;
	dsp->output_rms = 0.0f;
	dsp->i_filt_rms = 0.0f;
	dsp->q_filt_rms = 0.0f;
	dsp->sum_out_rms = 0.0f;
	dsp->callback_count = 0;
	dsp->output_gain = 1.0f;
	dsp->demod_mode = 0; // USB
	dsp->nco_inc = 2.0f * M_PI * NCO_FREQ / AUDIO_FS;

	// Initialize gain table for FFT magnitude compensation
	for (int i = 0; i < FFT_SIZE / 2; i++) {
		if (i >= BIN_TO_COMPENSATE_ATTENUATION) {
			dsp->fft_gain[i] = 1.0f + 0.22f * expf(0.1f * (i - BIN_TO_COMPENSATE_ATTENUATION));
		} else {
			dsp->fft_gain[i] = 1.0f;
		}
	}
	// Initialize sine table and Hann window
	for (int i = 0; i < SINE_TABLE_SIZE; i++) {
		dsp->sine_table[i] = sinf(2.0f * M_PI * i / SINE_TABLE_SIZE);
	}
	for (int i = 0; i < FFT_SIZE; i++) {
		dsp->window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_SIZE - 1)));
	}

	// Initialize filters
	arm_fir_init_f32(&dsp->fir_i, HILBERT_TAPS, (float32_t*) hilbert_i_coeffs,
			dsp->fir_i_state, BLOCK_SIZE);
	arm_fir_init_f32(&dsp->fir_q, HILBERT_TAPS, (float32_t*) hilbert_q_coeffs,
			dsp->fir_q_state, BLOCK_SIZE);
	arm_fir_init_f32(&dsp->fir_bpf, BPF_TAPS, (float32_t*) bpf_coeffs,
			dsp->fir_bpf_state, BLOCK_SIZE);
	arm_fir_init_f32(&dsp->fir_lpf, LPF_TAPS, (float32_t*) lpf_coeffs,
			dsp->fir_lpf_state, BLOCK_SIZE);

	// Initialize FFT
	arm_rfft_fast_init_f32(&dsp->fft_instance, FFT_SIZE);
}

// Generate NCO block
static void DSP_GenerateNCOBlock(DSPContext *dsp, float32_t *nco_buf,
		uint32_t size) {
	for (uint32_t i = 0; i < size; i++) {
		uint32_t idx = (uint32_t) (dsp->nco_phase * PHASE_TO_INDEX)
				% SINE_TABLE_SIZE;
		nco_buf[i] = dsp->sine_table[idx];
		dsp->nco_phase += dsp->nco_inc;
		if (dsp->nco_phase >= 2.0f * M_PI)
			dsp->nco_phase -= 2.0f * M_PI;
	}
}

// Process audio block
void DSP_ProcessAudioBlock(DSPContext *dsp, int16_t *input_buf,
		int16_t *output_buf) {
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
	arm_rms_f32(i_in, BLOCK_SIZE, &dsp->input_rms);

	// Apply Hilbert filters
	arm_fir_f32(&dsp->fir_i, i_in, i_filt, BLOCK_SIZE);
	arm_fir_f32(&dsp->fir_q, q_in, q_filt, BLOCK_SIZE);

	// Subtract for USB image rejection (I - Q)
	arm_sub_f32(i_filt, q_filt, sum_out, BLOCK_SIZE);

	// Accumulate for FFT
	if (!dsp->fft_buffer_full && dsp->fft_buffer_index + BLOCK_SIZE <= FFT_SIZE) {
		memcpy(&dsp->fft_input[dsp->fft_buffer_index], sum_out,
				BLOCK_SIZE * sizeof(float32_t));
		dsp->fft_buffer_index += BLOCK_SIZE;
		if (dsp->fft_buffer_index >= FFT_SIZE) {
			dsp->fft_buffer_full = 1;
		}
	}

	// Compute RMS values
	arm_rms_f32(i_filt, BLOCK_SIZE, &dsp->i_filt_rms);
	arm_rms_f32(q_filt, BLOCK_SIZE, &dsp->q_filt_rms);
	arm_rms_f32(sum_out, BLOCK_SIZE, &dsp->sum_out_rms);

	// Apply USB BPF
	arm_fir_f32(&dsp->fir_bpf, sum_out, bpf_out, BLOCK_SIZE);

	// Generate NCO
	DSP_GenerateNCOBlock(dsp, nco, BLOCK_SIZE);

	// Mix with NCO
	arm_mult_f32(bpf_out, nco, mixed_out, BLOCK_SIZE);

	// Apply LPF
	arm_fir_f32(&dsp->fir_lpf, mixed_out, lpf_out, BLOCK_SIZE);

	// Compute output RMS
	arm_rms_f32(lpf_out, BLOCK_SIZE, &dsp->output_rms);

	// Output to both channels
	for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
		int16_t sample = (int16_t) (lpf_out[i] * 32767.0f * dsp->output_gain);
		sample = (sample > 32767) ? 32767 : (sample < -32768) ? -32768 : sample;
		output_buf[2 * i] = sample;
		output_buf[2 * i + 1] = sample;
	}

	dsp->callback_count++;
}

int get_x(int i) {
	return (LCD_WIDTH - 1) - (int) ((float) i / (FFT_SIZE / 2) * LCD_WIDTH);
}

// Update spectrum display
void update_spectrum_display(DSPContext *dsp) {
	static int16_t fft_update_counter = 0;
	static int16_t fft_grid_update_counter = 0;
	static int prev_bar_heights[FFT_SIZE / 2] = {0}; // Store previous bar heights

	if (!dsp->fft_buffer_full)
		return;

	fft_update_counter++;
	fft_grid_update_counter++; // Increment grid counter only when processing FFT buffer

	// Draw grid every FFT_GRID_FREQUENCY_FACTOR calls (~232 ms) - disabled per user
	if (fft_grid_update_counter % FFT_GRID_FREQUENCY_FACTOR == 0) {
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		for (int i = 1; i < 5; i++) {
			BSP_LCD_DrawHLine(0, SPECTRUM_Y_OFFSET + (SPECTRUM_HEIGHT / 5 * i),
					LCD_WIDTH);
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
	memcpy(dsp->fft_input_ready, dsp->fft_input, FFT_SIZE * sizeof(float32_t));
	dsp->fft_buffer_full = 0;
	dsp->fft_buffer_index = 0;

	// Apply Hann window
	for (int i = 0; i < FFT_SIZE; i++) {
		dsp->fft_input_ready[i] *= dsp->window[i];
	}

	// Compute FFT
	arm_rfft_fast_f32(&dsp->fft_instance, dsp->fft_input_ready, dsp->fft_output, 0);

	// Compute magnitude
	arm_cmplx_mag_f32(dsp->fft_output, dsp->fft_magnitude, FFT_SIZE / 2);

#if ENABLE_FFT_SMOOTHING
	// Apply exponential moving average (EMA) smoothing
	for (int i = 0; i < FFT_SIZE / 2; i++) {
		dsp->fft_magnitude_smoothed[i] = FFT_SMOOTH_ALPHA * dsp->fft_magnitude[i] +
		                                (1.0f - FFT_SMOOTH_ALPHA) * dsp->fft_magnitude_smoothed[i];
	}
#endif

	int bar_width = (LCD_WIDTH + (FFT_SIZE / 2 - 1)) / (FFT_SIZE / 2);
	// Update bars based on height differences
	for (int i = 0; i < FFT_SIZE / 2; i++) {
		int x = get_x(i); // Reversed: high freq (~22 kHz, ~6.799 MHz) at x≈0 (left), low freq (0 Hz, ~7.001 MHz) at x≈479 (right)
		// Apply precomputed gain for all bins
		float32_t gain = dsp->fft_gain[i];
#if ENABLE_FFT_SMOOTHING
		float32_t scaled_mag = gain * dsp->fft_magnitude_smoothed[i] / MAX_MAGNITUDE;
#else
		float32_t scaled_mag = gain * dsp->fft_magnitude[i] / MAX_MAGNITUDE;
#endif
		scaled_mag = (scaled_mag > 1.0f) ? 1.0f : scaled_mag;
		int bar_height = (int)(scaled_mag * SPECTRUM_HEIGHT);
		int prev_height = prev_bar_heights[i];
		int y = SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT - bar_height;

		if (bar_height > prev_height) {
			// Draw additional height for taller bars
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
			int draw_height = bar_height - prev_height;
			int draw_y = SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT - bar_height;
			BSP_LCD_FillRect(x, draw_y, bar_width, draw_height);
		} else if (bar_height < prev_height) {
			// Clear difference for shorter bars
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			int clear_height = prev_height - bar_height;
			int clear_y = SPECTRUM_Y_OFFSET + SPECTRUM_HEIGHT - prev_height;
			BSP_LCD_FillRect(x, clear_y, bar_width, clear_height);
			// Draw new bar
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
			BSP_LCD_FillRect(x, y, bar_width, bar_height);
		}
		// Skip if heights are equal to reduce draws
		prev_bar_heights[i] = bar_height; // Update stored height
	}

	fft_update_counter = 0; // Reset counter to prevent overflow

#ifdef DEBUG
	// Send debug info to UART terminal
	char debug_text[64];
	snprintf(debug_text, sizeof(debug_text),
	         "In:%.2f I:%.2f Q:%.2f Sum:%.2f Out:%.2f CB:%lu\r\n",
	         dsp->input_rms, dsp->i_filt_rms, dsp->q_filt_rms, dsp->sum_out_rms,
	         dsp->output_rms, dsp->callback_count);
	HAL_UART_Transmit(&huart1, (uint8_t*)debug_text, strlen(debug_text), HAL_MAX_DELAY);
#endif
}
