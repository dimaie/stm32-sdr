#pragma once
#include <stdint.h>

#define DEFAULT_FREQUENCY_STEP 50

typedef struct {
    char current_vfo; // Current VFO ('A' or 'B')
    uint32_t frequency; // Current frequency (Hz)
    uint32_t freq_step; // Default frequency step (Hz)
} SDR;

extern SDR sdr;

void SDR_init(SDR* sdr);
