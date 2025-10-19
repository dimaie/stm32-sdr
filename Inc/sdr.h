#pragma once
#include <stdint.h>

#define DEFAULT_FREQUENCY_STEP 	50
#define INTERMEDIATE_FREQ		11000
typedef struct {
    char current_vfo; // Current VFO ('A' or 'B')
    uint32_t frequency; // Current frequency (Hz)
    uint32_t freq_step; // Default frequency step (Hz)
} SDR;

typedef enum {
    VFO_A,
    VFO_B
} VFO;

extern SDR sdr;

void SDR_init(SDR* sdr);
void SDR_set_frequency(SDR* sdr, VFO vfo, int32_t frequency);
