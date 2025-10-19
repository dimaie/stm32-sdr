#include "sdr.h"

SDR sdr;

void SDR_init(SDR* sdr) {
	sdr->freq_step = DEFAULT_FREQUENCY_STEP;
    sdr->current_vfo = 'A'; // Default VFO
    sdr->frequency = 7001000; // Default 7.001 MHz
}
