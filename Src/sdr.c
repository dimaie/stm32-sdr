#include "sdr.h"

SDR sdr;

void SDR_init(SDR* sdr) {
	sdr->freq_step = DEFAULT_FREQUENCY_STEP;
}
