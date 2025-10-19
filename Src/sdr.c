#include "sdr.h"
#include "si5351.h"

SDR sdr;

void SDR_init(SDR* sdr) {
	sdr->freq_step = DEFAULT_FREQUENCY_STEP;
    sdr->current_vfo = 'A'; // Default VFO
    sdr->frequency = 0;
    const int32_t correction = 978;
    si5351_Init(correction);
    SDR_set_frequency(sdr, VFO_A, 28000000);
}

void SDR_set_frequency(SDR* sdr, VFO vfo, int32_t frequency) {
	if (sdr->frequency == frequency) {
		return;
	}
	sdr->frequency = frequency;
	frequency = (frequency + INTERMEDIATE_FREQ) * 4;
    if (vfo == VFO_A) {
    	si5351_SetupCLK1(frequency, SI5351_DRIVE_STRENGTH_4MA);
        si5351_EnableOutputs(1 << 1);
    } else {
    	si5351_SetupCLK2(frequency, SI5351_DRIVE_STRENGTH_4MA);
        si5351_EnableOutputs(1 << 2);
    }
}
