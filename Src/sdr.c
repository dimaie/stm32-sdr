#include "sdr.h"
#include "si5351.h"

SDR sdr;

void Button_draw(Component *self) {
	Button *button = (Button*) self;
}

void FrequencyPanel_draw(Component *self) {
	FrequencyPanel *frequency_panel = (FrequencyPanel*) self;
}

static Button filter_button = { .base = { .draw = Button_draw }, .latching = 0 };
static FrequencyPanel frequency_panel = { .base =
		{ .draw = FrequencyPanel_draw } };

void SDR_init(SDR *sdr) {
	sdr->freq_step = DEFAULT_FREQUENCY_STEP;
	sdr->current_vfo = 'A'; // Default VFO
	sdr->frequency = 0;
	const int32_t correction = 978;
	si5351_Init(correction);
	SDR_set_frequency(sdr, VFO_A, 28000000);
	static Component *_drawables[] = {
			(Component*)&filter_button,
			(Component*)&frequency_panel
	};
	sdr->drawables = _drawables;
	sdr->drawables_count = sizeof(_drawables) / sizeof(_drawables[0]);
}

void SDR_set_frequency(SDR *sdr, VFO vfo, int32_t frequency) {
	if (sdr->frequency == frequency) {
		return;
	}
	sdr->frequency = frequency;
	if (vfo == VFO_A) {
		frequency = (frequency + INTERMEDIATE_FREQ) * 4;
		si5351_SetupCLK1(frequency, SI5351_DRIVE_STRENGTH_4MA);
		si5351_EnableOutputs(1 << 1);
	} else {
		si5351_SetupCLK2(frequency, SI5351_DRIVE_STRENGTH_4MA);
		si5351_EnableOutputs(1 << 2);
	}
}

void SDR_draw(SDR *sdr) {
	if (!sdr || !sdr->drawables)
		return;

	for (size_t i = 0; i < sdr->drawables_count; i++) {
		Component *c = sdr->drawables[i];
		if (!c)
			continue;

		// Only draw components that are invalid (need redraw)
		if (c->invalid) {
			c->draw(c);
			c->invalid = 0;  // mark as valid after drawing
		}
	}
}

