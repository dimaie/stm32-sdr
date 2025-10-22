#pragma once
#include <stdint.h>
#include <stddef.h>

#define DEFAULT_FREQUENCY_STEP 	50
#define INTERMEDIATE_FREQ		11000

typedef struct Component {
    void (*draw)(struct Component* self);
    int8_t invalid;
} Component;

typedef struct {
    char current_vfo; // Current VFO ('A' or 'B')
    uint32_t frequency; // Current frequency (Hz)
    uint32_t freq_step; // Default frequency step (Hz)
    Component** drawables; // Drawable components
    size_t drawables_count; // Size of the drawables array
} SDR;

typedef enum {
    VFO_A,
    VFO_B
} VFO;

extern SDR sdr;

void SDR_init(SDR* sdr);
void SDR_set_frequency(SDR* sdr, VFO vfo, int32_t frequency);
void SDR_draw(SDR *sdr);

typedef struct Button {
    Component base; // embedded, not a pointer!
    void (*action)(struct Component* self);
    int8_t latching;
} Button;

void Button_draw(Component* self);

typedef struct FrequencyPanel {
    Component base;
} FrequencyPanel;

void FrequencyPanel_draw(Component* self);
