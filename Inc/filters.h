#pragma once
#include "arm_math.h"

#define HILBERT_TAPS        100
#define BPF_TAPS            100
#define LPF_TAPS            54

extern const float32_t hilbert_i_coeffs[HILBERT_TAPS];
extern const float32_t hilbert_q_coeffs[HILBERT_TAPS];
extern const float32_t bpf_coeffs[BPF_TAPS];
extern const float32_t lpf_coeffs[LPF_TAPS];
