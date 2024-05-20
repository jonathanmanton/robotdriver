#ifdef __cplusplus
extern "C" {
#endif

// interface for the quadrature encoder helper functions
// Based on the quadrature encoder demo from the pico SDK

#include "quadrature_encoder.pio.h"

typedef struct quadrature_encoder {
	PIO pio;
	uint sm;
	uint pin_ab;
} quadrature_encoder;

void qe_init(quadrature_encoder* qe);
int32_t qe_get_count(quadrature_encoder* qe);

#ifdef __cplusplus
}
#endif
