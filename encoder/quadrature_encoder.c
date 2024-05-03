/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
// based on the quadrature encoder demo from the Pico SDK

#include "hardware/pio.h"
#include "quadrature_encoder.h"

void qe_init(quadrature_encoder* qe) {
    pio_add_program(qe->pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(qe->pio, qe->sm, qe->pin_ab, 0);

}
int32_t qe_get_count(quadrature_encoder* qe) {
	return quadrature_encoder_get_count(qe->pio, qe->sm);
}

