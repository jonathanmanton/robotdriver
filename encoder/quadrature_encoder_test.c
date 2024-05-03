/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
// This code is modified from the quadrature_encoder demo included with the
// Pico SDK

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "quadrature_encoder.h"

quadrature_encoder left, right;

int main() {
    	int new_value, delta, old_value = 0;
    	int new_value2, delta2, old_value2 = 0;

	left.pio = pio0;
	left.sm = 0;
	left.pin_ab = 10;

	right.pio = pio1;
	right.sm = 0;
	right.pin_ab = 12;

    	stdio_init_all();

	qe_init(&left);
	qe_init(&right);


    while (1) {
        // note: thanks to two's complement arithmetic delta will always
        // be correct even when new_value wraps around MAXINT / MININT
        new_value = qe_get_count(&right);
        new_value2 = qe_get_count(&left);
        delta = new_value - old_value;
        delta2 = new_value2 - old_value2;
        old_value = new_value;
        old_value2 = new_value2;

        printf("position %8d, delta %6d\n", new_value, delta);
        printf("position2 %8d, delta2 %6d\n\n", new_value2, delta2);
        sleep_ms(500);
    }
}

