#ifdef __cplusplus
extern "C" {
#endif

// controls the breakout board for the motor driver
// make sure to only use GPIOs for PWMs that do not differ by 16.
// e.g., don't use 3 for one PWM, and 19 for the other, because it won't work

#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define DIR_NORMAL 1.0
#define DIR_REVERSE -1.0

typedef struct mc_channel {
	uint IN1_pin;
	uint IN2_pin;
	uint PWM_pin;
	float direction;
} mc_channel;

// set the standby pin to high (which enables the driver)
void init_chip(uint STBY_pin);

void init_channel(mc_channel* channel, uint IN1_pin, uint IN2_pin, uint PWM_pin, float direction);

void set_effort(mc_channel* channel, float effort);

void stop(mc_channel* channel);

#ifdef __cplusplus
}
#endif
