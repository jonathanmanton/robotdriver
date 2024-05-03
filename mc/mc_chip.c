#include "mc_chip.h"
#include <math.h>

// Note there is no error handling here.

// all we do here is set standby to "high" to enable chip
void init_chip(uint STBY_pin) {
        gpio_init(STBY_pin);
        gpio_set_dir(STBY_pin, GPIO_OUT);
	gpio_put(STBY_pin, 1);
}
	


// Set up IN1 and IN2 pins, and set up the PWM module
void init_channel(mc_channel* channel, uint IN1_pin, uint IN2_pin, uint PWM_pin, float direction) {

	channel->IN1_pin = IN1_pin;
	channel->IN2_pin = IN2_pin;
	channel->PWM_pin = PWM_pin;
	channel->direction = direction;

  // set up gpios and set to zero (stop mode)

        gpio_init(IN1_pin);
        gpio_init(IN2_pin);
        gpio_set_dir(IN1_pin, GPIO_OUT);
        gpio_set_dir(IN2_pin, GPIO_OUT);
	gpio_put(IN1_pin, 0);
	gpio_put(IN2_pin, 0);

  // configure PWM and start with level of 0

	gpio_set_function(PWM_pin, GPIO_FUNC_PWM);

    	  // Figure out which slice we just connected to the LED pin
    	uint slice_num = pwm_gpio_to_slice_num(PWM_pin);

	  // Get some sensible defaults for the slice configuration. 
	  // By default, the counter is allowed to wrap over its 
	  // maximum range (0 to 2**16-1)
	pwm_config config = pwm_get_default_config();

	  // Sysclock for Pico is 125MHz.  64K steps means about 2KHz period 
	  // Load the configuration into our PWM slice, and set it running.

	pwm_init(slice_num, &config, true);
	pwm_set_gpio_level(PWM_pin, 0);

}

// manipulate output pins to drive the motor 
void set_effort(mc_channel* channel, float effort) {
	  // effort must be between -1 and 1 inclusive
	if (effort > 1.0)
		effort = 1.0;
	if (effort < -1.0)
		effort = -1.0;

	  // normalize direction to CW vs CCW
	effort = effort * channel->direction;

	  // set direction based on effort.  > 0 is CW, <= 0 is CCW
	if (effort > 0) {
		gpio_put(channel->IN1_pin, 1);
		gpio_put(channel->IN2_pin, 0);
	} else {
		gpio_put(channel->IN1_pin, 0);
		gpio_put(channel->IN2_pin, 1);
	}
		
	  // set PWM level
	uint int_effort = (uint)(fabs(effort) * (float)((1 << 16) - 1));
	pwm_set_gpio_level(channel->PWM_pin, int_effort);
}

void stop(mc_channel* channel) {
	gpio_put(channel->IN1_pin, 0);
	gpio_put(channel->IN2_pin, 0);
	pwm_set_gpio_level(channel->PWM_pin, 0);
}

