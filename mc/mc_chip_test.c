#include "pico/stdlib.h"
#include "mc_chip.h"

#define RIGHT_IN1_PIN	2
#define RIGHT_IN2_PIN	3
#define RIGHT_PWM_PIN	4
#define LEFT_IN1_PIN	5
#define LEFT_IN2_PIN	6
#define LEFT_PWM_PIN	7
#define CHIP_STBY_PIN	8

mc_channel right_channel, left_channel;

void main() {
	init_chip(CHIP_STBY_PIN);
	init_channel(& right_channel, RIGHT_IN1_PIN, 
			RIGHT_IN2_PIN, RIGHT_PWM_PIN, DIR_REVERSE);
	init_channel(& left_channel, LEFT_IN1_PIN, 
			LEFT_IN2_PIN, LEFT_PWM_PIN, DIR_NORMAL);

	set_effort(& right_channel, 0.5);
	set_effort(& left_channel, 0.3);
	
	sleep_ms(2000);

	set_effort(& right_channel, 0);
	set_effort(& left_channel, 0);

	sleep_ms(1000);

	set_effort(& right_channel, -0.5);
	set_effort(& left_channel, -1.0);

	sleep_ms(2000);

	stop(& right_channel);
	stop(& left_channel);

	sleep_ms(2000);

	set_effort(& right_channel, -1.0);
	set_effort(& left_channel, 1.0);

	sleep_ms(2000);
	
	stop(& right_channel);
	stop(& left_channel);
}
