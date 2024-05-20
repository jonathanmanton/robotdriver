#include <rcutils/time.h>
#include "../encoder/quadrature_encoder.h"
#include "../mc/mc_chip.h"
#include "../pid/pid.hpp"

#define	RIGHT_MOTOR	0
#define	LEFT_MOTOR	1

typedef struct robot_data {
	// placeholder for PID classes right and left
	//	pid[2];
	quadrature_encoder	encoder[2];
	mc_channel		channel[2];
	float			motor_effort[2];
	float			commanded_velocity[2];
	float			current_velocity[2];
	float			last_velocity[2];
	bool			pid_enabled;
	int32_t			current_position[2];
	int32_t			last_position[2];
	control_toolbox::Pid			pid[2];
} robot_data;

void init_robot_data(robot_data * prd);
