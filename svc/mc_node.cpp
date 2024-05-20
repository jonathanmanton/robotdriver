#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <rcutils/time.h>

#include "robotdriver_interfaces/srv/motor_effort.h"
#include "robotdriver_interfaces/srv/motor_velocity.h"
#include "robotdriver_interfaces/srv/pid_gains.h"
#include "robotdriver_interfaces/msg/controller_status.h"
#include "robotdriver_interfaces/msg/motor_status.h"
#include <std_srvs/srv/empty.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "node_helpers.h"

#include "mc_node.h"
#include "pin_config.h"

using control_toolbox::Pid;

const uint LED_PIN = 25;

node_data nd;
robot_data rd;

const char* node_name = "pico_node";

rcl_publisher_t MotorStatus_publisher[2];
rcl_publisher_t ControllerStatus_publisher[2];
robotdriver_interfaces__msg__MotorStatus MotorStatus_msg[2];
robotdriver_interfaces__msg__ControllerStatus ControllerStatus_msg[2];

rcl_service_t MotorEffort_service;
const char* MotorEffort_service_name = "/set_motor_effort";
robotdriver_interfaces__srv__MotorEffort_Request MotorEffort_request_msg;
robotdriver_interfaces__srv__MotorEffort_Response MotorEffort_response_msg;

rcl_service_t MotorVelocity_service;
const char* MotorVelocity_service_name = "/set_motor_velocity";
robotdriver_interfaces__srv__MotorVelocity_Request MotorVelocity_request_msg;
robotdriver_interfaces__srv__MotorVelocity_Response MotorVelocity_response_msg;

rcl_service_t SetPIDGains_service;
const char* SetPIDGains_service_name = "/set_pid_gains";
robotdriver_interfaces__srv__PIDGains_Request SetPIDGains_request_msg;
robotdriver_interfaces__srv__PIDGains_Response SetPIDGains_response_msg;

rcl_service_t ResetController_service;
const char* ResetController_service_name = "/reset_controller";
std_srvs__srv__Empty_Request ResetController_request_msg;
std_srvs__srv__Empty_Response ResetController_response_msg;

void populate_controller_status()
{
	double pe, ie, de;
	for(int i = 0; i < 2; i++) {
		ControllerStatus_msg[i].commanded_velocity = 
					rd.commanded_velocity[i];
	
		rd.pid[i].getCurrentPIDErrors(pe, ie, de);
		ControllerStatus_msg[i].velocity_error = pe;
		ControllerStatus_msg[i].integrated_velocity_error = ie;
		ControllerStatus_msg[i].velocity_derivative = de;

		ControllerStatus_msg[i].p_effort = 0.0;
		ControllerStatus_msg[i].i_effort = 0.0;
		ControllerStatus_msg[i].d_effort = 0.0;
		ControllerStatus_msg[i].i_saturated = false;
	}
}

void populate_motor_status() 
{
	for(int i = 0; i < 2; i++) {
		MotorStatus_msg[i].motor_velocity = rd.current_velocity[i];
		MotorStatus_msg[i].motor_position = rd.current_position[i];
		MotorStatus_msg[i].motor_effort = rd.motor_effort[i];
	}
}


// ********************************************************************
// data timer callback
// ********************************************************************
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	populate_controller_status();
	populate_motor_status();

	rcl_ret_t ret = 0;
	for(int i = 0; i < 2; i++) {
		ret += rcl_publish(&MotorStatus_publisher[i], 
				&MotorStatus_msg[i], NULL);
		ret += rcl_publish(&ControllerStatus_publisher[i], 
				&ControllerStatus_msg[i], NULL);
	}
}

// ********************************************************************
// controller timer callback
// ********************************************************************
void controller_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	// do two things.  First update position and velocity.
	// then, IF PID loop is configured, set the effort based on the PID
	// controller

	double v_dot[2];
	for (int i = 0; i<2 ; i++) {
		rd.last_position[i] = rd.current_position[i];
		rd.current_position[i] = qe_get_count(&(rd.encoder[i]));
		rd.last_velocity[i] = rd.current_velocity[i];
		if (last_call_time > 0) {
			rd.current_velocity[i] = (rd.current_position[i] - rd.last_position[i])/(last_call_time * 1e-9);
			v_dot[i] = (rd.current_velocity[i] - rd.last_velocity[i])/(last_call_time * 1e-9);
		} else {
			rd.current_velocity[i] = 0.0;
			v_dot[i] = 0.0;
		}
	}

// these two loops could be merged but my brain is slow right now
	for(int i = 0; i< 2; i++) {
		if(rd.pid_enabled) {
			rd.motor_effort[i] = rd.pid[i].computeCommand(	
			    rd.current_velocity[i] - rd.commanded_velocity[i],
			    v_dot[i], last_call_time);
		}
		set_effort(&(rd.channel[i]), rd.motor_effort[i]);
	}
}

// ********************************************************************
// set_motor_effort callback
// ********************************************************************
void set_motor_effort_callback(const void * request_msg, void * response_msg) {
	robotdriver_interfaces__srv__MotorEffort_Request * req_in =
	   (robotdriver_interfaces__srv__MotorEffort_Request *) request_msg; 

	rd.motor_effort[RIGHT_MOTOR] = req_in->right_motor_effort;
	rd.motor_effort[LEFT_MOTOR] = req_in->left_motor_effort;
}

// ********************************************************************
// set_motor_velocity callback
// ********************************************************************
void set_motor_velocity_callback(const void * request_msg, void * response_msg) {
	robotdriver_interfaces__srv__MotorVelocity_Request * req_in =
	   (robotdriver_interfaces__srv__MotorVelocity_Request *) request_msg; 

	rd.pid_enabled = true;

	rd.commanded_velocity[RIGHT_MOTOR] = req_in->right_motor_velocity;
	rd.commanded_velocity[LEFT_MOTOR] = req_in->left_motor_velocity;
}

// ********************************************************************
// set_pid_gains callback
// ********************************************************************
void set_pid_gains_callback(const void * request_msg, void * response_msg) {
	robotdriver_interfaces__srv__PIDGains_Request * req_in =
	   (robotdriver_interfaces__srv__PIDGains_Request *) request_msg; 

	for(int i = 0; i < 2; i++) {
		rd.pid[i].setGains(
			req_in->p, 
			req_in->i, 
			req_in->d, 
			req_in->i_max, 
			req_in->i_min, 
			req_in->anti_windup);
	}
}


// ********************************************************************
// reset_controller callback
// ********************************************************************
void reset_controller_callback(const void * request_msg, void * response_msg) {

	rd.pid_enabled = false;

	for(int i = 0; i < 2; i++) {
		rd.motor_effort[i] = 0.0;
		rd.commanded_velocity[i] = 0.0;
		rd.pid[i].reset();
	}
}


// ********************************************************************
// init_robot_data
// ********************************************************************
void init_robot_data (robot_data * prd) {
	rcl_ret_t ret;

	prd->pid_enabled = false;

	prd->encoder[RIGHT_MOTOR].pio = RIGHT_DECODER_PIO;
	prd->encoder[RIGHT_MOTOR].sm = RIGHT_DECODER_SM;
	prd->encoder[RIGHT_MOTOR].pin_ab = RIGHT_DECODER_PIN;
	prd->encoder[LEFT_MOTOR].pio = LEFT_DECODER_PIO;
	prd->encoder[LEFT_MOTOR].sm = LEFT_DECODER_SM;
	prd->encoder[LEFT_MOTOR].pin_ab = LEFT_DECODER_PIN;

	stdio_init_all();

	for(int i = 0; i < 2; i++) {
		prd->motor_effort[i] = 0.0;
		qe_init(&(prd->encoder[i]));
		prd->current_position[i] = 
			qe_get_count(&(prd->encoder[i]));
	}

	// initialize motor driver chip
	init_chip(CHIP_STBY_PIN);
        init_channel(& (prd->channel[RIGHT_MOTOR]), RIGHT_IN1_PIN,
                        RIGHT_IN2_PIN, RIGHT_PWM_PIN, RIGHT_DIR);
        init_channel(& (prd->channel[LEFT_MOTOR]), LEFT_IN1_PIN,
                        LEFT_IN2_PIN, LEFT_PWM_PIN, LEFT_DIR);
}

// ********************************************************************
// main function
// ********************************************************************
int main()
{
	init_transport();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_timer_t controller_timer;

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

	init_robot_data(&rd);

	// third parameter is total number of subscriptions, timers, services, 
	// clients and guard conditions. 
	// Do not include the number of nodes and publishers.
	init_node_data (&nd, node_name, 6);
	// 2 timers, 4 services (set motor velocity, set motor effort, 
	// set pid gains, reset controller)

	////////////////////
	// init publishers

	ret += rclc_publisher_init_default(
		&MotorStatus_publisher[RIGHT_MOTOR],
		&(nd.node),
		ROSIDL_GET_MSG_TYPE_SUPPORT(
			robotdriver_interfaces, msg, MotorStatus),
        	"right_motor_status");

	ret += rclc_publisher_init_default(
		&MotorStatus_publisher[LEFT_MOTOR],
		&(nd.node),
		ROSIDL_GET_MSG_TYPE_SUPPORT(
			robotdriver_interfaces, msg, MotorStatus),
		"left_motor_status");

	ret += rclc_publisher_init_default(
		&ControllerStatus_publisher[RIGHT_MOTOR],
		&(nd.node),
		ROSIDL_GET_MSG_TYPE_SUPPORT(
			robotdriver_interfaces, msg, ControllerStatus),
        	"right_controller_status");

	ret += rclc_publisher_init_default(
		&ControllerStatus_publisher[LEFT_MOTOR],
		&(nd.node),
		ROSIDL_GET_MSG_TYPE_SUPPORT(
			robotdriver_interfaces, msg, ControllerStatus),
        	"left_controller_status");

	////////////////////
	// init services

	ret += rclc_service_init_default(
		&MotorEffort_service,
        	&(nd.node),
        	ROSIDL_GET_SRV_TYPE_SUPPORT(
			robotdriver_interfaces, srv, MotorEffort),
		MotorEffort_service_name);

	ret += rclc_service_init_default(
		&MotorVelocity_service,
        	&(nd.node),
        	ROSIDL_GET_SRV_TYPE_SUPPORT(
			robotdriver_interfaces, srv, MotorVelocity),
		MotorVelocity_service_name);

	ret += rclc_service_init_default(
		&SetPIDGains_service,
        	&(nd.node),
        	ROSIDL_GET_SRV_TYPE_SUPPORT(
			robotdriver_interfaces, srv, PIDGains),
		SetPIDGains_service_name);

	ret += rclc_service_init_default(
		&ResetController_service,
        	&(nd.node),
        	ROSIDL_GET_SRV_TYPE_SUPPORT(
			std_srvs, srv, Empty),
		ResetController_service_name);

	////////////////////
	// init timers

	ret += rclc_timer_init_default(
		&timer,
		&(nd.support),
		RCL_MS_TO_NS(100),
		timer_callback);

	ret += rclc_timer_init_default(
		&controller_timer,
		&(nd.support),
		RCL_MS_TO_NS(10),
		controller_timer_callback);

	////////////////////
	// add timers

	ret += rclc_executor_add_timer(&(nd.executor), &timer);
	ret += rclc_executor_add_timer(&(nd.executor), &controller_timer);

	////////////////////
	// add services

	ret += rclc_executor_add_service(
		&(nd.executor),
		&MotorEffort_service,
		&MotorEffort_request_msg,
		&MotorEffort_response_msg,
		set_motor_effort_callback);

	ret += rclc_executor_add_service(
		&(nd.executor),
		&MotorVelocity_service,
		&MotorVelocity_request_msg,
		&MotorVelocity_response_msg,
		set_motor_velocity_callback);

	ret += rclc_executor_add_service(
		&(nd.executor),
		&SetPIDGains_service,
		&SetPIDGains_request_msg,
		&SetPIDGains_response_msg,
		set_pid_gains_callback);

	ret += rclc_executor_add_service(
		&(nd.executor),
		&ResetController_service,
		&ResetController_request_msg,
		&ResetController_response_msg,
		reset_controller_callback);

    if (ret != RCL_RET_OK)
    {
        // something went wrong during initialization, don't turn on LED
    	gpio_put(LED_PIN, 0);
        return ret;
    }

    gpio_put(LED_PIN, 1);

    while (true)
    {
        rclc_executor_spin_some(&(nd.executor), RCL_MS_TO_NS(100));
    }
    return 0;
}
