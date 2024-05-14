#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "robotdriver_interfaces/srv/motor_effort.h"
#include "robotdriver_interfaces/srv/motor_velocity.h"
#include "robotdriver_interfaces/srv/pid_gains.h"
#include "robotdriver_interfaces/msg/controller_status.h"
#include "robotdriver_interfaces/msg/motor_status.h"
#include <std_srvs/srv/empty.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "node_helpers.h"

const uint LED_PIN = 25;

node_data nd;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t RightMotorStatus_publisher;
rcl_publisher_t LeftMotorStatus_publisher;
rcl_publisher_t RightControllerStatus_publisher;
rcl_publisher_t LeftControllerStatus_publisher;
robotdriver_interfaces__msg__MotorStatus RightMotorStatus_msg;
robotdriver_interfaces__msg__MotorStatus LeftMotorStatus_msg;
robotdriver_interfaces__msg__ControllerStatus RightControllerStatus_msg;
robotdriver_interfaces__msg__ControllerStatus LeftControllerStatus_msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
	RightMotorStatus_msg.motor_position = msg.data + 6;
	RightMotorStatus_msg.motor_velocity = msg.data / 3.14;
	RightMotorStatus_msg.motor_effort = msg.data * 3.14;

	ret += rcl_publish(&RightMotorStatus_publisher, &RightMotorStatus_msg, NULL);
/*
	rcl_publish(&RightControllerStatus_publisher, &RightControllerStatus_msg, NULL);
	rcl_publish(&LeftMotorStatus_publisher, &LeftMotorStatus_msg, NULL);
	rcl_publish(&LeftControllerStatus_publisher, &LeftControllerStatus_msg, NULL);
*/
    msg.data++;
}

int main()
{
	init_transport();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

	init_node_data (&nd, "pico_node");

    rclc_publisher_init_default(
        &RightMotorStatus_publisher,
        &(nd.node),
        ROSIDL_GET_MSG_TYPE_SUPPORT(robotdriver_interfaces, msg, MotorStatus),
        "right_motor_status");

/*
    rclc_publisher_init_default(
        &LeftMotorStatus_publisher,
        &(nd.node),
        ROSIDL_GET_MSG_TYPE_SUPPORT(robotdriver_interfaces, msg, MotorStatus),
        "left_motor_status");
*/

    rclc_publisher_init_default(
        &publisher,
        &(nd.node),
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_timer_init_default(
        &timer,
        &(nd.support),
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_add_timer(&(nd.executor), &timer);

    gpio_put(LED_PIN, 1);

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&(nd.executor), RCL_MS_TO_NS(100));
    }
    return 0;
}
