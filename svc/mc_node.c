#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#include "node_helpers.h"

node_data nd;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

int main()
{
	init_transport();
	init_node_data(&nd, "pico node");

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

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&(nd.executor), RCL_MS_TO_NS(100));
    }
    return 0;
}
