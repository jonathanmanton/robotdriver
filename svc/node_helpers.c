#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "node_helpers.h"

void init_node_data(node_data* nd, char* node_name) {
	nd->allocator = rcl_get_default_allocator();
	rclc_support_init(&(nd->support), 0, NULL, &(nd->allocator));

	rclc_node_init_default(&(nd->node), node_name, "", &(nd->support));
	rclc_executor_init(&(nd->executor), &(nd->support.context), 
		1, &(nd->allocator));
}

void init_transport() {
	rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);
}

