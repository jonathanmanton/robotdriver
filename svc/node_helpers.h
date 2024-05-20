#ifdef __cplusplus
extern "C" {
#endif

#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

typedef struct node_data {
	rcl_node_t node;
	rcl_allocator_t allocator;
	rclc_support_t support;
	rclc_executor_t executor;
} node_data;

void init_node_data(node_data*, const char*, int);
void init_transport();

#ifdef __cplusplus
}
#endif
