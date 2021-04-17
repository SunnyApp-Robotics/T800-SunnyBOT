//librarys include
#include "include/microros.h"
#include "include/freertos.h"

//subcribers objects
rcl_subscription_t subs_controlBox;

//subcribers messages
common__msg__Wheels msg_controlBox;

//publishers objects
rcl_publisher_t pub_wheels;


//main
void appMain(void *arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node,"robotTecnoparque","", &support));

	// create subscribers
	RCCHECK(rclc_subscription_init_default(&subs_controlBox, &node,
										   con_subscriber_support, "/robot/cmd_wheels"));

	// create publishers
	RCCHECK(rclc_publisher_init_best_effort(&pub_wheels, &node,
										wheels_publisher_support, "/robot/enc_wheels"));
											
	// create timers
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), 
	wheels_callback));
	

	//init messages
	initMessages();

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subs_controlBox, 
	&msg_controlBox, &subs_controlBox_callback, ON_NEW_DATA));

	
	while (1)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&subs_controlBox, &node));
	RCCHECK(rcl_publisher_fini(&pub_wheels, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}
