#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <common/msg/wheels.h>
#include <std_msgs/msg/header.h>
#include "rosidl_runtime_c/string.h"
#include <string.h>

#define RCCHECK(fn)                                                                      \
	{                                                                                    \
		rcl_ret_t temp_rc = fn;                                                          \
		if ((temp_rc != RCL_RET_OK))                                                     \
		{                                                                                \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			vTaskDelete(NULL);                                                           \
		}                                                                                \
	}
#define RCSOFTCHECK(fn)                                                                    \
	{                                                                                      \
		rcl_ret_t temp_rc = fn;                                                            \
		if ((temp_rc != RCL_RET_OK))                                                       \
		{                                                                                  \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                  \
	}

#define STRING_BUFFER_LEN 15

//subcribers objects
extern rcl_subscription_t subs_controlBox;

//subcribers messages
extern common__msg__Wheels msg_controlBox;
#define con_subscriber_support ROSIDL_GET_MSG_TYPE_SUPPORT(common, msg, Wheels)
//publishers objects
extern rcl_publisher_t pub_wheels;

//publishers messages
extern common__msg__Wheels msg_wheels;

//define type support messages publishers
#define wheels_publisher_support ROSIDL_GET_MSG_TYPE_SUPPORT(common, msg, Wheels)

//callbacks
void wheels_callback(rcl_timer_t *timer, int64_t last_call_time);
void subs_controlBox_callback(const void *msgin);

//functions for initialize
void initStringMessages(rosidl_runtime_c__String *data, int len);
void initMessages();

//function to publish
void publish_sen_wheels(const common__msg__Wheels *msg);
