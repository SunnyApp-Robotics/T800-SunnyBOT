#include "microros.h"
#include "freertos.h"

//subcribers messages
common__msg__Wheels msg_controlBox;
//publishers
rcl_publisher_t pub_wheels;


//publishers messages
common__msg__Wheels msg_wheels;

//variables
int device_id;		  //mac address;
uint8_t position = 3; // 1.frente 2.atras 3.control 0.sin especificar

//motors
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;
const motor_t motorR_1 = {.timer1 = &htim2, .timer2 = &htim2, .channel1 = TIM_CHANNEL_3, .channel2 = TIM_CHANNEL_4};
const motor_t motorL_2 = {.timer1 = &htim2, .timer2 = &htim11, .channel1 = TIM_CHANNEL_1, .channel2 = TIM_CHANNEL_1};
const motor_t motorL_1 = {.timer1 = &htim8, .timer2 = &htim8, .channel1 = TIM_CHANNEL_1, .channel2 = TIM_CHANNEL_2};
const motor_t motorR_2 = {.timer1 = &htim9, .timer2 = &htim9, .channel1 = TIM_CHANNEL_1, .channel2 = TIM_CHANNEL_2};
motor_t motors[4] = {motorL_1, motorL_2, motorR_1, motorR_2};
encoder_t encoderR = {.timer = &htim3, .countTicks = 0};
encoder_t encoderL = {.timer = &htim4, .countTicks = 0};
//callbacks
void wheels_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		common__msg__Wheels msgWheels;
		updateTicks(&encoderR);
		updateTicks(&encoderL);
		msgWheels.param[0] = getDistance(&encoderR);
		msgWheels.param[1] = getDistance(&encoderL);
		printf("distR %f rad,distL %f rad, \n", msgWheels.param[0], msgWheels.param[1]);
		publish_sen_wheels(&msgWheels);
		//printf("free heap %d", xPortGetFreeHeapSize());
	}
}


void subs_controlBox_callback(const void *msgin)
{
	int state = HAL_GPIO_ReadPin(GPIOB, LD2_Pin) ? GPIO_PIN_RESET:GPIO_PIN_SET;
	HAL_GPIO_WritePin(GPIOB, LD2_Pin , state);
	const common__msg__Wheels *msg = (const common__msg__Wheels *)msgin;
	printf("velR %f, velL %f \n", msg->param[0], msg->param[1]);
	moveMotor(motorR_1, msg->param[0]);
	moveMotor(motorR_2, msg->param[0]);
	moveMotor(motorL_1, msg->param[1]);
	moveMotor(motorL_2, msg->param[1]);
	//msg.param[0] /vel_right msg.param[1] /vel_left
}

//functions initialize
void initStringMessages(rosidl_runtime_c__String *data, int len)
{
	char msg_ping_buffer[len];
	data->data = msg_ping_buffer;
	data->capacity = len;
}
//initialize messages
void initMessages()
{
	device_id = rand();
	initStringMessages(&msg_wheels.header.frame_id, (int)STRING_BUFFER_LEN);
	initStringMessages(&msg_controlBox.header.frame_id, (int)STRING_BUFFER_LEN);
	startMotors(&motors[0], 4);
	startEncoders(&encoderR, 1);
	startEncoders(&encoderL, 1);
}

void publish_sen_wheels(const common__msg__Wheels *msgin)
{
	const common__msg__Wheels *msg = msgin;

	msg_wheels.param[0] = msg->param[0];
	msg_wheels.param[1] = msg->param[1];
	sprintf(msg_wheels.header.frame_id.data, "%d_%d", device_id, position);
	msg_wheels.header.frame_id.size = strlen(msg_wheels.header.frame_id.data);
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	msg_wheels.header.stamp.sec = ts.tv_sec;
	msg_wheels.header.stamp.nanosec = ts.tv_nsec;
	RCSOFTCHECK(rcl_publish(&pub_wheels, (const void *)&msg_wheels, NULL));
}
