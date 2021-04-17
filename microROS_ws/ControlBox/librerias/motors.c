#include "motors.h"

void startMotors(motor_t *motors, uint8_t len)
{
    for (int i = 0; i < len; i++)
    {

        HAL_TIM_PWM_Start(motors[i].timer1, motors[i].channel1);
        HAL_TIM_PWM_Start(motors[i].timer2, motors[i].channel2);
    }
}

void startEncoders(encoder_t *encoders, uint8_t len)
{
    for (int i = 0; i < len; i++)
    {

        HAL_TIM_Encoder_Start(encoders[i].timer, TIM_CHANNEL_ALL);
    }
}

void moveMotor(motor_t motor, float velocity)
{
    int pwm_value = (ABS(velocity) / MAX_VELOCITY) * 1000;
    pwm_value = pwm_value > MAX_PWM_COMPARE ? 1000 : pwm_value;
    if (POSITIVE(velocity))
    {
        __HAL_TIM_SET_COMPARE(motor.timer1, motor.channel1, pwm_value);
        __HAL_TIM_SET_COMPARE(motor.timer2, motor.channel2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(motor.timer2, motor.channel2, pwm_value);
        __HAL_TIM_SET_COMPARE(motor.timer1, motor.channel1, 0);
    }
}

float getDistance(encoder_t *encoder)
{
    float Distance = (float)(encoder->countTicks)* PI/TICKS4REVOLUTION;
    return Distance;
}

void updateTicks(encoder_t *encoder)
{
    encoder->countTicks=(int16_t)__HAL_TIM_GET_COUNTER(encoder->timer)>>1; //four times more of ticks because of the timer count in both edges and count both channels
    __HAL_TIM_SET_COUNTER(encoder->timer,0);
}