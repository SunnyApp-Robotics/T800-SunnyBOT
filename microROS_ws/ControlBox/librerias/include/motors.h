#ifndef MOTORS_H_
#define MOTORS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f7xx_hal.h"
#define MAX_VELOCITY 28.0
#define MAX_PWM_COMPARE 1000
#define TICKS4REVOLUTION 40.0
#define PI 3.14159625
#define ABS(a) ({ a >= 0 ? a : -a; })
#define POSITIVE(a) ({ a >= 0 ? 1 : 0; })

    typedef struct MOTOR
    {
        TIM_HandleTypeDef *timer1;
        TIM_HandleTypeDef *timer2;
        uint32_t channel1;
        uint32_t channel2;
    } motor_t;

    typedef struct ENCODER
    {
        TIM_HandleTypeDef *timer;
        int countTicks;
    } encoder_t;

    void startMotors(motor_t *motor, uint8_t len);
    void startEncoders(encoder_t *encoders, uint8_t len);
    void moveMotor(motor_t motor, float velocity);
    float getDistance(encoder_t *encoder);
    void updateTicks(encoder_t *encoder);

    extern encoder_t encoderR;
    extern encoder_t encoderL;

#ifdef __cplusplus
    extern "C"
}
#endif

#endif