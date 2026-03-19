#include "elegoo.hpp"

void Motor::init()
{
    pinMode(Motor_PWMA, OUTPUT);
    pinMode(Motor_PWMB, OUTPUT);
    pinMode(Motor_AIN1, OUTPUT);
    pinMode(Motor_BIN1, OUTPUT);
    pinMode(Motor_STBY, OUTPUT);

    analogWrite(Motor_PWMA, 0);
    analogWrite(Motor_PWMB, 0);
    digitalWrite(Motor_STBY, LOW);
}

void Motor::set(Direction direction_L, uint8_t speed_L,
                Direction direction_R, uint8_t speed_R)
{
    digitalWrite(Motor_STBY, HIGH);

    switch (direction_L)
    {
        case Forward:
            digitalWrite(Motor_AIN1, LOW);
            analogWrite(Motor_PWMA, speed_L);
            break;
        case Backward:
            digitalWrite(Motor_AIN1, HIGH);
            analogWrite(Motor_PWMA, speed_L);
            break;
        default:
            analogWrite(Motor_PWMA, 0);
            digitalWrite(Motor_STBY, LOW);
            break;
    }

    switch (direction_R)
    {
        case Forward:
            digitalWrite(Motor_BIN1, LOW);
            analogWrite(Motor_PWMB, speed_R);
            break;
        case Backward:
            digitalWrite(Motor_BIN1, HIGH);
            analogWrite(Motor_PWMB, speed_R);
            break;
        default:
            analogWrite(Motor_PWMB, 0);
            digitalWrite(Motor_STBY, LOW);
            break;
    }
}
