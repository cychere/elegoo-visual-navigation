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

void Motor::set(uint16_t speed_L, uint16_t speed_R)
{
    digitalWrite(Motor_STBY, HIGH);

    if (speed_L >= 0) digitalWrite(Motor_AIN1, LOW);
    else digitalWrite(Motor_AIN1, HIGH);
    analogWrite(Motor_PWMA, speed_L);

    if (speed_R >= 0) digitalWrite(Motor_BIN1, LOW);
    else digitalWrite(Motor_BIN1, HIGH);
    analogWrite(Motor_PWMB, speed_R);
}
