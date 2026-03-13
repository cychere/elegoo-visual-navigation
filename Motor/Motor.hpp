#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class Motor
{
    public:
    enum Direction {Forward, Backward};

    void init();
    void set(Direction direction_L, uint8_t speed_L,
             Direction direction_R, uint8_t speed_R);

    private:

    static constexpr uint8_t Motor_PWMA = 5;
    static constexpr uint8_t Motor_PWMB = 6;
    static constexpr uint8_t Motor_BIN1 = 8;
    static constexpr uint8_t Motor_AIN1 = 7;
    static constexpr uint8_t Motor_STBY = 3;
};

#endif