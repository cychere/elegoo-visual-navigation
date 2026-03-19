#ifndef SERVO_HPP
#define SERVO_HPP

#include <Arduino.h>
#include <Servo.h>

class MyServo
{
    public:
        void set(unsigned int angle);

    private:
        static constexpr uint8_t Min_Angle = 0;
        static constexpr uint8_t Max_Angle = 180;
        static constexpr uint8_t Servo_PIN = 10;

        Servo servo_;
        bool attached_ = false;
};

#endif