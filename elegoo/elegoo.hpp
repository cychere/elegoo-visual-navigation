#ifndef ELEGOO_HPP
#define ELEGOO_HPP

#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

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

class YawTracker
{
    public:
        void init(float gyroBias = 0.0f);
        void update(float gyroZ);
        float getYaw() const;

    private:
        float yawDeg = 0.0f;
        float bias = 0.0f;
        unsigned long lastUpdate = 0;
};

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

class Ultrasonic
{
    public:
        void init();
        uint16_t get();

    private:
        static constexpr uint8_t TRIG_PIN = 13;
        static constexpr uint8_t ECHO_PIN = 12;
        static constexpr uint16_t MAX_DISTANCE = 400; // cm
};

#endif
