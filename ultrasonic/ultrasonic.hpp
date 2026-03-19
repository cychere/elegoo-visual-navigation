#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

#include <Arduino.h>

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
