#include "elegoo.hpp"

void Ultrasonic::init()
{
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
}

uint16_t Ultrasonic::get()
{
    constexpr unsigned long Echo_Timeout_Us = MAX_DISTANCE * 58UL * 2UL;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long pulseDurationUs = pulseIn(ECHO_PIN, HIGH, Echo_Timeout_Us);
    uint16_t distance = static_cast<uint16_t>(pulseDurationUs / 58UL);

    return distance;
}
