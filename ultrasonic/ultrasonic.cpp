#include "ultrasonic.hpp"

void Ultrasonic::init()
{
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
}

uint16_t Ultrasonic::get()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    uint16_t distance = static_cast<uint16_t>(pulseIn(ECHO_PIN, HIGH) / 58);

    return distance;
}
