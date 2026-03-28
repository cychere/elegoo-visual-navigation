#include "elegoo.hpp"

void MyServo::set(unsigned int angle)
{
    if (!attached_)
    {
        servo_.attach(Servo_PIN);
        attached_ = true;
    }

    servo_.write(angle);
}
