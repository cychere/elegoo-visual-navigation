#include "elegoo.hpp"

void MyServo::set(unsigned int angle)
{
    if (!attached_)
    {
        servo_.attach(Servo_PIN);
        attached_ = true;
    }

    uint8_t clamped_angle = static_cast<uint8_t>(constrain(static_cast<int>(angle), Min_Angle, Max_Angle));

    servo_.write(clamped_angle);
}
