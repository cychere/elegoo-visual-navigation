#include "elegoo.hpp"

void MyServo::set(unsigned int angle)
{
    servo_.attach(Servo_PIN);

    uint8_t clamped_angle = static_cast<uint8_t>(constrain(static_cast<int>(angle), Min_Angle, Max_Angle));

    servo_.write(clamped_angle);

    delay(500);

    servo_.detach();
}
