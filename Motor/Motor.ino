#include <avr/wdt.h>
#include "Motor.hpp"

Motor motor;

static unsigned long last_cmd_ms = 0;
static const unsigned long cmd_timeout_ms = 500;

static void applySpeed(int left, int right)
{
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    Motor::Direction dirL = (left >= 0) ? Motor::Forward : Motor::Backward;
    Motor::Direction dirR = (right >= 0) ? Motor::Forward : Motor::Backward;

    uint8_t speedL = static_cast<uint8_t>(abs(left));
    uint8_t speedR = static_cast<uint8_t>(abs(right));

    motor.set(dirL, speedL, dirR, speedR);
}

static bool parseAndApply(const String &line)
{
    int left = 0;
    int right = 0;

    if (sscanf(line.c_str(), "%d %d", &left, &right) == 2)
    {
        applySpeed(left, right);
        last_cmd_ms = millis();
        return true;
    }
    return false;
}

void setup()
{
    motor.init();
    Serial.begin(115200);
    Serial.setTimeout(10);
    last_cmd_ms = millis();
}

void loop()
{
    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0)
        {
            parseAndApply(line);
        }
    }

    if (millis() - last_cmd_ms > cmd_timeout_ms)
    {
        applySpeed(0, 0);
    }
}
