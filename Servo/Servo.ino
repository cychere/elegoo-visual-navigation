#include "Servo.hpp"

MyServo servo;

static bool parseAndApply(const String &line)
{
    int angle = 0;

    if (sscanf(line.c_str(), "%d", &angle) == 1)
    {
        servo.set(angle);
        return true;
    }
    return false;
}

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(10);
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
}