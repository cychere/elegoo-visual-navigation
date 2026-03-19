#include <avr/wdt.h>
#include "ultrasonic.hpp"

Ultrasonic sonic;

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(10);
    sonic.init();
}

void loop()
{
    if (!Serial.available())
    {
        return;
    }

    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "GET")
    {
        Serial.println(sonic.get());
    }
}
