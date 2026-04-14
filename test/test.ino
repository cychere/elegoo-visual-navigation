#include <Arduino.h>

namespace
{
    constexpr unsigned long Serial_Baud_Rate = 115200;
    constexpr size_t Command_Buffer_Max_Length = 48;

    constexpr uint8_t Motor_PWMA = 5;
    constexpr uint8_t Motor_PWMB = 6;
    constexpr uint8_t Motor_BIN1 = 8;
    constexpr uint8_t Motor_AIN1 = 7;
    constexpr uint8_t Motor_STBY = 3;

    size_t commandBufferLength = 0;
    char commandBuffer[Command_Buffer_Max_Length + 1] = {};
    unsigned long motorStopAtMs = 0;
    bool motorRunning = false;

    int clampSpeed(int speed)
    {
        if (speed < 0)
        {
            return 0;
        }

        if (speed > 255)
        {
            return 255;
        }

        return speed;
    }

    void setMotor(int left, int right)
    {
        int16_t speedL = static_cast<int16_t>(left);
        int16_t speedR = static_cast<int16_t>(right);

        digitalWrite(Motor_STBY, HIGH);

        if (speedL >= 0)
        {
            digitalWrite(Motor_AIN1, LOW);
        }
        else
        {
            digitalWrite(Motor_AIN1, HIGH);
        }

        analogWrite(Motor_PWMA, abs(speedL));

        if (speedR >= 0)
        {
            digitalWrite(Motor_BIN1, LOW);
        }
        else
        {
            digitalWrite(Motor_BIN1, HIGH);
        }

        analogWrite(Motor_PWMB, abs(speedR));
    }

    void stopMotor()
    {
        setMotor(0, 0);
        motorRunning = false;
        motorStopAtMs = 0;
    }

    void initMotor()
    {
        pinMode(Motor_PWMA, OUTPUT);
        pinMode(Motor_PWMB, OUTPUT);
        pinMode(Motor_AIN1, OUTPUT);
        pinMode(Motor_BIN1, OUTPUT);
        pinMode(Motor_STBY, OUTPUT);

        analogWrite(Motor_PWMA, 0);
        analogWrite(Motor_PWMB, 0);
        digitalWrite(Motor_STBY, LOW);
    }

    void runTimedMotor(int left, int right, unsigned long durationSeconds)
    {
        setMotor(left, right);
        motorRunning = true;
        motorStopAtMs = millis() + (durationSeconds * 1000UL);
    }

    bool handleCommand(const char *line)
    {
        char command = '\0';
        int speed = 0;
        unsigned long durationSeconds = 0;

        if (sscanf(line, " %c %d %lu", &command, &speed, &durationSeconds) != 3)
        {
            return false;
        }

        speed = clampSpeed(speed);

        switch (command)
        {
            case 'f':
                runTimedMotor(speed, speed, durationSeconds);
                return true;

            case 'b':
                runTimedMotor(-speed, -speed, durationSeconds);
                return true;

            case 'c':
                runTimedMotor(speed, -speed, durationSeconds);
                return true;

            case 'a':
                runTimedMotor(-speed, speed, durationSeconds);
                return true;

            default:
                return false;
        }
    }

    void handleSerial()
    {
        while (Serial.available() > 0)
        {
            char incoming = static_cast<char>(Serial.read());

            if (incoming == '\r')
            {
                continue;
            }

            if (incoming == '\n')
            {
                commandBuffer[commandBufferLength] = '\0';

                if (commandBufferLength > 0 && !handleCommand(commandBuffer))
                {
                    Serial.println(F("Unknown command"));
                }

                commandBufferLength = 0;
                commandBuffer[0] = '\0';
                continue;
            }

            if (commandBufferLength < Command_Buffer_Max_Length)
            {
                commandBuffer[commandBufferLength++] = incoming;
            }
        }
    }

    void motorTimeout()
    {
        if (motorRunning && static_cast<long>(millis() - motorStopAtMs) >= 0)
        {
            stopMotor();
        }
    }
}

void setup()
{
    Serial.begin(Serial_Baud_Rate);
    Serial.setTimeout(10);

    initMotor();
}

void loop()
{
    handleSerial();
    motorTimeout();
}
