#include "elegoo.hpp"

namespace
{
    constexpr unsigned long Serial_Baud_Rate = 115200;
    constexpr unsigned long Command_Timeout_Ms = 500;
    constexpr unsigned long Loop_Frequency_Hz = 60;
    constexpr unsigned long Loop_Period_Us = 1000000UL / Loop_Frequency_Hz;
    constexpr int Gyro_Calibration_Samples = 200;
    constexpr int Yaw_Print_Precision = 6;
    constexpr size_t Command_Buffer_Max_Length = 48;

    Motor motor;
    MyServo servo;
    Adafruit_MPU6050 mpu;
    Ultrasonic ultrasonic;
    YawTracker yawTracker;

    float gyroBias = 0.0f;
    unsigned long nextLoopUs = 0;
    unsigned long lastMotorCommandMs = 0;
    size_t commandBufferLength = 0;
    char commandBuffer[Command_Buffer_Max_Length + 1] = {};

    void setMotor(int left, int right)
    {
        uint16_t speedL = static_cast<uint16_t>(left);
        uint16_t speedR = static_cast<uint16_t>(right);
        motor.set(speedL, speedR);
    }

    float calibrateGyro()
    {
        float biasSum = 0.0f;

        for (int sample = 0; sample < Gyro_Calibration_Samples; ++sample)
        {
            sensors_event_t acceleration, gyro, temperature;
            mpu.getEvent(&acceleration, &gyro, &temperature);
            biasSum += gyro.gyro.z;
            delay(5);
        }

        return biasSum / Gyro_Calibration_Samples;
    }

    bool initMpu()
    {
        mpu.begin();
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

        delay(100);

        gyroBias = calibrateGyro();
        yawTracker.init(gyroBias);

        Serial.print(F("Gyro Z bias: "));
        Serial.println(gyroBias, 6);

        return true;
    }

    void refreshYaw()
    {
        sensors_event_t acceleration, gyro, temperature;
        mpu.getEvent(&acceleration, &gyro, &temperature);
        yawTracker.update(gyro.gyro.z);
    }

    void delayUntilNextLoop()
    {
        long remainingUs = static_cast<long>(nextLoopUs - micros());

        if (remainingUs > 0)
        {
            unsigned long remaining = static_cast<unsigned long>(remainingUs);
            delay(remaining / 1000UL);
            delayMicroseconds(static_cast<unsigned int>(remaining % 1000UL));
        }

        nextLoopUs += Loop_Period_Us;

        if (static_cast<long>(micros() - nextLoopUs) >= 0)
        {
            nextLoopUs = micros() + Loop_Period_Us;
        }
    }

    void sendReading(uint16_t distanceCm)
    {
        Serial.print(F("SENSOR "));
        Serial.print(yawTracker.getYaw(), Yaw_Print_Precision);
        Serial.print(' ');
        Serial.println(distanceCm);
    }

    bool handleCommand(const char *line)
    {
        int left = 0;
        int right = 0;
        int angle = 0;

        if (sscanf(line, "MOTOR %d %d", &left, &right) == 2)
        {
            setMotor(left, right);
            lastMotorCommandMs = millis();
            return true;
        }

        if (sscanf(line, "SERVO %d", &angle) == 1)
        {
            servo.set(angle);
            return true;
        }

        return false;
    }

    void handleSerial()
    {
        while (Serial.available() > 0)
        {
            char incoming = static_cast<char>(Serial.read());

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
        if (millis() - lastMotorCommandMs > Command_Timeout_Ms)
        {
            setMotor(0, 0);
        }
    }
}

void setup()
{
    Serial.begin(Serial_Baud_Rate);
    Serial.setTimeout(10);

    motor.init();
    ultrasonic.init();
    lastMotorCommandMs = millis();
    initMpu();
    nextLoopUs = micros() + Loop_Period_Us;
}

void loop()
{
    delayUntilNextLoop();
    refreshYaw();
    uint16_t distanceCm = ultrasonic.get();
    sendReading(distanceCm);
    handleSerial();
    motorTimeout();
}
