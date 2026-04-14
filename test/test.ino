#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

namespace
{
    constexpr unsigned long Serial_Baud_Rate = 115200;
    constexpr unsigned long Loop_Frequency_Hz = 60;
    constexpr unsigned long Loop_Period_Us = 1000000UL / Loop_Frequency_Hz;
    constexpr unsigned long Speed_Window_Us = 1000000UL;
    constexpr size_t Speed_Window_Sample_Capacity = static_cast<size_t>(Loop_Frequency_Hz) + 5;
    constexpr int Gyro_Calibration_Samples = 2000;
    constexpr int Yaw_Print_Precision = 6;
    constexpr size_t Command_Buffer_Max_Length = 48;

    constexpr uint8_t Motor_PWMA = 5;
    constexpr uint8_t Motor_PWMB = 6;
    constexpr uint8_t Motor_BIN1 = 8;
    constexpr uint8_t Motor_AIN1 = 7;
    constexpr uint8_t Motor_STBY = 3;
    constexpr uint8_t Ultrasonic_TRIG_PIN = 13;
    constexpr uint8_t Ultrasonic_ECHO_PIN = 12;
    constexpr uint16_t Ultrasonic_MAX_DISTANCE = 400; // cm
    constexpr float RAD_TO_DEG_F = 57.2957795f;

    Adafruit_MPU6050 mpu;
    size_t commandBufferLength = 0;
    char commandBuffer[Command_Buffer_Max_Length + 1] = {};
    unsigned long motorStopAtMs = 0;
    bool motorRunning = false;
    float gyroBias = 0.0f;
    float yawDeg = 0.0f;
    float yawAngularSpeedDegPerSec = 0.0f;
    float forwardSpeedCmPerSec = 0.0f;
    unsigned long yawSampleTimesUs[Speed_Window_Sample_Capacity] = {};
    float yawSampleDeg[Speed_Window_Sample_Capacity] = {};
    size_t yawSampleStart = 0;
    size_t yawSampleCount = 0;
    unsigned long distanceSampleTimesUs[Speed_Window_Sample_Capacity] = {};
    uint16_t distanceSampleCm[Speed_Window_Sample_Capacity] = {};
    size_t distanceSampleStart = 0;
    size_t distanceSampleCount = 0;
    unsigned long lastYawUpdateUs = 0;
    unsigned long nextLoopUs = 0;

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

    void initUltrasonic()
    {
        pinMode(Ultrasonic_ECHO_PIN, INPUT);
        pinMode(Ultrasonic_TRIG_PIN, OUTPUT);
    }

    uint16_t getDistanceCm()
    {
        constexpr unsigned long Echo_Timeout_Us = Ultrasonic_MAX_DISTANCE * 58UL * 2UL;

        digitalWrite(Ultrasonic_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(Ultrasonic_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(Ultrasonic_TRIG_PIN, LOW);

        unsigned long pulseDurationUs = pulseIn(Ultrasonic_ECHO_PIN, HIGH, Echo_Timeout_Us);
        return static_cast<uint16_t>(pulseDurationUs / 58UL);
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
        yawDeg = 0.0f;
        yawAngularSpeedDegPerSec = 0.0f;
        yawSampleStart = 0;
        yawSampleCount = 0;
        lastYawUpdateUs = micros();

        Serial.print(F("Gyro Z bias: "));
        Serial.println(gyroBias, 6);

        return true;
    }

    size_t nextSampleIndex(size_t start, size_t count)
    {
        return (start + count) % Speed_Window_Sample_Capacity;
    }

    void appendYawSample(unsigned long now, float yaw)
    {
        size_t sampleIndex = 0;

        if (yawSampleCount < Speed_Window_Sample_Capacity)
        {
            sampleIndex = nextSampleIndex(yawSampleStart, yawSampleCount);
            ++yawSampleCount;
        }
        else
        {
            sampleIndex = yawSampleStart;
            yawSampleStart = nextSampleIndex(yawSampleStart, 1);
        }

        yawSampleTimesUs[sampleIndex] = now;
        yawSampleDeg[sampleIndex] = yaw;

        while (yawSampleCount > 1 && now - yawSampleTimesUs[yawSampleStart] > Speed_Window_Us)
        {
            yawSampleStart = nextSampleIndex(yawSampleStart, 1);
            --yawSampleCount;
        }
    }

    void appendDistanceSample(unsigned long now, uint16_t distanceCm)
    {
        size_t sampleIndex = 0;

        if (distanceSampleCount < Speed_Window_Sample_Capacity)
        {
            sampleIndex = nextSampleIndex(distanceSampleStart, distanceSampleCount);
            ++distanceSampleCount;
        }
        else
        {
            sampleIndex = distanceSampleStart;
            distanceSampleStart = nextSampleIndex(distanceSampleStart, 1);
        }

        distanceSampleTimesUs[sampleIndex] = now;
        distanceSampleCm[sampleIndex] = distanceCm;

        while (distanceSampleCount > 1 && now - distanceSampleTimesUs[distanceSampleStart] > Speed_Window_Us)
        {
            distanceSampleStart = nextSampleIndex(distanceSampleStart, 1);
            --distanceSampleCount;
        }
    }

    void clearDistanceSamples()
    {
        distanceSampleStart = 0;
        distanceSampleCount = 0;
        forwardSpeedCmPerSec = 0.0f;
    }

    void refreshYaw()
    {
        sensors_event_t acceleration, gyro, temperature;
        mpu.getEvent(&acceleration, &gyro, &temperature);

        unsigned long now = micros();
        unsigned long deltaUs = now - lastYawUpdateUs;
        lastYawUpdateUs = now;

        float deltaSeconds = deltaUs / 1000000.0f;
        float correctedGyroZ = gyro.gyro.z - gyroBias;
        yawDeg += correctedGyroZ * RAD_TO_DEG_F * deltaSeconds;

        appendYawSample(now, yawDeg);

        if (yawSampleCount < 2)
        {
            yawAngularSpeedDegPerSec = 0.0f;
            return;
        }

        unsigned long sampleDeltaUs = now - yawSampleTimesUs[yawSampleStart];
        if (sampleDeltaUs == 0)
        {
            yawAngularSpeedDegPerSec = 0.0f;
            return;
        }

        float sampleDeltaSeconds = sampleDeltaUs / 1000000.0f;
        yawAngularSpeedDegPerSec = (yawDeg - yawSampleDeg[yawSampleStart]) / sampleDeltaSeconds;
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
        Serial.print(yawDeg, Yaw_Print_Precision);
        Serial.print(F(" deg "));
        Serial.print(distanceCm);
        Serial.print(F(" cm "));
        Serial.print(yawAngularSpeedDegPerSec, Yaw_Print_Precision);
        Serial.print(F(" deg/s "));
        Serial.print(forwardSpeedCmPerSec, Yaw_Print_Precision);
        Serial.println(F(" cm/s"));
    }

    void refreshForwardSpeed(uint16_t distanceCm)
    {
        unsigned long now = micros();

        if (distanceCm == 0)
        {
            clearDistanceSamples();
            return;
        }

        appendDistanceSample(now, distanceCm);

        if (distanceSampleCount < 2)
        {
            forwardSpeedCmPerSec = 0.0f;
            return;
        }

        unsigned long deltaUs = now - distanceSampleTimesUs[distanceSampleStart];
        if (deltaUs == 0)
        {
            forwardSpeedCmPerSec = 0.0f;
            return;
        }

        float deltaSeconds = deltaUs / 1000000.0f;
        forwardSpeedCmPerSec = (static_cast<float>(distanceSampleCm[distanceSampleStart]) - distanceCm) / deltaSeconds;
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
    initUltrasonic();
    initMpu();
    nextLoopUs = micros() + Loop_Period_Us;
}

void loop()
{
    delayUntilNextLoop();
    refreshYaw();
    uint16_t distanceCm = getDistanceCm();
    refreshForwardSpeed(distanceCm);
    sendReading(distanceCm);
    handleSerial();
    motorTimeout();
}
