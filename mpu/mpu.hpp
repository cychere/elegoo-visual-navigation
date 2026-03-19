#ifndef MPU_HPP
#define MPU_HPP

#include <Arduino.h>

class YawTracker
{
    public:
        void begin(float gyroZBiasRadPerSec = 0.0f);
        void reset();
        void update(float gyroZRadPerSec);
        float getYawDegrees() const;

    private:
        float yawDeg = 0.0f;
        float gyroZBias = 0.0f;
        unsigned long lastUpdateMicros = 0;
};

#endif
