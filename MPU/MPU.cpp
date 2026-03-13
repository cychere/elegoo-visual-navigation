#include "MPU.hpp"

namespace
{
    constexpr float RAD_TO_DEG_F = 57.2957795f;
}

void YawTracker::begin(float gyroZBiasRadPerSec)
{
    yawDeg = 0.0f;
    gyroZBias = gyroZBiasRadPerSec;
    lastUpdateMicros = micros();
}

void YawTracker::reset()
{
    begin();
}

void YawTracker::update(float gyroZRadPerSec)
{
    unsigned long now = micros();
    unsigned long deltaMicros = now - lastUpdateMicros;
    lastUpdateMicros = now;

    float deltaSeconds = deltaMicros / 1000000.0f;
    float correctedGyroZ = gyroZRadPerSec - gyroZBias;
    yawDeg += correctedGyroZ * deltaSeconds * RAD_TO_DEG_F;
}

float YawTracker::getYawDegrees() const
{
    return yawDeg;
}
