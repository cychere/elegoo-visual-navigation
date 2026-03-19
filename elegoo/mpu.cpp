#include "elegoo.hpp"

namespace
{
    constexpr float RAD_TO_DEG_F = 57.2957795f;
}

void YawTracker::init(float gyroBias)
{
    yawDeg = 0.0f;
    bias = gyroBias;
    lastUpdate = micros();
}

void YawTracker::update(float gyroZ)
{
    unsigned long now = micros();
    unsigned long deltaUs = now - lastUpdate;
    lastUpdate = now;

    float deltaSeconds = deltaUs / 1000000.0f;
    float correctedGyroZ = gyroZ - bias;
    yawDeg += correctedGyroZ * deltaSeconds * RAD_TO_DEG_F;
}

float YawTracker::getYaw() const
{
    return yawDeg;
}
