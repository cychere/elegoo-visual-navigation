#ifndef ESP32_HPP
#define ESP32_HPP

#include "esp_camera.h"
#include <WiFi.h>

void startCameraStreamServer();

enum class WifiMode
{
    Station,
    AccessPoint
};

namespace WifiSettings
{
constexpr WifiMode mode = WifiMode::AccessPoint;

constexpr char stationSsid[] = "";
constexpr char stationPassword[] = "";

constexpr char accessPointSsid[] = "ElegooCamera";
constexpr char accessPointPassword[] = "elegoo1234";
constexpr uint8_t accessPointIp[] = {192, 168, 4, 1};
constexpr uint8_t accessPointSubnet[] = {255, 255, 255, 0};
}

class CameraWebServer
{
public:
    void begin();
};

#endif
