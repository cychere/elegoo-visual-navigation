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

constexpr char stationSsid[] = "Asus";
constexpr char stationPassword[] = "cyBer751465!";

constexpr char accessPointSsid[] = "ElegooCamera";
constexpr char accessPointPassword[] = "elegoo1234";
}

class CameraWebServer
{
public:
    void begin();
};

#endif
