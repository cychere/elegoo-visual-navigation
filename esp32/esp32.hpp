#ifndef ESP32_HPP
#define ESP32_HPP

#include "esp_camera.h"
#include <WiFi.h>

void startCameraStreamServer();

class CameraWebServer
{
public:
    void begin();

private:
    const char *ssid = "Asus";
    const char *password = "cyBer751465!";
};

#endif
