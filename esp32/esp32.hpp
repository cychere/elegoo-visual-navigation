#ifndef ESP32_HPP
#define ESP32_HPP

#include "esp_system.h"
#include "esp_camera.h"
#include <WiFi.h>

class CameraWebServer
{
    public:
        void Init(void);

    private:
        const char* ssid = "Asus";
        const char* password = "cyBer751465!";
};

#endif