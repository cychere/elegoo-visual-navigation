#include "esp32.hpp"

constexpr int STATUS_LED_PIN = 46;
constexpr int STATUS_LED_ON_LEVEL = LOW;
constexpr int STATUS_LED_OFF_LEVEL = HIGH;

CameraWebServer camera;

void updateStatusLed()
{
    bool networkReady = WifiSettings::mode == WifiMode::AccessPoint || WiFi.status() == WL_CONNECTED;
    digitalWrite(STATUS_LED_PIN, networkReady ? STATUS_LED_ON_LEVEL : STATUS_LED_OFF_LEVEL);
}

void setup()
{
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF_LEVEL);
    camera.begin();
}

void loop()
{
    updateStatusLed();
}
