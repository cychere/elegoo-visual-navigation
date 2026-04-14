#include "esp32.hpp"

constexpr int RXD2 = 3;
constexpr int TXD2 = 40;
constexpr int STATUS_LED_PIN = 46;
constexpr int FACTORY_SERVER_PORT = 100;
constexpr unsigned long WIFI_BLINK_INTERVAL_MS = 100;

bool wasConnected = false;
CameraWebServer camera;
WiFiServer server(FACTORY_SERVER_PORT);

void handleFactoryProbeCommand(const String &command)
{
    if (command == "{BT_detection}")
    {
        Serial2.print("{BT_OK}");
        Serial.println("Factory...");
    }
    else if (command == "{WA_detection}")
    {
        Serial2.print("{}");
        Serial.println("Factory...");
    }
}

void updateFactoryStatus()
{
    static String readBuffer;

    if (Serial2.available())
    {
        char c = Serial2.read();
        readBuffer += c;
        if (c == '}')
        {
            handleFactoryProbeCommand(readBuffer);
            readBuffer = "";
        }
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        if (!wasConnected)
        {
            digitalWrite(STATUS_LED_PIN, LOW);
            Serial2.print("{WA_OK}");
            wasConnected = true;
        }
        return;
    }

    static unsigned long lastBlinkMs;
    static bool ledOn = true;
    if (millis() - lastBlinkMs > WIFI_BLINK_INTERVAL_MS)
    {
        if (wasConnected)
        {
            Serial2.print("{WA_NO}");
            wasConnected = false;
        }

        ledOn = !ledOn;
        digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
        lastBlinkMs = millis();
    }
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    camera.begin();
    server.begin();
    delay(100);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop()
{
    updateFactoryStatus();
}
