#include "CameraWebServer.h"

#define RXD2 3
#define TXD2 40

bool was_connected = false;
CameraWebServer camera;
WiFiServer server(100);

void FactoryTest(void)
{
    static String readBuff;
    String sendBuff;
    if (Serial2.available())
    {
        char c = Serial2.read();
        readBuff += c;
        if (c == '}')
        {
            if (true == readBuff.equals("{BT_detection}"))
            {
                Serial2.print("{BT_OK}");
                Serial.println("Factory...");
            }
            else if (true == readBuff.equals("{WA_detection}"))
            {
                Serial2.print("{");
                Serial2.print("}");
                Serial.println("Factory...");
            }
            readBuff = "";
        }
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        if (was_connected == false)
        {
            digitalWrite(46, LOW);
            Serial2.print("{WA_OK}");
            was_connected = true;
        }
    }

    else
    {
        static unsigned long Test_time;
        static bool en = true;
        if (millis() - Test_time > 100)
        {
            if (was_connected)
            {
                Serial2.print("{WA_NO}");
                was_connected = false;
            }

            en = !en;
            digitalWrite(46, en ? HIGH : LOW);
            Test_time = millis();
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    camera.Init();
    server.begin();
    delay(100);
    pinMode(46, OUTPUT);
    digitalWrite(46, HIGH);
}

void loop()
{
    FactoryTest();
}