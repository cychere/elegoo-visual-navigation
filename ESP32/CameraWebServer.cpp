#include "esp_system.h"
#include "esp_camera.h"
#include <WiFi.h>

void startCameraServer();

class CameraWebServer
{
    public:
        void Init(void);

    private:
        const char* ssid = "Asus";
        const char* password = "cyBer751465!";
};

void CameraWebServer::Init(void)
{
    Serial.setDebugOutput(true);
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 11;
    config.pin_d1 = 9;
    config.pin_d2 = 8;
    config.pin_d3 = 10;
    config.pin_d4 = 12;
    config.pin_d5 = 18;
    config.pin_d6 = 17;
    config.pin_d7 = 16;
    config.pin_xclk = 15;
    config.pin_pclk = 13;
    config.pin_vsync = 6;
    config.pin_href = 7;
    config.pin_sscb_sda = 4;
    config.pin_sscb_scl = 5;
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_SVGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 10;
    config.fb_count = 2;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();

    s->set_gainceiling(s, (gainceiling_t)GAINCEILING_16X);
    s->set_aec2(s, 0);
    s->set_exposure_ctrl(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_awb_gain(s, 1);

    delay(800);

    uint64_t chipid = ESP.getEfuseMac();
    char string[10];
    sprintf(string, "%04X", (uint16_t)(chipid >> 32));
    String mac0_default = String(string);
    sprintf(string, "%08X", (uint32_t)chipid);
    String mac1_default = String(string);
    String url = ssid + mac0_default + mac1_default;
    const char *mac_default = url.c_str();

    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    WiFi.begin(ssid, password);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(300);
        Serial.print(".");
        if (millis() - t0 > 20000)
        {
          Serial.println("\nWiFi connect timeout");
          return;
        }
    }

    Serial.println("\nWiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    startCameraServer();

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("/stream' to connect");
}