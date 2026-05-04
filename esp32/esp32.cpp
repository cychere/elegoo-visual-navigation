#include "esp32.hpp"

constexpr uint32_t XCLK_FREQ_HZ = 20000000;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
constexpr uint16_t WIFI_CONNECT_RETRY_DELAY_MS = 300;

camera_config_t makeCameraConfig()
{
    camera_config_t config = {};
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
    config.pin_sccb_sda = 4;
    config.pin_sccb_scl = 5;
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.frame_size = FRAMESIZE_SVGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    return config;
}

void configureFixedExposure()
{
    sensor_t *sensor = esp_camera_sensor_get();

    sensor->set_exposure_ctrl(sensor, 0);
    sensor->set_aec2(sensor, 0);
    sensor->set_aec_value(sensor, 60);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_gainceiling(sensor, (gainceiling_t)GAINCEILING_2X);
}

bool connectStation(IPAddress &streamIp)
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WifiSettings::stationSsid, WifiSettings::stationPassword);

    IPAddress unsetIp(0, 0, 0, 0);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED || WiFi.localIP() == unsetIp)
    {
        delay(WIFI_CONNECT_RETRY_DELAY_MS);
        Serial.print(".");
        if (millis() - t0 > WIFI_CONNECT_TIMEOUT_MS)
        {
            Serial.println("\nWiFi connect timeout");
            return false;
        }
    }

    Serial.println("\nWiFi connected");
    streamIp = WiFi.localIP();
    return true;
}

void startAccessPoint(IPAddress &streamIp)
{
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    IPAddress apIp(
        WifiSettings::accessPointIp[0],
        WifiSettings::accessPointIp[1],
        WifiSettings::accessPointIp[2],
        WifiSettings::accessPointIp[3]
    );
    IPAddress apSubnet(
        WifiSettings::accessPointSubnet[0],
        WifiSettings::accessPointSubnet[1],
        WifiSettings::accessPointSubnet[2],
        WifiSettings::accessPointSubnet[3]
    );
    WiFi.softAPConfig(apIp, apIp, apSubnet);
    WiFi.softAP(WifiSettings::accessPointSsid, WifiSettings::accessPointPassword);

    Serial.println("\nWiFi access point started");
    Serial.print("SSID: ");
    Serial.println(WifiSettings::accessPointSsid);
    streamIp = WiFi.softAPIP();
}

bool startWifi(IPAddress &streamIp)
{
    if (WifiSettings::mode == WifiMode::AccessPoint)
    {
        startAccessPoint(streamIp);
        return true;
    }

    return connectStation(streamIp);
}

void CameraWebServer::begin()
{
    Serial.setDebugOutput(true);

    camera_config_t config = makeCameraConfig();
    esp_camera_init(&config);
    configureFixedExposure();

    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    IPAddress streamIp;
    if (!startWifi(streamIp))
    {
        return;
    }
    Serial.print("IP: ");
    Serial.println(streamIp);

    startCameraStreamServer();

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(streamIp);
    Serial.println("/stream' to connect");
}
