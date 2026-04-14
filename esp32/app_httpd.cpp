#include "esp_http_server.h"
#include "esp_camera.h"
#include "Arduino.h"

namespace
{
constexpr char STREAM_CONTENT_TYPE[] = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
constexpr char STREAM_BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
constexpr char STREAM_PART[] = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t streamHttpd = nullptr;

esp_err_t sendJpegFrame(httpd_req_t *req, camera_fb_t *fb)
{
    char partBuffer[64];
    size_t headerLength = snprintf(partBuffer, sizeof(partBuffer), STREAM_PART, (unsigned int)fb->len);

    esp_err_t res = httpd_resp_send_chunk(req, partBuffer, headerLength);
    if (res == ESP_OK)
    {
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    }
    if (res == ESP_OK)
    {
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    }
    return res;
}

esp_err_t handleStreamRequest(httpd_req_t *req)
{
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    while (true)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            Serial.println("Camera capture failed");
            return ESP_FAIL;
        }

        res = sendJpegFrame(req, fb);
        esp_camera_fb_return(fb);

        if (res != ESP_OK)
        {
            return res;
        }
    }
}
}

void startCameraStreamServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t stream_uri = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = handleStreamRequest,
        .user_ctx = nullptr
    };

    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&streamHttpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(streamHttpd, &stream_uri);
    }
}
