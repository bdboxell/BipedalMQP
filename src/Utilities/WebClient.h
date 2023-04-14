#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Arduino.h>

class WebClient {
    private:
        // const char* ssid = "iPhone 13 Mini";
        // const char* password = "WiFi_Mooch";

        const char* ssid = "Router? I hardly know her";
        const char* password = "GoonSquad";
        float pitch = 0;

    public:
        WebClient();
        void init_server();
        String get_pitch();
        void set_pitch(float p);

};