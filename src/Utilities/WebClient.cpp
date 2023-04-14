#include "WebClient.h"

WebClient::WebClient() {

}

String WebClient::get_pitch() {
    return String(pitch);
}

void WebClient::set_pitch(float p) {
    pitch = p;
}

void WebClient::init_server() {
    AsyncWebServer server(80);

    if (!SPIFFS.begin()) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.println(WiFi.localIP());

     // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/index.html");
    });
    server.on("/pitch", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", String(5).c_str());
    });
    // server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    //     request->send_P(200, "text/plain", readBME280Humidity().c_str());
    // });
    // server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    //     request->send_P(200, "text/plain", readBME280Pressure().c_str());
    // });

    // Start server
    server.begin();
}