// include/mqtt_handler.h
#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

class MQTTHandler {
public:
    MQTTHandler(const char* mqtt_server, int mqtt_port, const char* mqtt_user, const char* mqtt_password);
    void setup();
    void loop();
    void publish(const char* topic, const char* message, bool retained = false);
    bool isConnected();
    void setCallback(void (*callback)(char*, byte*, unsigned int));

private:
    void reconnect();
    PubSubClient client;
    WiFiClient espClient;
    const char* mqtt_server;
    int mqtt_port;
    const char* mqtt_user;
    const char* mqtt_password;
    unsigned long lastReconnectAttempt = 0;
};

#endif