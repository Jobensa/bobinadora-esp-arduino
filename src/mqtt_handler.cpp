// src/mqtt_handler.cpp
#include "mqtt_handler.h"

MQTTHandler::MQTTHandler(const char* server, int port, const char* user, const char* password) 
    : mqtt_server(server), mqtt_port(port), mqtt_user(user), mqtt_password(password) {
    client.setClient(espClient);
    client.setServer(mqtt_server, mqtt_port);
}

void MQTTHandler::setup() {
    reconnect();
}

void MQTTHandler::loop() {
    if (!client.connected()) {
        unsigned long now = millis();
        if (now - lastReconnectAttempt > 5000) {
            lastReconnectAttempt = now;
            reconnect();
        }
    }
    client.loop();
}

void MQTTHandler::publish(const char* topic, const char* message, bool retained) {
    if (client.connected()) {
        client.publish(topic, message, retained);
    }
}

bool MQTTHandler::isConnected() {
    return client.connected();
}

void MQTTHandler::setCallback(void (*callback)(char*, byte*, unsigned int)) {
    client.setCallback(callback);
}

void MQTTHandler::reconnect() {
    if (!client.connected()) {
        Serial.print("Conectando a MQTT...");
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("conectado");
            
            // Suscribirse a los tópicos
            client.subscribe("/bob/qos1");
            client.subscribe("/bob/start");
            client.subscribe("/bob/stop");
            client.subscribe("/bob/set_length_inductor");
            client.subscribe("/bob/set_bob_awg");
            client.subscribe("/bob/set_speed_tensor");
            client.subscribe("/bob/set_SendFF");
            client.subscribe("/bob/set_SendCCW");
            client.subscribe("/bob/set_bob_diametro");    
            client.subscribe("/bob/set_StopPos"); 
            
            // Publicar mensaje de conexión
            client.publish("esp32/conexion", "ESP32 conectado", true);
        } else {
            Serial.print("falló, rc=");
            Serial.print(client.state());
            Serial.println(" intentando de nuevo en 5 segundos");
        }
    }
}