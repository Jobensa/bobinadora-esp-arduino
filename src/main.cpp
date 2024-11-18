// src/main.cpp

#include "common.h"
#include "control.h"

// Configuración WiFi
const char* ssid = "Claro_081C36";
const char* password = "K8Q7T9F7E9D7";

// Configuración MQTT
const char* mqtt_server = "192.168.20.11";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";

// Pines PWM




// Crear instancia de MQTT
MQTTHandler mqtt(mqtt_server, mqtt_port, mqtt_user, mqtt_password);

int16_t voltage;
int16_t corriente;
uint16_t out;
int16_t kI,Kp,pv1,err,IntValue;

TaskHandle_t encarriladorTaskHandle = NULL;
TaskHandle_t bobinadorTaskHandle = NULL;
TaskHandle_t tensorTaskHandle = NULL;
TaskHandle_t comMQTTTaskHandle = NULL;
TaskHandle_t OnTimer1Hanle=NULL;

QueueHandle_t queue_mqtt;
QueueHandle_t queue_encarrilador;
QueueHandle_t queue_bobinador;
QueueHandle_t queue_tensor;
QueueHandle_t queue_chage_status;
QueueHandle_t queue_chage_dir;
QueueHandle_t queue_trm_send;
QueueHandle_t queue_tmr_periodo_pulso;

TimerHandle_t H_send_mqtt;
TimerHandle_t H_periodo_pulso;
int id_send_mqtt=1;
int id_periodo_pulso=2;



uint16_t set_bob_length=10;
uint16_t set_speed_tensor=0;
uint8_t  set_bob_awg=40;
float set_carrete_diametro;
uint32_t countPasos=0;

bool     set_start;
bool     set_stop;
bool     IsRuning;
bool     set_IsFF;
bool     set_IsCCW;
bool    IsSend_mqtt=false;
bool    IsOnPeriodo_pulso=false;


 
data_Cencarrilador_t dataENCA; 
data_Cbobinador_t data_bobinador;
data_Ctensor_t data_tensor;


void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.printf("Conectando a %s", ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi conectado");
    Serial.print("Dirección IP: ");
    Serial.println(WiFi.localIP());
}

void publishStatus() {
       
    //mqtt.publish("esp32/status", status.c_str(), true);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    
    JsonDocument doc;   
    int temp_val;       
    char json[64];    
    mempcpy(json,payload,length);

    DeserializationError error = deserializeJson(doc, json);

    //Serial.printf("Dato: %s \n",json);

    //int valor = mensaje.toInt();
    //valor = constrain(valor, 0, 255);

    if (String(topic) == "/bob/start") {
        
        IsRuning=true;        
        temp_val=doc["start"];
        Serial.printf("Value: %d\n",temp_val);

    }
    else if (String(topic) == "/bob/stop") {
        //ledcWrite(motor2Channel, valor);
        IsRuning=false;
        temp_val=doc["stop"];
        Serial.printf("Value: %d\n",temp_val);
        //Serial.printf("Stop: %s\n", json);
    }
    else if (String(topic) == "/bob/set_speed_tensor") {
        //ledcWrite(motor3Channel, mensaje);
        Serial.printf("speed_tensor: %s\n", json);
        set_speed_tensor=doc["set_speed_tensor"];        
        Serial.printf("Value: %d\n",set_speed_tensor);
    }

    else if (String(topic) == "/bob/set_bob_awg") {
        //ledcWrite(motor3Channel, valor);
        Serial.printf("set_bob_awg: %s\n", json);
        set_bob_awg=doc["set_bob_awg"];
        Serial.printf("Value: %d\n",set_bob_awg);
    }

    else if (String(topic) == "/bob/set_length_inductor") {
        //ledcWrite(motor3Channel, valor);
        Serial.printf("length_inductor: %s\n", json);
        set_bob_length=doc["set_length_inductor"];
        Serial.printf("Value: %d\n",set_bob_length);
    }

    else if (String(topic) == "/bob/set_SendFF") {
        //ledcWrite(motor3Channel, valor);
        Serial.printf("/bob/set_SendFF: %s\n", json);
        temp_val=doc["set_SendFF"];
        set_IsFF=true;
        set_IsCCW=false;
        Serial.printf("Value: %d\n",temp_val);
    }

    else if (String(topic) == "/bob/set_SendCCW") {
        //ledcWrite(motor3Channel, valor);
        Serial.printf("/bob/set_SendCCW: %s\n", json);
        temp_val=doc["set_SendCCW"];
        set_IsFF=false;
        set_IsCCW=true;
        Serial.printf("Value: %d\n",temp_val);
    }

     else if (String(topic) == "/bob/set_StopPos") {
        //ledcWrite(motor3Channel, valor);
        Serial.printf("/bob/set_StopPos: %s\n", json);
        temp_val=doc["set_StopPos"];
        set_IsFF=false;        
        set_IsCCW=false;
        Serial.printf("Value: %d\n",temp_val);
    }

    

    else if (String(topic) == "/bob/set_bob_diametro") {
        //ledcWrite(motor3Channel, valor);
        Serial.printf("/bob/set_bob_diametro %s\n", json);
        set_carrete_diametro=doc["set_bob_diametro"];
        Serial.printf("Value: %d\n",set_carrete_diametro);
    }
    
    //publishStatus();
}


void Task_comMQTT(void *par)
{

    mqtt_data_t mqtt_data_rx;
    for (;;)
    {
        mqtt.loop();
        if( xQueueReceive(queue_mqtt, &(mqtt_data_rx), (TickType_t)1))
        {
            //printf("Received data from queue == %s\n", mqtt_data_rx.topic);            
            //publish_msg_mqtt(mqtt_data_rx.topic,mqtt_data_rx.paload);
            mqtt.publish(mqtt_data_rx.topic,mqtt_data_rx.paload,false);
            
        }        
        vTaskDelay(pdMS_TO_TICKS(100));

    }
    
}


void setup() {
    Serial.begin(115200);
    

    InitHardware();
    setup_wifi();
    mqtt.setCallback(mqtt_callback);
    mqtt.setup();

    pv1=0;
    kI=1;
    Kp=1;
    voltage=0;
    data_Cencarrilador_t dataENCA; 
    data_Cbobinador_t data_bobinador;
    data_Ctensor_t data_tensor;

    set_IsFF=false;
    set_IsCCW=false;
   
    InitQueue();

    if (queue_mqtt == 0)
    {
        printf("Failed to create queue= %p\n", queue_mqtt);
    }
    

    xTaskCreate(Task_encarrilador, "Task_ecarrilador", 4096, NULL, 10, &encarriladorTaskHandle);
    //xTaskCreate(Task_bobinador, "Task_bobinador", 4096, NULL, 10, &bobinadorTaskHandle);
    xTaskCreate(Task_tensor, "Task_tensor", 4096, NULL, 10, &tensorTaskHandle);
    xTaskCreate(Task_comMQTT, "Task_comMQTT", 4096, NULL, 10, &comMQTTTaskHandle);

    H_send_mqtt= xTimerCreate("TIMER1",pdMS_TO_TICKS(1000),pdTRUE,(void *)id_send_mqtt,&OnTimer1);
    H_periodo_pulso= xTimerCreate("TIMER2",pdMS_TO_TICKS(10),pdTRUE,(void *)id_periodo_pulso,&OnTimer2);


    xTimerStart(H_send_mqtt,10);
    xTimerStart(H_periodo_pulso, 10);

}

void loop() {
    
    bool tmpRuning;

    if( xQueueReceive(queue_chage_status, &(tmpRuning), (TickType_t)1))
    {
        IsRuning=tmpRuning;
    }   

    // if( xQueueReceive(queue_chage_dir, &(tmpFF), (TickType_t)1))
    // {
    //     set_IsFF=tmpFF;
    // }   

    data_bobinador.IsRuning=IsRuning;
    data_bobinador.set_bob_awg=set_bob_awg;
    data_bobinador.set_carrete_diameter=set_carrete_diametro;
    xQueueSend(queue_bobinador, (void*)&data_bobinador, (TickType_t)0);

    data_tensor.IsRuning=IsRuning;
    data_tensor.set_bob_awg=set_bob_awg;
    data_tensor.set_bob_speed=set_speed_tensor;
    xQueueSend(queue_tensor, (void*)&data_tensor, (TickType_t)0);


    dataENCA.IsRuning=IsRuning;
    dataENCA.set_bob_awg=set_bob_awg;
    dataENCA.set_bob_speed=set_speed_tensor;
    dataENCA.isFF=set_IsFF;
    dataENCA.set_bob_length=set_bob_length;

    xQueueSend(queue_encarrilador, (void*)&dataENCA, (TickType_t)0);
    vTaskDelay(pdMS_TO_TICKS(750));
}