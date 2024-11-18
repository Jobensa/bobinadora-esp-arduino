// src/main.cpp
#include <Arduino.h>
#include <ArduinoJson.h>
#include "mqtt_handler.h"

#include <WiFi.h>
#include <PubSubClient.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "common.h"

// Configuración WiFi
const char* ssid = "Claro_081C36";
const char* password = "K8Q7T9F7E9D7";


// Configuración MQTT
const char* mqtt_server = "192.168.20.11";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";

// Pines PWM


// Configuración PWM
const int freq = 600;
const int resolution = LEDC_TIMER_13_BIT;
const int BobinadorChannel = 0;
const int TensorChannel = 1;
const int EncarriladorChannel = 2;

// Crear instancia de MQTT
MQTTHandler mqtt(mqtt_server, mqtt_port, mqtt_user, mqtt_password);


void PID_Torque(uint16_t pv, uint16_t sp);
void OnTimer1(TimerHandle_t xTimer);
void OnTimer2(TimerHandle_t xTimer);

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

void FiltrarADC(int16_t *lastValue)
{
    int16_t new_data=analogRead(CURRUNT_IN);
    *lastValue=(*lastValue-new_data)/4; 

}


void InitQueue()
{
    queue_mqtt = xQueueCreate(2, sizeof(mqtt_data_t)); 
    queue_encarrilador= xQueueCreate(2, sizeof(mqtt_encarrilador_t)); 
    queue_bobinador= xQueueCreate(2, sizeof(data_Cbobinador_t));
    queue_tensor= xQueueCreate(2, sizeof(data_Ctensor_t));

    queue_chage_status=xQueueCreate(2, sizeof(bool));
    queue_chage_dir=xQueueCreate(2, sizeof(bool));
    queue_trm_send=xQueueCreate(2,sizeof(tmr_t));
    queue_tmr_periodo_pulso=xQueueCreate(2,sizeof(tmr_t));
}

void OnTimer1(TimerHandle_t xTimer)
{
    IsSend_mqtt=true;
}

void OnTimer2(TimerHandle_t xTimer)
{
    IsOnPeriodo_pulso=true;
}

void PID_Torque(uint16_t pv, uint16_t sp)
{
    int16_t tmpP;
    err=sp-pv;
    IntValue=err* kI;
    IntValue/=2;
    tmpP=(pv1-pv)*Kp;
    tmpP/=2;
    out+=tmpP+IntValue;
    pv1=pv;

    if (out>10000)
    {
        out=10000;
    }

    if (out<=0)
    {
        out=0; /* code */
    }

    
    if(!IsRuning) out=0;
    //SetDuty_Bobinador(out);
    ledcWrite(BobinadorChannel,out);

    Serial.printf("pv: %d sp_corriente: %d  pid-ot: %d \n",pv,sp,out);

    

}

void Task_bobinador(void *par)
{

    data_Cbobinador_t data_rx;
    //uint16_t corriente=0;
    uint16_t SP_corriente=0;
    uint16_t AdjSP=0;
    float diametro=1.0;
    uint8_t awg=40;
    bool IsRuning=false;

    // if (!xTimerIsTimerActive(H_send_mqtt)) 
    // {
    //     xTimerStart(H_send_mqtt, 0);
    // }
   

    for (;;)
    {
        
        if( xQueueReceive(queue_bobinador, &(data_rx), (TickType_t)1))
        {
            awg=data_rx.set_bob_awg;
            IsRuning=data_rx.IsRuning;
            diametro=data_rx.set_carrete_diameter;            
        }   

        switch (awg)
            {
            case 40:
                    SP_corriente=1500;
                break;
            case 39:
                    SP_corriente=1600;
                break;
            case 38:
                    SP_corriente=1700;
                break;  
            case 37:
                    SP_corriente=1800;
                break;
            case 36:
                    SP_corriente=2000;
                break; 
            case 35:
                    SP_corriente=2100;
                break;
            case 34:
                    SP_corriente=2200;
                break;
            case 33:
                    SP_corriente=2300;
                break;  
            case 32:
                    SP_corriente=2400;
                break;
            case 31:
                    SP_corriente=2500;
                break;      
            case 30:
                    SP_corriente=2600;
                break;
            case 29:
                    SP_corriente=2700;
                break;
            case 28:
                    SP_corriente=2800;
                break;  
            case 27:
                    SP_corriente=2900;
                break;
            case 26:
                    SP_corriente=3200;
                break; 
            case 25:
                    SP_corriente=3500;
                break;                               
            
            default:
                SP_corriente=4800;
                break;
            }

        
            if (IsRuning)
            {
                AdjSP=SP_corriente+(diametro*30);
                // gpio_set_level(OUT_BBobinador, 0 );

                if(AdjSP>6000)AdjSP=6000;
            }
            else
            {
                SP_corriente=0;
            }                


        FiltrarADC(&corriente);  

    
        
        
        
        PID_Torque(corriente,AdjSP);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    
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

void Task_encarrilador(void *par)
{

    //Tornillo 20 VxIn ==> 1.27 mm x Vuelta 
    //Motor 48 pasos x vuelta  
    //Recuccion 2.2 a 1 == 105.6 pasos por vuelta  ==> 83.15 pasos por milimetro
    //75mm logitud ==> 75mm * 83.15 pasos = 6237 pasos

  

    uint32_t set_pasos=0;
    // uint16_t periodo_pulso=10; 
    // static uint16_t last_periodo_pulso;
    
    uint8_t  counCapas=0;
    uint8_t  Longitud=10; //mm
    uint16_t speed=0;
    
   // uint8_t diametro=10;
    float distancia=0.0;   //mm
    bool IsFF_enca=false;
    bool IsRuning=false;
    data_Cencarrilador_t dataRX;  //Recivir datos de control
    mqtt_data_t data; //Enviar datos mqtt 

    mqtt_encarrilador_t data_enca;  
    // static uint8_t last_pulso;
    
    sprintf(data_enca.topic,"/bob/encarrilador");
    data_enca.No_pasos=0;
    data_enca.NoCapas=0;
    data_enca.Distancia=0; //en mm
    data_enca.IsFF=false;
    data_enca.IsRuning=false;
    data_enca.Speed=0;

    //79 pasos por milimetro

    

    set_pasos=185*10;

    for (;;)
    {
        if( xQueueReceive(queue_encarrilador, &(dataRX), (TickType_t)1))
        {
             IsRuning=dataRX.IsRuning; 
             Longitud=dataRX.set_bob_length;
             speed=dataRX.set_bob_speed;

        }   


        if (IsRuning)
        {
            data_enca.IsRuning=dataRX.IsRuning; 
            //periodo_pulso=1;   

            if (speed>10 && speed < 500)
            { 
                speed=500;
            }
            

            if (IsFF_enca)
            {
                //SetDuty_Bobinador(speed+24);
                ledcWrite(EncarriladorChannel,speed+24);
                //gpio_set_level(OUT_BBobinador, 0); 
                digitalWrite(OUT_BEncarrilador,0);

            }
            else 
            {
                //SetDuty_Bobinador(1000-speed);
                ledcWrite(EncarriladorChannel,1000-speed);
                //gpio_set_level(OUT_BBobinador, 1); 
                digitalWrite(OUT_BEncarrilador,1);
            }
            
            
            
            countPasos+=(speed/38);
            data_enca.No_pasos=countPasos;
            set_pasos=Longitud*185;
            distancia=(countPasos/185.0);
            data_enca.Distancia=distancia;
            data_enca.Speed=speed;
           

            if (countPasos>=set_pasos)
            {
                countPasos=0;
                counCapas++;
                
                if (IsFF_enca) IsFF_enca=false;
                else IsFF_enca=true;
                
                IsRuning=false;

                xQueueSend(queue_chage_status, (void*)&IsRuning, (TickType_t)0);
                //xQueueSend(queue_chage_dir, (void*)&IsFF_enca, (TickType_t)0);
               
            }

            //gpio_set_level(EN_ENCARRILADOR, 0); 
            
        }
        else
        {
            //SetDuty_Bobinador(0);
            ledcWrite(EncarriladorChannel,0);           

            //gpio_set_level(OUT_BBobinador, 0); 
            digitalWrite(OUT_BEncarrilador,0);
        }
         


        data_enca.IsFF=IsFF_enca;
        data_enca.NoCapas=counCapas;
        data_enca.No_pasos=countPasos;
        data_enca.Speed=speed;
        data_enca.Distancia=distancia;
       
        if (IsSend_mqtt)
        {
            IsSend_mqtt=false;
            //Serial.printf("countPasos: %ld, distancia: %.2f dir: %d set-pasos: %ld\n",countPasos, distancia, IsFF_enca,set_pasos);
            sprintf(data.paload,"{\"NoPasos\": %ld,\"Distancia\": %.2f,\"isFF\": %d,\"Runing\": %d,\"NoCapas\": %d,\"Speed\": %d}", data_enca.No_pasos,data_enca.Distancia,data_enca.IsFF,data_enca.IsRuning,data_enca.NoCapas,data_enca.Speed);
            sprintf(data.topic,"/bob/encarrilador");
            xQueueSend(queue_mqtt, (void*)&data, (TickType_t)0);
            //Serial.printf("Speed Enca: %d\n",speed);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
                 
    }
    

}

void Task_tensor(void *par)
{
    uint16_t duty_tensor=0;
    uint16_t speedTen=0;
    float diametro=1.0;
    data_Ctensor_t dataRX;
   
    bool IsRuningTen=false;
    //SetDuty_M1_Tensor(0);

    for(;;)
    {
        if( xQueueReceive(queue_tensor, &(dataRX), (TickType_t)1))
        {
             IsRuningTen=dataRX.IsRuning;             
             speedTen=dataRX.set_bob_speed;
             
            
        }   
        
        //printf("Int: %d speed: %d\n", IntValue, speedTen);

        if(IsRuningTen)duty_tensor=speedTen;        
        else duty_tensor=0; 

        duty_tensor/=4;
        
       // SetDuty_M1_Tensor(duty_tensor);
       ledcWrite(TensorChannel,duty_tensor);
       // gpio_set_level(OUT_BTensor, 0);
       digitalWrite(OUT_BTensor,0);
       
        vTaskDelay(pdMS_TO_TICKS(1000));
        
    }

}


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

void setup() {
    Serial.begin(115200);
    
    // Configurar canales PWM
    ledcSetup(BobinadorChannel, freq, resolution);
    ledcSetup(TensorChannel, freq, resolution);
    ledcSetup(EncarriladorChannel, freq, resolution);
    
    // Adjuntar pines a canales
    ledcAttachPin(OUT_ABobinador, BobinadorChannel);
    ledcAttachPin(OUT_BTensor, TensorChannel);
    ledcAttachPin(OUT_BEncarrilador, EncarriladorChannel);

    //pinMode(OUT_ABobinador,OUTPUT);
    pinMode(OUT_ATensor,OUTPUT);
    pinMode(OUT_AEncarrilador,OUTPUT);
    pinMode(OUT_BBobinador,OUTPUT);

    
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