#ifndef COMMON_H
#define COMMON_H
    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>
    #include <inttypes.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include "freertos/event_groups.h"
    #include "driver/gpio.h"
    #include "esp_log.h"   

    #include "esp_dpp.h"
    #include "nvs_flash.h"
    #include "esp_mac.h"
    #include "esp_wifi.h"
    #include "esp_event.h"
    #include "esp_log.h"
    #include "esp_netif_net_stack.h"
    #include "esp_netif.h"
    #include "nvs_flash.h"
    //#include "lwip/inet.h"
    //#include "lwip/netdb.h"
    //#include "lwip/sockets.h"
   
    //#include "lwip/err.h"
    //#include "lwip/sys.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include "mqtt_handler.h"

#include <WiFi.h>
#include <PubSubClient.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"



#define OUT_ABobinador   19
#define OUT_BBobinador   18

#define OUT_ATensor   26
#define OUT_BTensor  27

#define OUT_AEncarrilador     33
#define OUT_BEncarrilador     25

#define IN_PULSE 12
#define CURRUNT_IN 36

#define GPIO_OUTPUT_PIN_SEL   ((1ULL<<DIR_ECARRILADOR) | (1ULL<<EN_ENCARRILADOR)| (1ULL<<OUT_BBobinador) |  (1ULL<<OUT_BTensor) |  (1ULL<<PULSE))

#define GPIO_INPUT_IO_0     16
#define GPIO_INPUT_IO_1     15
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))

#define ESP_INTR_FLAG_DEFAULT 0


/* STA Configuration */
//Barrancabermeja
//#define ESP_WIFI_STA_SSID           "FMLSC_wifi"
//#define ESP_WIFI_STA_PASSWD         "wifisacoy2020"
//Modem Claro Makesens
#define ESP_WIFI_STA_SSID           "Claro_081C36"
#define ESP_WIFI_STA_PASSWD         "K8Q7T9F7E9D7"

#define ESP_MAXIMUM_RETRY           3


#define ESP_WIFI_CHANNEL            1
#define MAX_STA_CONN                4




/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

typedef union  _trm_t
{
    struct 
    {
        uint dumy: 31;
        uint done: 1;
    };
   uint32_t Preset;   
}tmr_t;


typedef struct __mqtt_data_t
{
    char topic[32];
    char paload[128];
}mqtt_data_t;

typedef struct __mqtt_encarrilador_t
{
    char topic[32];
    uint32_t No_pasos;
    uint8_t NoCapas;
    float Distancia;
    uint16_t Longitud;
    uint16_t Speed;
    uint8_t Diametro;
    bool IsFF;
    bool IsRuning;
}mqtt_encarrilador_t;

typedef struct __data_bobinador_t
{
    uint16_t set_bob_speed;
    uint8_t set_bob_awg;
    float set_carrete_diameter;
    bool IsRuning;    
}data_Cbobinador_t;

typedef struct __data_Cencarrilador_t
{
    uint16_t set_bob_speed;
    uint8_t set_bob_awg;
    uint8_t set_bob_length;
    bool IsRuning;  
    bool isFF;  
}data_Cencarrilador_t;

typedef struct __data_tensor_t
{
    uint16_t set_bob_awg;
    uint16_t set_bob_speed;
    float set_carrete_diameter;
    bool IsRuning;
}data_Ctensor_t;




//Nuemro de vueltas por milimetro
#define AWG_40  10.6
#define AWG_39  9.61
#define AWG_38  8.4
#define AWG_37  7.5
#define AWG_36  6.8
#define AWG_35  6.1
#define AWG_34  5.46
#define AWG_33  4.85
#define AWG_32  4.32
#define AWG_31  3.93
#define AWG_30  3.5
#define AWG_29  3.1
#define AWG_28  2.8
#define AWG_27  2.52
#define AWG_26  2.27
#define AWG_25  2.02

//void FiltrarADC(int16_t *lastValue);
void Task_comMQTT(void *par);
void InitHardware();


#endif // !COMMON_H
