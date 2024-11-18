#include "common.h"

extern TaskHandle_t encarriladorTaskHandle;
extern TaskHandle_t bobinadorTaskHandle;
extern TaskHandle_t tensorTaskHandle ;
extern TaskHandle_t comMQTTTaskHandle;
extern TaskHandle_t OnTimer1Hanle;

extern QueueHandle_t queue_mqtt;
extern QueueHandle_t queue_encarrilador;
extern QueueHandle_t queue_bobinador;
extern QueueHandle_t queue_tensor;
extern QueueHandle_t queue_chage_status;
extern QueueHandle_t queue_chage_dir;
extern QueueHandle_t queue_trm_send;
extern QueueHandle_t queue_tmr_periodo_pulso;

extern TimerHandle_t H_send_mqtt;
extern TimerHandle_t H_periodo_pulso;
extern int id_send_mqtt;
extern int id_periodo_pulso;

extern uint16_t set_bob_length;
extern uint16_t set_speed_tensor;
extern uint8_t  set_bob_awg;
extern float set_carrete_diametro;
extern uint32_t countPasos;

extern bool     set_start;
extern bool     set_stop;
extern bool     IsRuning;
extern bool     set_IsFF;
extern bool     set_IsCCW;
extern bool    IsSend_mqtt;
extern bool    IsOnPeriodo_pulso;


extern int16_t voltage;
extern int16_t corriente;
extern uint16_t out;
extern int16_t kI,Kp,pv1,err,IntValue;

// // Configuración PWM
const int freq = 600;
const int resolution = LEDC_TIMER_13_BIT;
const int BobinadorChannel = 0;
const int TensorChannel = 1;
const int EncarriladorChannel = 2;

// Configuración PWM


void FiltrarADC(int16_t *lastValue)
{
    int16_t new_data=analogRead(CURRUNT_IN);
    *lastValue=(*lastValue-new_data)/4; 

}

void InitHardware()
{

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
