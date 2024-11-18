#ifndef CONTROL_H
#define CONTROL_H

#include "common.h"
void FiltrarADC(int16_t *lastValue);
void InitQueue();
void PID_Torque(uint16_t pv, uint16_t sp);
void OnTimer1(TimerHandle_t xTimer);
void OnTimer2(TimerHandle_t xTimer);
void Task_bobinador(void *par);
void Task_encarrilador(void *par);
void Task_tensor(void *par);

#endif