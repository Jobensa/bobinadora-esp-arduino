

#include "common.h"
#ifndef _MODBUS_TCP_H_
#define _MODBUS_TCP_H_
void InitMQTT(void);
int  publish_msg_mqtt(char * topic, char *payload);

#endif // !_MODBUS_TCP_H_