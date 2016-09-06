#ifndef __WIFI_H__
#define __WIFI_H__

#include "esp_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "espconn.h"

#define AP_SSID "DRONE"

#define UDP_PORT 50000
#define UDP_REMOTE_PORT 50000

#define TCP_PORT 50001

void initAP(void);

void init_user_udp(void (* receive_function(void *, char *, unsigned short)));
void send_udp(uint8_t* pdata, uint8_t length);

//void init_user_tcp(void);
//void send_tcp(uint8_t* pdata, uint8_t length);

#endif