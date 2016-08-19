# ESP8266drone
Robot Controller based on ESP8266 running freertos

The main idea is the dessign of the controller of an autonomus robot, like a quadcopter.

The ESP8266drone is based on the ESP8266, which includes the microprocessor and the Wifi link.
The dessing has two parts:
  -Hardware : PCB support for ICs and the electronics necessary for the brush motors.
  -Software : The ESP8266 programation, over the RTOS, of the procedures to sense, control and comunication.
            : The APP to command the ESP8266drone based on WifiLink
