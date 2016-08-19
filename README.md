# ESP8266drone
Robot Controller based on ESP8266 running FreeRTOS

The main idea is to design and develop the controller of an autonomus robot, like a quadcopter.

The ESP8266Drone is based on the ESP8266 IC, which includes a microprocessor and a Wifi link both in the same chip.

The desing has two parts:

  -Hardware : PCB support for ICs (Sensors and microprocessor) and the electronic necessary to control the brush motors.
  
  -Software : The ESP8266 firmware to sense, control and communicate with the App. It's developed using FreeRTOS version. This part also includes an Android Application that allows to pilot and receive telemetry from the ESP8266Drone.
