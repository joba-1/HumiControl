# Humidity Control

## Current Status

A Bosch BME280 delivers air humidity data
An analog capacitive moisture sensor delivers soil humidity data
A Fan is controlled with a pwm pin via web slider control
Setup Wifi with captive portal.
Reset Wifi settings with long press of flash button
Led feedback:
* fast blink: boot setup and wifi connect
* slow blink: wifi setup
* on: normal operation
* off: flash button pressed (on again if long press detected)

## Todo

* Use humidity value to set fan speed.
* mDNS does not work

## Software Installation

Precondition: Python V2

### PlatformIO

    pip install platformio

### Arduino Framework for PlatformIO

    pio platforms install espressif8266 --with-package framework-arduinoespressif8266

## Wiring

### BME280

This is a breadboard friendly wiring for a standard BME280 breakout board and a NodeMCU:
All pins align, no extra wiring necessary.

    BME280 GND <--> ESP8266 GND
    BME280 SCL <--> ESP8266 D5
    BME280 SDA <--> ESP8266 D6
    BME280 3V3 <--> ESP8266 3V3

If you use other pins for SCL and SDA, adjust the Wire.begin(SDA, SCL) call accordingly.

### Capacitive Soil Moisture Sensor (V1.2)

    CAP GND  <--> ESP8266 GND
    CAP VCC  <--> ESP8266 3V3
    CAP AOUT <--> ESP8266 A0
