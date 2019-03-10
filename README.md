# Humidity Control

## Current Status

* A Bosch BME280 delivers air humidity data
* An analog capacitive moisture sensor delivers soil humidity data
* A fan is controlled with a pwm pin via web slider control or flash button
* Automatic mode uses humidity value to set fan speed.
* Setup wifi with captive portal.
* Reset wifi settings with long press of flash button
* Led feedback:
    * fast blink: boot setup and wifi connect
    * slow blink: wifi setup
    * on: normal operation
    * off: flash button pressed (on again if long press detected)

## Todo

* mDNS does not work properly
* mqtt integration
* configurable limits for the automatic fan control

## Flash Software Installation

Precondition: Python V2

### PlatformIO

    pip install platformio

### Arduino Framework for PlatformIO

    pio platforms install espressif8266 --with-package framework-arduinoespressif8266

## Wiring

If you want to use other pins, adjust the #defines accordingly.

### BME280

This is a breadboard friendly wiring for a standard BME280 breakout board and a NodeMCU:
All pins align, no extra wiring necessary.

    BME280 GND <--> ESP8266 GND
    BME280 SCL <--> ESP8266 D5 (D1 is standard)
    BME280 SDA <--> ESP8266 D6 (D2 is standard)
    BME280 3V3 <--> ESP8266 3V3


### Capacitive Soil Moisture Sensor (V1.2)

    CAP GND  <--> ESP8266 GND
    CAP VCC  <--> ESP8266 3V3
    CAP AOUT <--> ESP8266 A0

### Fan Power

    for PWM control of a fan connect D5 with mosfet gate.
    Schematics and pcb board are in the eagle/ subfolder (warning: uses rare p-channel mosfet).
  
