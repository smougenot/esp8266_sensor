[![Build Status](https://travis-ci.org/smougenot/esp8266_sensor.svg?branch=master)](https://travis-ci.org/smougenot/esp8266_sensor)
[![Join the chat at https://gitter.im/smougenot/esp8266_sensor](https://badges.gitter.im/smougenot/esp8266_sensor.svg)](https://gitter.im/smougenot/esp8266_sensor?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

# esp8266_sensor

ESP8266 micro controller and I2C connected sensors pushing messages over MQTT


### Aim
Provide a **simple** usage of sensors (hardware) with a **simple** way to publish them (software).

----

>**Note**
> This document is under construction
>
> TODO:
>- links I2C, MQTT, ESP8266
>- BOM
>- BOM links
>- Configuration instructions (Wifi)
>- images

----

### BOM

| Name     | Cost  | Informations   |
| :------- | ----: | ---- |
| NodeMCU  | $5    | Microcontroller compliant with Arduino with Wifi     |
| BMP180   |       | Sensor : temperature and pressure |
| BME280   |       | Sensor : temperature, pressure and humidity |
| BH1750   |       | Sensor : light |
| SHT21    |       | Sensor : temperature and humidity  |

### Build

Thanks to PlatformIO it is pretty simple to Build.
* install [PlatformIO](http://platformio.org/platformio-ide)
* install shell commands
* using a shell run : 
```pio run```
