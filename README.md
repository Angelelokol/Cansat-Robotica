# Cansat Sistema de Comunicaciones

En este repositorio se encuentran todos los archivos necesarios, diagramas de conexión y codigos para hacer el sistema de comunicaciones del satelite enlatado.

## Dispositivos utilizados

 - Raspberry Pi Pico
![Raspberry_Pico](https://www.330ohms.com/cdn/shop/products/foto_RaspberryPi_Pico_04_1500x.png?v=1610473122)
 - Arduino Nano
![Arduino_Nano](https://electrocrea.com/cdn/shop/files/oem-9380-5624042-2-zoom_x700.jpg?v=1699591305)
 - Modulo de telemetria RYLR998
![RYLR998](https://reyax.com/upload/products/product_photo/566.png)
 - Sensor acelerometro y giroscopio MPU6050
![MPU6050](https://m.media-amazon.com/images/I/51eWrHRzWJL._AC_UF894,1000_QL80_.jpg)


## Conexiónes a los microcontroladores
### Conexiones con Raspberry Pi Pico
![Raspbery_Pico_Pinout](https://www.raspberrypi.com/documentation/microcontrollers/images/pico-pinout.svg) ![Diagrama_de_Conexiones]()

Como se muestra en el diagrama de conexiones y guiandonos de el pinout del raspberry pi pico, el modulo de telemetria (Lora RYLR998) va conectado de la siguiente forma

| Raspberry Pi Pico | Lora RYLR998 | 
| --------- | --------- |
| 3v3(Out) // Pin 36    | VDD // Pin 1   |
| GND // Pin 38    | GND // Pin 5    |
| UART0 RX // Pin 2    | TX // Pin 4   |
| UART0 TX // Pin 1    | RX // Pin 3    | 

De la misma manera tenemos las conexiones para conectar el acelerometro y giroscopio (MPU6050)

| Raspberry Pi Pico | MPU6050 | 
| --------- | --------- |
| 3v3(Out) // Pin 36    | VDD // Pin 1   |
| GND // Pin 38    | GND // Pin 2    |
| I2C1 SDA // Pin 4    | SDA // Pin 4   |
| I2C1 SCL // Pin 5    | SCL // Pin 3    |

### Conexiones a Arduino NANO
![Arduino_Nano_Pinout](https://devboards.info/images/boards/arduino-nano/arduino-nano-pinout.webp) ![Diagrama_de_Conexiones]()

Para el caso de arduino tenemos las siguientes conexiones para el modulo de telemtria (Lora RYLR998)

| Arduino Nano | Lora RYLR998 | 
| --------- | --------- |
| +3v3 // Pin 17    | VDD // Pin 1   |
| GND // Pin 29    | GND // Pin 5    |
| UART0 RX // Pin 2    | TX // Pin 4   |
| UART0 TX // Pin 1    | RX // Pin 3    |

Y para las conexiones del celerometro y giroscopio (MPU6050)

| Arduino Nano | MPU6050 | 
| --------- | --------- |
| 5v // Pin 27    | VDD // Pin 1   |
| GND // Pin 29    | GND // Pin 2    |
| A4 SDA // Pin 23    | SDA // Pin 4   |
| A5 SCL // Pin 24    | SCL // Pin 3    |

## Requisitos Previos

### Trabajando con la Raspberry Pi Pico

Para trabajar con la raspberry se recomienda el uso de [Thonny](https://thonny.org) que es un IDE de programación para la gente que apenas comienza a programar en python, tiene una interfaz sencilla y te ayuda a realizar la configuración inicial de la raspberry.

#### Flasheo

## Referencias
- Instructables. (2023, May 22). How to use MPU6050 with Raspberry Pi Pico or Pico W. Instructables. https://www.instructables.com/How-to-Use-MPU6050-With-Raspberry-Pi-Pico-or-Pico-/
- Instructables. (2023, July 6). How to connect MPU6050 to Arduino Nano Every. Instructables. https://www.instructables.com/How-to-Connect-MPU6050-to-Arduino-Nano-Every/