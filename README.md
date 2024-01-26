# Cansat Sistema de Comunicaciones

En este repositorio se encuentran todos los archivos necesarios, diagramas de conexión y códigos para hacer el sistema de comunicaciones del satélite enlatado.

## Dispositivos utilizados

 - Raspberry Pi Pico

    <img src="https://www.330ohms.com/cdn/shop/products/foto_RaspberryPi_Pico_04_1500x.png?v=1610473122" width="200" height="200">

 - Arduino Nano

    <img src="https://electrocrea.com/cdn/shop/files/oem-9380-5624042-2-zoom_x700.jpg?v=1699591305" width="200" height="200">

 - Modulo de telemetría RYLR998

    <img src="https://reyax.com/upload/products/product_photo/566.png" width="200" height="200">

 - Sensor acelerómetro y giroscopio MPU6050

    <img src="https://m.media-amazon.com/images/I/51eWrHRzWJL._AC_UF894,1000_QL80_.jpg" width="200" height="200">

## Conexiones a los microcontroladores
### Conexiones con Raspberry Pi Pico
![Raspbery_Pico_Pinout](https://www.raspberrypi.com/documentation/microcontrollers/images/pico-pinout.svg) ![Diagrama_de_Conexiones]()

Como se muestra en el diagrama de conexiones y guiándonos de el pinout del raspberry pi pico, el modulo de telemetría (Lora RYLR998) va conectado de la siguiente forma

| Raspberry Pi Pico | Lora RYLR998 | 
| --------- | --------- |
| 3v3(Out) // Pin 36    | VDD // Pin 1   |
| GND // Pin 38    | GND // Pin 5    |
| UART0 RX // Pin 2    | TX // Pin 4   |
| UART0 TX // Pin 1    | RX // Pin 3    | 

De la misma manera tenemos las conexiones para conectar el acelerómetro y giroscopio (MPU6050)

| Raspberry Pi Pico | MPU6050 | 
| --------- | --------- |
| 3v3(Out) // Pin 36    | VDD // Pin 1   |
| GND // Pin 38    | GND // Pin 2    |
| I2C1 SDA // Pin 4    | SDA // Pin 4   |
| I2C1 SCL // Pin 5    | SCL // Pin 3    |

### Conexiones a Arduino NANO
![Arduino_Nano_Pinout](https://devboards.info/images/boards/arduino-nano/arduino-nano-pinout.webp) ![Diagrama_de_Conexiones]()

Para el caso de arduino tenemos las siguientes conexiones para el modulo de telemetría (Lora RYLR998)

| Arduino Nano | Lora RYLR998 | 
| --------- | --------- |
| +3v3 // Pin 17    | VDD // Pin 1   |
| GND // Pin 29    | GND // Pin 5    |
| UART0 RX // Pin 2    | TX // Pin 4   |
| UART0 TX // Pin 1    | RX // Pin 3    |

Y para las conexiones del acelerómetro y giroscopio (MPU6050)

| Arduino Nano | MPU6050 | 
| --------- | --------- |
| 5v // Pin 27    | VDD // Pin 1   |
| GND // Pin 29    | GND // Pin 2    |
| A4 SDA // Pin 23    | SDA // Pin 4   |
| A5 SCL // Pin 24    | SCL // Pin 3    |

## Requisitos Previos

### Trabajando con la Raspberry Pi Pico

Para trabajar con la raspberry se recomienda el uso de [Thonny](https://thonny.org) que es un IDE de programación muy amigable para todos aquellos que comienzan a programar en python, tiene una interfaz sencilla y te ayuda a realizar la configuración inicial de la raspberry, ademas sera el IDE que se utilizara de ejemplo.

[**Nota**] Los tutoriales de instalación de software se encontraran las distintas carpetas junto con los instaladores.

#### **Flasheo mediante Thonny**

Una vez que tengas instalado Thonny conecta tu raspberry pi pico mientras presionas el botón bootsel.

![BOOTSEL](/resources/images/getting_started/image13.png)

Al hacer esto tu computadora reconocerá el dispositivo como una unidad de almacenamiento. Cuando abras thonny te aparecerá una ventana similar a esa, da click donde dice *Python 3.X.X*

 ![Alt text](/resources/images/getting_started/image.png)

 y selecciona la opción que diga *Micropython (Raspberry Pi Pico)*

![Micropython](/resources/images/getting_started/image-10.png)

 Al hacer esto debería aparecer una ventana en donde te permita instalar el firmware necesario para utilizar micropython en la raspberry. La ventana lucirá algo asi

 ![Ventana_Instalación](/resources/images/getting_started/image-11.png)

 Simplemente dale a instalar y espera a que termine la instalación, después de eso cierra la ventana y ya estas listo para comenzar a programar 😊.

 #### **Flasheo Manual**

![Prueba](https://raw.githubusercontent.com/raspberrypi/documentation/develop/documentation/asciidoc/microcontrollers/micropython/images/MicroPython-640x360-v2.gif)

1. Descarga el firmware de micropython para raspberry pi pico dando click [aquí](https://micropython.org/download/rp2-pico/rp2-pico-latest.uf2)
2. Mantén presionado el botón de BOOTSEL mientras conectas tu raspberry al dispositivo.
3. Te aparece tu raspberry como una unidad de almacenamiento. Normalmente llamado RPI-RP2
4. Arrastra el archivo UF2 que descargaste a tu raspberry.
5. Comienza a programar 😊.

#### **Descarga de librerias**

Para utilizar el modulo de telemetría no es necesario ninguna librería, sin embargo para utilizar el acelerómetro y giroscopio necesitaremos las siguientes librerias [imu.py](https://github.com/shillehbean/youtube-channel/blob/main/imu.py) y [vector3d.py](https://github.com/shillehbean/youtube-channel/blob/main/vector3d.py), para instalar las librerías existen varias formas, haciendo uso de los beneficios que tiene thonny con micropython utilizando el sistema de archivos dentro del microcontrolador copiando y pegando los archivos(medio), y finalmente guardando los archivos mediante el IDE(sencillo). Aquí explicare de manera detallada la forma mas sencilla y dentro de las carpetas con los archivos vendrá el otro método explicado.

1. Da click a la opción de abrir fichero y selecciona este computador

    ![Abrir_Fichero](/resources/images/getting_started/image-1.png)
    ![Este_Computador](/resources/images/getting_started/image-2.png)

2. Encuentra la librería descargada y ábrela
3. Da click en fichero, después a guardar como

    ![Fichero](/resources/images/getting_started/image-4.png)
    ![Guardar_como](/resources/images/getting_started/image-5.png)

4. Aparecerá una ventana preguntando a donde guardar, seleccionamos *Dispositivo MicroPython* 

    ![Donde_Guardar](/resources/images/getting_started/image-6.png)

5. Saldrá una nueva ventana que nos pedirá darle un nombre al archivo y nos mostrara los archivos que se encuentran en la memoria Flash de nuestro microcontrolador. En este caso estamos guardando la librería vector3d.py por lo que lo guardaremos bajo ese nombre.

    ![Name](/resources/images/getting_started/image-7.png)

5. Y listo tenemos guardada nuestra librería en la raspberry

### Trabajando con el Arduino Nano

Para trabajar con el arduino nano es necesario instalar [arduino IDE](https://www.arduino.cc/en/software) que es un poderoso IDE que te agiliza el proceso de desarrollo al contar con una amplia compatibilidad de dispositivos y librerías listas para utilizarse.

#### **Instalación de librerias.**

Al utilizar el arduino nano con el modulo de telemetría no es necesario descargar ninguna librería, sin embargo para utilizar el acelerómetro y giroscopio es necesario instalar la librería correspondiente, para ello hay que hacer lo siguiente.

1. Dar click a *Library Manager*

    ![Library Manager](/resources/images/getting_started/image-8.png)

2. Buscar la librería llamada *Adafruit MPU6050* y la instalamos. (Puede tardar un rato)

    ![Instalar libreria](/resources/images/getting_started/image-9.png)

3. Listo tienes instalada la librería y puedes comenzar a programar 😊.

## Programación.

En esta sección se dará un breve resumen de los puntos mas importantes al momento de programar, si se desea una explicación mas profunda revisar los archivos que se encuentran dentro de la carpeta de códigos.

Es importante señalar que se esta trabajando con dos distintos lenguajes de programación, mientras que arduino utiliza C++ en su mayoría, en la raspberry estaríamos utilizando python para el desarrollo de la aplicación, es por eso que debemos tener mucho cuidado al seleccionar que hardware utilizaremos ya que no es compatible el código que se desarrolle en un hardware con el otro.

El modulo de telemetría hace uso del protocolo UART (universal asynchronous receiver transmitter) para comunicarse con el microcontrolador, pero solamente entiende instrucciones especificas que son una serie instrucciones llamados comandos AT. Adjunto una tabla con los comandos mas importantes al igual que la liga para ver todos los comandos que hay [Comandos AT](https://reyax.com//upload/products_download/download_file/LoRa_AT_Command_RYLR998_RYLR498_EN.pdf)

| Syntax | Response | Explicación
| --------- | --------- | --------- | 
| AT    | +OK   | Comprueba que el modulo responda a los comandos. |
| AT+ADDRESS = < Address > **AT+ADDRESS=1**| +OK | Establece la dirección ID del modulo. | 
| AT+ADDRESS?    | +ADDRESS=1   | Responde la dirección ID del modulo |
| AT+SEND=< Dirección >,< Longitud del mensaje >,< Mensaje> **AT+SEND=0,4,Hola**   | +OK    | Envía un mensaje a un modulo con la dirección especificada |
| AT+SEND? | +SEND=0,4,Hola | Responde cual fue el ultimo mensaje transmitido. |
| +RCV=< Dirección>,< Longitud>,< Mensaje>,< RSSI>,< SNR> **+RCV=1,4,Hola,-99,40** | | Enseña la información recibida. |

**¡Nota!** Los mensajes enviados deben ser strings (cadenas de caracteres) o en su defecto formato ASCII.

**¡Nota2!** El modulo por default viene configurado a una velocidad de baudrate de 115200, por lo que al inicializar los módulos UART deberás especificar esa velocidad.

### Programando Raspberry Pi Pico

Como se explicaba arriba para comunicarse con el modulo de telemetría hay que hacer uso del protocolo UART para eso hay que usar las funciones que vienen con micropython

```python
from machine import UART, Pin # Importa el objeto UART

uart1 = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5)) # Inicializa el GPIO de UART 1 a una velocidad de  9600, le indica los pines 4 y 5 para TX y RX respectivamente (Estos pines varían según el UART la pico tiene 8 pines 4 para UART0 y 4 para UART1)
uart1.write('hello')  # Escribe hello en el puerto

uart1.read(5)         # Lee 5 bytes e imprime lo leído
```
Esto es del lado del microcontrolador, ahora para poder hablar con el modulo tenemos que utilizar los comandos AT, un ejemplo sencillo seria

```python
from machine import UART, Pin # Importa el objeto UART

uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5)) # Inicializa el GPIO de UART 1 a una velocidad de  115200, le indica los pines 4 y 5 para TX y RX respectivamente

uart1.write('AT\r\n')  # Checa si el modulo responde

uart1.read(5)         # Lee 5 bytes e imprime lo leído
```
**¡Nota!** Es importante que al finalizar cada envío de datos al modulo se añada un *"enter"* o en su defecto
```python
'\r\n'
``` 

En este ejemplo mandamos el comando AT que comprueba la conexión con el modulo por lo que estamos esperando que al leer el puerto recibamos 

```
+OK
```

Para comunicarse con el sensor acelerómetro y giroscopio sera necesario inicializar el protocolo I2C (Inter-Integrated Circuit) para inicializar lo es muy similar al protocolo UART, unicamente debemos importar el objeto I2C que viene en la librería machine incluida con micropython de la siguiente forma

```python
from machine import Pin, I2C

i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=400_000) # El primer parámetro es el módulos I2C que utilizaremos, scl y sda reciben el numero de Pin en el que se encuentra ya que tenemos 8 pares de posibles pines para utilizar.
```
Una vez creado nuestro objeto i2c para trabajar con el modulo MPU6050 unicamente hay que utilizar la librería de imu y hacemos uso de la clase MPU6050, este se encargara de realizar toda la configuración necesaria para trabajar con el modulo 

```python
from imu import MPU6050
from time import sleep
from machine import Pin, I2C

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c) # Creación de objeto imu, para la obtención de información

while True:
    ax=round(imu.accel.x,2) # Obtiene información de la aceleración en X y se redondea a 2 decimales
    ay=round(imu.accel.y,2) # Obtiene información de la aceleración en Y
    az=round(imu.accel.z,2) # Obtiene información de la aceleración en Z
    gx=round(imu.gyro.x) # Obtiene información de posición en X
    gy=round(imu.gyro.y) # Obtiene información de posición en Y
    gz=round(imu.gyro.z) # Obtiene información de posición en Z
    tem=round(imu.temperature,2) # Obtiene la temperatura del sensor y redondea a 2 decimales
    print("ax",ax,"\t","ay",ay,"\t","az",az,"\t","gx",gx,"\t","gy",gy,"\t","gz",gz,"\t","Temperature",tem,"        ",end="\r")
    sleep(0.2) 
```
Entre la tabla de comandos AT, la inicialización de los módulos UART e I2C tenemos los comandos clave para realizar un programa en la raspberry pi pico que pueda recopilar información del sensor y mandarla a traves del modulo de telemetría, de todas formas encontraras un código que realiza esta función en la carpeta de códigos. 

### Programando Arduino Nano

Al igual que con la raspberry pi pico, debemos de inicializar el protocolo serial UART para poder comunicarnos con el modulo de telemetría. Para ello debemos utilizar las funciones especificas que incluye Arduino IDE

```C++
void Setup(){
    Serial.begin(115200) // Inicializa el modulo a 115200 baudrate
}
```
Una vez inicializado el protocolo UART podemos comenzar a leer y mandar información a los puertos, esto lo podemos hacer con las instrucciones *Serial.write()* y *Serial.read()*
```C++
void Setup(){ 
    Serial.begin(115200) // Inicializa el modulo a 115200 baudrate
}

void loop(){ // Loop principal
    Serial.write("AT\r\n"); // Manda comando AT a traves del puerto
    if (Serial.available()){ // Solamente si hay una respuesta 
        String response = Serial.read() // Lee y guarda la respuesta obtenida. Ej: +Ok\r\n
        Serial.write(response) // Imprime la respuesta obtenida
    }
}
```
Para el caso de el acelerómetro y giroscopio se cuenta con un código de ejemplo que viene junto con la librería del MPU6050 Adafruit, el cual destacaremos sus puntos mas importantes para comprender su funcionamiento.
```C++
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> // Librería de I2C 

Adafruit_MPU6050 mpu; // Creación del objeto mpu

void setup (void){
    mpu.begin() 
/*  Inicialización del modulo 
    Al inicializar lo sin parámetros se inicializa los valores predeterminados es decir
    	(	uint8_t 	i2c_address = MPU6050_I2CADDR_DEFAULT,
        TwoWire * 	wire = &Wire,
        int32_t 	sensor_id = 0 
        )	*/
}

void loop(){
    sensor_event_t a, g, tmp; // Se crean 3 variables de tipo sensor event
    mpu.getEvent(&a,&g,&tmp); // Obtenemos la información del sensor
}
```
El código final resulta mucho mas largo y con mayor cantidad de variables e instrucciones sin embargo estas serán detalladas en la sección de códigos. Por el momento nos enfocamos en el núcleo de este código que consta de 3 momentos

1. La importación de las librerías a utilizar.
2. Creación e inicialización del objeto mpu.
3. Definición de variables de tipo sensor_event, asi cómo obtención de la información proporcionada por el sensor. 

    - Tanto la variable de aceleración *a* como la variable de giroscopio *g* tienen componentes en el plano es decir se descomponen en *y, x* y *z* siendo la sintaxis
        
        - a.acceleration.x
        - g.gyro.x
    - En el caso de la temperatura es mas sencillo ya que unicamente es

        - temp.temperature

## Problemas Conocidos y Consejos de Ejecución

- Si los mensajes transmitidos no llegan completos o llegan incompletos en el modulo de telemetría

    - Verifica que todas las tierras estén aterrizadas, es decir estén conectadas entre si.
    - Verifica con que voltaje estas trabajando ya que no puedes alimentar al modulo con 5v y trabajar con un voltaje lógico de 3v3. 
        - Si estas trabajando con la raspberry pi pico, esta tiene un voltaje lógico de 3v3 mientras que el arduino nano trabaja con 5v.
        - Si tienes que alimentar al modulo con 5v y trabajas con la raspberry utiliza un voltage level shifter ![Level_shifter](https://uelectronics.com/wp-content/uploads/2018/07/PINOUT-min-4.jpg)
            - La forma en la que operan es que VA se conecta un voltaje de 3v3 y a VB un voltaje de 5v, de esta forma si tienes un voltaje lógico de 3v3 deberás conectarlo en algún pin de A1-A8 y obtendrás una salida de voltaje lógico a 5v por B1-B8.
            - Es aconsejable evitar utilizar el level shifter a toda costa ya que suele ocupar mucho espacio.
    -Verifica que el baud rate este bien configurado, es decir que corresponda el valor introducido al momento de crear el objeto UART y la velocidad a la que esta configurado el modulo RYLR998 que por default viene a una velocidad de 115200
- Para comprobar que se están enviando los mensajes utiliza un arduino uno y quítale el controlador AT con cuidado utilizando unas pinzas o destornillador plano chico, conecta el LORA RYLR998 a los puertos TX y RX de la tarjeta e inicializa el monitor serial y selecciona la opción que diga *Both NL & CR*.
    - De esta forma podrás interactuar con el modulo de manera directa y sin necesidad de un código, para esto en la linea de comando escribe los comandos *AT* y deberías comenzar a recibir respuestas, si esto no funciona podría significar que el modulo esta fallando o que alguna conexión es errónea.
    - Un arduino uno sin el controlador AT se debería ver algo asi ![Arduino_SinAT](/resources/images/getting_started/image-12.png)
    
- Recuerda que TX de la placa va a RX del Modulo

- Un buen punto para partir es probar por separado cada componente. 
    - Realizar códigos sencillos en los microcontroladores (como encender un led) para cerciorarse de que funciona el micro.
    - Conectar de manera individual cada modulo/sensor por ejemplo 
        - Al conectar el modulo de telemetría RYLR998 y mandar mensajes de prueba a un arduino uno sin controlador y comprobar que lleguen los mensaje y no se alteren.
        - Al conectar el acelerómetro y giroscopio MPU6050 comprobar que la información se actualiza a moverlo y no marque nada raro

- Se encontraron varios problemas al intentar programar con el Arduino nano, no se ha descubierto una solución a una comunicación intermitente de LORA RYLR998 o que no se logre obtener la información del MPU6050

- Si en algún momento mientras trabajan con la raspberry pi pico deja de responder o desean eliminar todos los archivos que tenga osease empezar de cero todo hay un archivo que se llama *flash_nuke.uf2* que sirve precisamente para eso, este archivo se ejecuta al igual que como se instala el firmware para micropython, manteniendo el botón bootsel y arrastrando hasta la raspberry que sera detectada como unidad de almacenamiento externa. Ten mucho cuidado cuando hagas esto ya que borra absolutamente todo lo que haya dentro del microcontrolador y tendrás que volver a instalar micropython, las librerías y todo lo demás que desees utilizar.

- Algo importante a tener en cuanta mientras se trabaja con el Arduino NANO es que este solo cuenta con un solo puerto UART, que es utilizado para la comunicación con el módulo RYLR998 y para la comunicación con la usb, es por esto que si se envía algún *Serial.print* a manera de debuggeo también se estará enviando al módulo por lo que podría generar basura o interferir en el correcto funcionamiento de este.   

## Referencias
Toda la información utilizada y consultada para realizar esta documentación se encuentra en esta sección, si hay algún punto que no quedo claro o desean obtener mas información aquí pueden partir.

- Instructables. (2023, May 22). How to use MPU6050 with Raspberry Pi Pico or Pico W. Instructables. https://www.instructables.com/How-to-Use-MPU6050-With-Raspberry-Pi-Pico-or-Pico-/
- Instructables. (2023, July 6). How to connect MPU6050 to Arduino Nano Every. Instructables. https://www.instructables.com/How-to-Connect-MPU6050-to-Arduino-Nano-Every/
- Raspberry Pi documentation - MicroPython. (n.d.). https://www.raspberrypi.com/documentation/microcontrollers/micropython.html#drag-and-drop-micropython
- Add the MicroPython firmware | Getting started with Raspberry Pi Pico | Micropython | Coding projects for kids and teens. (n.d.). https://projects.raspberrypi.org/en/projects/getting-started-with-the-pico/3
- Quick reference for the RP2 — MicroPython latest documentation. (n.d.). https://docs.micropython.org/en/latest/rp2/quickref.html
- Serial - Arduino reference. (n.d.). https://www.arduino.cc/reference/en/language/functions/communication/serial/
- Adafruit MPU6050 Sensor Library: Adafruit_MPU6050 Class Reference. (n.d.). https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050.html#acd28bc1399b050b832fc8dd702a6d75a
- Wire - Arduino reference. (n.d.). https://www.arduino.cc/reference/en/language/functions/communication/wire/
- Serial.print() - Arduino reference. (n.d.). https://www.arduino.cc/reference/en/language/functions/communication/serial/print/
- Serial.read() - Arduino reference. (n.d.). https://www.arduino.cc/reference/en/language/functions/communication/serial/read/
- Serial.begin() - Arduino reference. (n.d.). https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/
