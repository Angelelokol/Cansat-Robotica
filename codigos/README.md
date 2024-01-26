# Explicación Código
En esta sección se encuentra las explicaciones de los códigos utilizados para realizar este proyecto
## Códigos de Raspberry Pi Pico
### Send.py
Si bien viene cómo librería fue escrita por mi y en realidad es algo muy sencillo ya que consta de una única función aunque podría tener mas si asi se necesitara.

```python
def inicio(coms,address="2"):
    length = str(len(coms))
    uart.write("AT+SEND="+address+","+length+","+coms+"\r\n")

```
Básicamente lo que hace es que recibe un mensaje ya sea un texto cómo "Hola" y obtiene la longitud del mensaje (que es necesaria para el segundo argumento de el comando *AT+SEND*) y posiciona el mensaje en el tercer argumento de nuestro comando AT, también necesita una dirección a la cuál enviara el mensaje si no se da una se asignara por default la dirección *2*.

Es importante que unicamente se pueden mandar variables de tipo string a la función de lo contrario mandara error y no funcionara el código.

### testTelemetria.py

Este código fue creado para probar el módulo RYLR998 sin mandar información del sensor. 

El codigo consta de las siguientes partes.

```python
from machine import Pin, UART
from time import sleep_ms
```
- La importación de los objetos y funciones a utilizar. En este caso los objetos *Pin, UART* que vienen de *machine* y la función de *sleep_ms* 
    - Con el objeto *Pin* le indicaremos al programa en que pines físicos se encuentra el puerto serial.
    - Con el objeto *UART* crearemos un objeto serial de tipo UART con el cual podremos mandar información al módulo y recibir información del mismo.
    - Finalmente utilizamos la función *sleep_ms* para esperar determinados milisegundos ha que se termine de ejecutar alguna instrucción.   

```python
def inicio(coms,address="2"):
    """
    Recibe como parámetros el mensaje y la dirección a la que se enviará el mensaje
    Función para enviar el comando AT+SEND
    """
    length = str(len(coms)) # Convierte la longitud del mensaje a una cadena de caracteres
    uart.write("AT+SEND="+address+length+coms+"\r\n") # Escribe el comando AT con los parámetros dados en el puerto serial 
    msg = uart.readline() # Lee la respuesta del modulo RYLR998
    print(msg) # Imprime en la terminal la respuesta del modulo RYLR998
```
- A continuación nos encontramos con la función de *inicio* la cuál ya ha sido definida. Se puede profundizar en explicar porque el parámetro *address = "2"*, esto es asi debido a que en caso de no recibir un segundo parámetro por default tomara la dirección 2.

```python
if __name__ == "__main__":
    uart = UART(0, 115200, tx=Pin(0), rx=Pin(1)) # Crea un objeto UART 
    uart.init(bits=8, parity=None, stop=1) # Inicializa el puerto serial
    count = 0 # Variable para contar los mensajes recibidos
```
- Aquí nos encontramos con el bloque de configuración del dispositivo, vamos a destriparlo por instrucciones 

    ```python
    uart = UART(0, 115200, tx=Pin(0), rx=Pin(1)) # Crea un objeto UART
    ```
    - En esta linea creamos un objeto *UART* la cuál recibe como parámetros:
        
        1. Número de puerto UART a utilizar, que en el caso de la raspberry pi pico son 2 UART0 y UART1. Cada UART tiene 2 pares de RX y TX por lo que nos adelantamos a los parámetros 3 y 4

        2. El parámetro tx pide que se mande un objeto de tipo Pin el cuál debe especificar en que puerto físico se encuentra el *Pin TX*, es importante especificar que pin se usara ya que UART0 cuenta con 2 *TX*, el primero se encuentra en el *Pin(0)* y el segundo en el *Pin(12)*.

        3. Por otro lado el cuarto parámetro nos pide que definamos en que pin se encuentra RX, para esto usamos el objeto *Pin* al igual que con TX debemos indicar que pin usaremos ya que existen 2 RX, para UART0 a RX lo encontramos en *Pin(1)* y en *Pin(13)*.

        4. Por ultimo el segundo parámetro son los bits por segundo o también conocidos como baud rate, este es un parámetro importantísimo de conocer ya que si este parámetro no coincide entre el que se coloca al momento de crear el objeto UART y la velocidad a la que va el modulo.
            - Para trabajar con el Lora RYLR998 debemos indicar un baud rate de 115200 ya que esta es la velocidad a la que viene por default el modulo.

    ```python
    uart.init(bits=8, parity=None, stop=1)
    ```
    - Aquí estamos inicializando el puerto serial, con sus configuraciones vamos a describir cada parámetro
        
        1. El primer parámetro, bits hace referencia a que tan grande serán los bits por carácter, por default viene 8 que es el equivalente a una letra, sin embargo es posible poner 7 o 9. Se recomienda dejarlo en 8 bits a menos que se desarrolle una aplicación muy especifica.

        2. *parity* o en español paridad, hace referencia a una forma de verificar que los datos se hayan enviado correctamente ya que si se habilita comprueba si el dato recibido es par o non. No se recomienda habilitarlo ya que variara la cantidad de bits que mandemos a traves del puerto dependiendo de las mediciones del sensor, por lo que es idóneo dejarlo en None, sin embargo si se quisiera utilizar acepta como parámetros 0 (par) y 1 (non)

        3. *stop* es la cantidad de bits destinados para indicar que se acabo el mensaje, acepta como parámetros 1 y 2

```python
while True:
        if str.lower(input('Enviar mensaje? ')) == 'no': # Pregunta si se desea enviar un mensaje, si se contesta que no, se entra en modo recepción
            msg = uart.read() # Lee el mensaje recibido
            if msg is not None: # Si el mensaje no es nulo, se imprime en la terminal
                print(count,msg)
                count = count+1
        else:
            address = input('Direccion: ') # Pregunta la dirección a la que se enviará el mensaje
            coms = input('Mensaje: ') # Pregunta el mensaje que se enviará
            inicio(coms,address) # Llama a la función inicio
```
- Finalmente tenemos el ciclo infinito que sera el código que se ejecutara hasta que el usuario decida finalizarlo. De igual forma vamos parte por parte.

    ```python
    if str.lower(input('Enviar mensaje? ')) == 'no': # Pregunta si se desea enviar un mensaje, si se contesta que no, se entra en modo recepción
            msg = uart.read() # Lee el mensaje recibido
            if msg is not None: # Si el mensaje no es nulo, se imprime en la terminal
                print(count,msg)
                count = count+1
    ```
    - Lo primero que encontramos al entrar al ciclo es una condicional que le pide al usuario un input mediante la terminal, esto es para saber si desea enviar un mensaje o desea recibir mensajes. 
    - Una vez que se recibe el input el comando *str.lower()* se encarga que no importa si el usuario escribe en mayúsculas o minúsculas o combinado, el código lo convertirá a minúsculas para que asi se pueda realizar la comparativa entre lo escrito por el usuario y nuestra condición.
    - En caso de que el usuario decida no mandar mensajes se leerá el puerto serial y solo si hay información en el puerto (*if msg is not None*) se imprimirá en la terminal para que podamos verlo junto con un contador que aumenta para saber cuantos mensajes, han sido recibidos.

    ```python
    else:
            address = input('Direccion: ') # Pregunta la dirección a la que se enviará el mensaje
            coms = input('Mensaje: ') # Pregunta el mensaje que se enviará
            inicio(coms,address) # Llama a la función inicio
    ```   
    - El final del código es la condición por si el usuario desea mandar un mensaje, en donde a traves de un input por terminal se le pide al usuario que entregue una dirección a la cual se desea enviar el mensaje, posteriormente se pide que ingrese el mensaje que quiere mandar y por ultimo se manda a la función inicio junto con los parámetros dados por el usuario.

### main.py

Este seria el código principal en el cuál se puede obtener información del sensor y mandarse mediante el modulo RYLR998 a otro microcontrolador o computador que tenga el mismo modulo. Por lo que describiremos el código y cómo funciona.

```python
from imu import MPU6050
from time import sleep
from machine import Pin, I2C
from send import inicio

#Shows Pi is on by turning on LED when plugged in
LED = machine.Pin("LED", machine.Pin.OUT) # Crea objeto LED
LED.on()
```
- Lo primero que hace el código es importar objetos de las librerías, el primer objeto que vemos que se importa es el *MPU6050* que sera el que nos ayude a obtener las mediciones de los sensores, posteriormente se importa la función sleep, los objetos *Pin e I2C* con los cuales configuraremos los puertos de entrada/salida y finalmente importamos nuestra ya conocida función inicio.

- Al final se encuentran dos lineas de código que son para asegurarnos de que ha comenzado el programa, utilizando el led que viene integrado en la placa de la raspberry pi pico. 
        
    - Esto lo logramos utilizando el objeto *Pin* que importamos y las funciones que tiene incluido micropython, ya que la forma en la que se crea un objeto Pin (en este caso Led) es dando al menos 2 parámetros un *id* o número de pin y si sera entrada o salida, sin embargo si al momento de crear un objeto *Pin* en lugar de poner un número de pin escribimos la palabra *LED* automáticamente seleccionara el numero de puerto que esta conectado internamente al led que esta integrado a la placa.
```python
if __name__ == "__main__":
    uart = UART(0, 115200, tx=Pin(0), rx=Pin(1)) # Crea un objeto UART
    uart.init(bits=8, parity=None, stop=1) # Inicializa el puerto serial

    i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000) # Crea un objeto I2C
    print("I2C inicializándose...")
    sleep(100) 
    print("I2C inicializado")
    imu = MPU6050(i2c) # Crea un objeto MPU6050

    address = input('Direccion: ')
```
- En este bloque nos encontramos con el inicio del código principal y con la parte de configuración, en la explicación del codigo de *testTelemetria.py* explique como se configuraba el puerto UART por lo que pasare directamente a explicar acerca de el *I2C*

- Al igual que con el protocolo serial UART, para configurar nuestro I2C es necesario especificar que número de I2C usaremos ya que la raspberry pi pico cuenta con I2C0 e I2C1, y cada uno cuenta con 12 pares de pines dedicados para ese I2C, en este código se decidió usar *I2C1* con la combinación de pines para *sda* (serial data) *Pin(2)* y para *scl* (serial clock) *Pin(3)* a una frecuencia de 400kHz u otra forma de decirlo con una velocidad de reloj de 400kHz
    - Tras crear el objeto I2C es recomendable esperar un minuto con 40 segundos (o 100 segundos) para darle tiempo al bus de I2C que inicie correctamente y que scanee todos los dispositivos conectados. Una vez que esperamos podemos crear un objeto de tipo *MPU6050* al que llamaremos *imu* para que sea mas corto de escribir. 

```python
    while True: 

        # Obtener los valores de los sensores
        ax=str(round(imu.accel.x,2))
        ay=str(round(imu.accel.y,2))
        az=str(round(imu.accel.z,2))
        
        gx=str(round(imu.gyro.x))
        gy=str(round(imu.gyro.y))
        gz=str(round(imu.gyro.z))
        
        tem=str(round(imu.temperature,2))

        # Imprimir los valores de los sensores
        print("ax",ax,"\t","ay",ay,"\t","az",az,"\t","gx",gx,"\t","gy",gy,"\t","gz",gz,"\t","Temperature",tem,"        ",end="\r")
        data = "ax="+ax+"ay="+ay+"az="+az+"gx="+gx+"gy="+gy+"gz="+gz+"Temperature="+tem # Concatenar los valores de los sensores en una cadena de caracteres
        inicio(data,address) # Llama a la función inicio
        sleep(0.2)
```
- Finalmente llegamos al ciclo infinito, en el cuál el primer paso sera obtener la información del sensor mediante el objeto *imu*. Es importante que notemos que cada variable que obtenemos esta redondeada a 2 decimales y convertida a string para poder mandarlo por la función que creamos.
    - La primer variable que obtenemos es la aceleración del sensor, esta se nos presenta en sus componentes cartesianos de *X, Y y Z*.
    - De igual forma obtenemos la posición del sensor a traves del giroscopio dividido en *X, Y y Z*
    - Finalmente obtenemos la temperatura del sensor.

- Tras obtener la información del sensor, lo imprimimos en la terminal para conocer cuales fueron los datos obtenidos, tras eso concatenamos todas las variables, es decir hacemos una sola cadena de caracteres y se la mandamos a la función *inicio* junto con la dirección que especificamos antes.

- Al final hay una instrucción que hace que espere el microcontrolador 200 milisegundos para volver a obtener lecturas y mandarlas por el módulo RYLR998.

