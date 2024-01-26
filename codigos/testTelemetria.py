"""
Este código es para probar la telemetría del modulo RYLR998
Autor: Angel Ivan Aboytes Hernández
Fecha: 2023

Funciones:
    inicio(coms,address="2"):
        Envía el comando AT+SEND=2, longitud del mensaje, mensaje
        Ejemplo: coms = "Hola mundo"
                 address = "1"
                    inicio(coms, address)
                    >> AT+SEND=1,10,Hola mundo
        Ejemplo2: coms = "Hola mundo"
                    inicio(coms)
                    >> AT+SEND=2,10,Hola mundo


"""


from machine import Pin, UART
from time import sleep_ms

def inicio(coms,address="2"):
    """
    Recibe como parámetros el mensaje y la dirección a la que se enviará el mensaje
    Función para enviar el comando AT+SEND
    """
    length = str(len(coms)) # Convierte la longitud del mensaje a una cadena de caracteres
    uart.write("AT+SEND="+address+length+coms+"\r\n") # Escribe el comando AT con los parámetros dados en el puerto serial 
    msg = uart.readline() # Lee la respuesta del modulo RYLR998
    print(msg) # Imprime en la terminal la respuesta del modulo RYLR998

if __name__ == "__main__":
    uart = UART(0, 115200, tx=Pin(0), rx=Pin(1)) # Crea un objeto UART 
    uart.init(bits=8, parity=None, stop=1) # Inicializa el puerto serial
    count = 0 # Variable para contar los mensajes recibidos
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
    
#SF9=9,Band=7/125khz, CodingRate=1,12
            
# str.lower(input(' ')) Es una instrucción que recibe una cadena de caracteres y la convierte a minúsculas