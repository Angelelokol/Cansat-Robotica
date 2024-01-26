"""
Librería para enviar comandos al modulo RYLR998

Autor: Angel Ivan Aboytes Hernández
Fecha: 2023

Funciones:
    inicio(coms):
        Envía el comando AT+SEND=dirección, longitud del mensaje, mensaje
        Ejemplo: coms = "Hola mundo"
                    inicio(coms)
                    >> AT+SEND=2,10,Hola mundo
"""

from machine import Pin, UART
from time import sleep_ms

def inicio(coms, address="2"):
    length = str(len(coms))
    uart.write("AT+SEND="+address+","+length+","+coms+"\r\n")
    
