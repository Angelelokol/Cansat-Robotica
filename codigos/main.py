"""
Este es el programa principal del proyecto. En este se inicializan los puertos
y se ejecutan las funciones de las librerías para enviar los datos de los sensores
al modulo RYLR998

Autor: Angel Ivan Aboytes Hernández
Basado en el código de: Shille (https://www.instructables.com/How-to-Use-MPU6050-With-Raspberry-Pi-Pico-or-Pico-/)

Fecha: 2023

"""

from imu import MPU6050
from time import sleep
from machine import Pin, I2C
from send import inicio

#Shows Pi is on by turning on LED when plugged in
LED = machine.Pin("LED", machine.Pin.OUT) # Crea un objeto LED
LED.on()

if __name__ == "__main__":
    uart = UART(0, 115200, tx=Pin(0), rx=Pin(1)) # Crea un objeto UART
    uart.init(bits=8, parity=None, stop=1) # Inicializa el puerto serial

    i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000) # Crea un objeto I2C
    print("I2C inicializándose...")
    sleep(100) 
    print("I2C inicializado")
    imu = MPU6050(i2c) # Crea un objeto MPU6050

    address = input('Direccion: ')
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