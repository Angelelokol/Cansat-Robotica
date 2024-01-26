# Instalación de Librerías en Thonny
En este texto encontraras 3 métodos para la instalación de librerías para micropython en la raspberry pi pico. 
## Método sencillo salvar utilizando el IDE

1. Da click a la opción de abrir fichero y selecciona este computador

    ![Abrir_Fichero](/resources/images/getting%20started/image-1.png)
    ![Este_Computador](/resources/images/getting%20started/image-2.png)

2. Encuentra la librería descargada y ábrela
3. Da click en fichero, después a guardar como

    ![Fichero](/resources/images/getting%20started/image-4.png)
    ![Guardar_como](/resources/images/getting%20started/image-5.png)

4. Aparecerá una ventana preguntando a donde guardar, seleccionamos *Dispositivo MicroPython* 

    ![Donde_Guardar](/resources/images/getting%20started/image-6.png)

5. Saldrá una nueva ventana que nos pedirá darle un nombre al archivo y nos mostrara los archivos que se encuentran en la memoria Flash de nuestro microcontrolador. En este caso estamos guardando la librería vector3d.py por lo que lo guardaremos bajo ese nombre.

    ![Name](/resources/images/getting%20started/image-7.png)

6. Y listo tenemos guardada nuestra librería en la raspberry

## Método avanzado utilizando archivos por interfaz

Otra forma en la que podemos guardar códigos o librerías dentro de la raspberry pi pico es utilizando el sistema de archivos que tiene Thonny utilizando micropython

1. Abre thonny y cambia a modo regular.

    ![modo regular](/resources/images/code/image.png)

2. Te pedirá reiniciar el IDE, al volverlo abrir habrán aparecido nuevos menus como *File, Edit, View, Etc.* damos click a *View*

    ![view](/resources/images/code/image-1.png)

3. se desplegara un menu en donde daremos click en *Files*

    ![files](/resources/images/code/image-2.png)

4. Observaremos que del lado izquierdo se despliega un nuevo menu donde encontramos los archivos que tenemos en la raspberry y en nuestro computador.

    ![fpi & fpc](/resources/images/code/image-3.png)

5. Ahora bastara con dar click derecho y seleccionar *Upload to/* en la computadora y se cargaran los archivos a la raspberry. De igual forma si en la raspberry damos click derecho podremos descargar los archivos a la 

    <video src="20240125-1920-48.2598682.mp4" controls title="UploadDownload"></video>

