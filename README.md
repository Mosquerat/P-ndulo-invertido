Documentacion para el pendulo invertido de las practicas de teoria de control

    Introduccion En este documento se recoge la creación de un primer prototipo de un robot tipo "pendulo invertido”.

    Hardware

2.1. Microcontrolador: ESP32

2.2. Sensor de velocidad: MPU-6050

2.3. Controlador (driver) TB6612FNG

2.4. Bateria: Bateria Lipo

2.5. Motores y ruedas: Usamos los motores "de aficionado" de 3 a 6v con reductora

2.6. PCB:

Usamos el software kicad para diseñar tanto el esquematico como el pcb, el resultado de ambos en los archivos esquematico.jpg y pcb.jpg

    Chasis

Diseñamos un chasis minimalista con 4 agujeros cuadrados para pasar el cable desde la bateria (parte inferior), y 4 circulares en cada esquina para instalar un segundo piso con varillas (esto ultimo no fue posible). El diseño del chasis se puede observar en el archivo chasis.jpg, hecho en software fusion360.

    Codigo:

No tuvimos tiempo de probar el codigo debidamente, sin embargo, lo cuelgo en el archivo, pendulo_pid.cpp para lectura de parte del profesor.
