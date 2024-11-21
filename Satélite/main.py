import network
import socket
from machine import Pin, I2C, ADC, PWM
import time
from dht import DHT11
from imu import MPU6050
from bmp280 import *


# Configuración WiFi
ssid = 'PicoNano'
password = 'wkax0081'

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

# Espera a la conexión WiFi
while not wlan.isconnected():
    time.sleep(0.5)

print('Conectado a WiFi:', wlan.ifconfig())

# Configuración de sensores
i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=400000)
imu = MPU6050(i2c)
sensor_temp = ADC(4)
ldr = ADC(28)
buzzer = Pin(11, Pin.OUT)
bus = I2C(0, sda=Pin(0), scl=Pin(1))
bmp = BMP280(bus)
pin = Pin(2, Pin.IN, Pin.PULL_UP)
dht11 = DHT11(pin)

# Configuración del servo
servo = PWM(Pin(15))  # Cambia el pin si es necesario
servo.freq(50)        # Frecuencia estándar para el SG90: 50 Hz

# Interruptores magnéticos
interruptor_1 = Pin(5, Pin.IN, Pin.PULL_UP)
interruptor_2 = Pin(6, Pin.IN, Pin.PULL_UP)

# Variables de estado
paracaidas_abierto = False
sensando = False
altitud_max = float('-inf')
altitud_min = float('inf')
tiempo_inicio = 0
tiempo_transcurrido = 0
angulo_servo = 0

# Función para mover el servo a un ángulo específico
def mover_servo(angulo):
    global angulo_servo
    if 0 <= angulo <= 180:
        if angulo_servo != angulo:
            duty = int((angulo / 180) * (7500 - 2500) + 2500)
            servo.duty_u16(duty)
            angulo_servo = angulo
    else:
        print("Angulo fuera de rango. Debe estar entre 0 y 180 grados.")

# Función para calcular la altitud a partir de la presión atmosférica
def calcular_altitud(presion):
    return 44330 * (1 - (presion / 101325) ** 0.1903)

# Formatear el tiempo
def formatear_tiempo(segundos):
    minutos = int(segundos // 60)
    segundos = int(segundos % 60)
    return f"{minutos:02}:{segundos:02}"

# Obtener datos de los sensores
def obtener_datos_sensores():
    ax = round(imu.accel.x, 2)
    ay = round(imu.accel.y, 2)
    az = round(imu.accel.z, 2)
    gx = round(imu.gyro.x, 2)
    gy = round(imu.gyro.y, 2)
    gz = round(imu.gyro.z, 2)

    valor = sensor_temp.read_u16() * 3.3 / 65535
    temp_cpu = round(27 - (valor - 0.706) / 0.001721, 2)

    lucecita = ldr.read_u16()
    luz = round((lucecita / 65535) * 100, 2)

    T = round(bmp.temperature, 2)
    P = round(bmp.pressure, 2)
    At = round(P / 101300, 2)

    dht11.measure()
    H = round(dht11.humidity(), 2)
    
    return ax, ay, az, gx, gy, gz, temp_cpu, luz, T, P, At, H
    

# Activar buzzer
def activar_buzzer(veces, duracion):
    for _ in range(veces):
        buzzer.on()
        time.sleep(duracion)
        buzzer.off()
        time.sleep(duracion)

# Función para manejar la interrupción
def manejar_interrupcion(pin):
    global paracaidas_abierto
    # Leer el estado invertido del pin
    if not pin.value() and not paracaidas_abierto:  # Detecta el estado "NOT"
        mover_servo(90)  # Desplegar paracaídas
        activar_buzzer(5, 0.1)
        paracaidas_abierto = True
        print("Paracaídas desplegado debido a interrupción.")

# Configurar la interrupción para el estado NOT (borde descendente)
interruptor_2.irq(trigger=Pin.IRQ_FALLING, handler=manejar_interrupcion)

# Generar la página web con los datos de los sensores
def generar_pagina_web(tiempo_formateado):
    global angulo_servo, paracaidas_abierto, sensando, altitud_max, altitud_min

    ax, ay, az, gx, gy, gz, temp_cpu, luz, T, P, At, H = obtener_datos_sensores()

    # Calcular altitud y actualizar máxima y mínima
    altitud = calcular_altitud(P)
    if altitud > altitud_max:
        altitud_max = altitud
    if altitud < altitud_min:
        altitud_min = altitud
     
    estado_sensado = "Sensando" if sensando else "No esta sensando"

    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
    <title>Sensor Data</title>
    <meta http-equiv="refresh" content="2">
    </head>
    <body>
    <h1>Datos de Sensores</h1>
    <p>Estado: {estado_sensado}</p>
    <p>Tiempo: {tiempo_formateado}</p>
    <p>Ax = {ax:.2f}, Ay = {ay:.2f}, Az = {az:.2f}</p>
    <p>Gx = {gx:.2f}, Gy = {gy:.2f}, Gz = {gz:.2f}</p>
    <p>Temperatura CPU: {temp_cpu:.2f}C</p>
    <p>Temperatura Ambiente: {T:.2f}C</p>
    <p>Humedad: {H:.2f}%</p>
    <p>Presion Atmosferica: {P:.2f} Pa << {At:.2f} atm</p>
    <p>Luz Solar: {luz:.2f}%</p>
    <p>Altitud Maxima: {altitud_max:.2f} m</p>
    <p>Altitud Minima: {altitud_min:.2f} m</p>
    <p>Paracaidas desplegado: {'Si' if paracaidas_abierto else 'No'}</p>
    <p>Angulo del servo: {angulo_servo} grados</p>
    <form method="get">
        <button name="action" value="reset">Reiniciar</button>
        <button name="action" value="start">Iniciar Sensado</button>
        <button name="action" value="pause">Pausar Sensado</button>
        <button name="action" value="servo0">Servo a 0 grados</button>
        <button name="action" value="servo90">Servo a 90 grados</button>
    </form>
    </body></html>
    """
    return html

# Manejar solicitudes HTTP
def manejar_solicitudes():
    global sensando, angulo_servo, paracaidas_abierto, tiempo_inicio, tiempo_transcurrido

    # Pitidos iniciales
    activar_buzzer(3, 0.2)

    s = socket.socket()
    s.bind(('0.0.0.0', 80))
    s.listen(1)

    while True:
        conn, addr = s.accept()
        print('Conexión de', addr)
        request = conn.recv(1024)
        request = str(request)

        # Procesar acciones de los botones
        if 'action' in request:
            accion = request.split('action=')[1].split(' ')[0]
            if accion == 'reset':
                print("Reiniciando...")
                sensando = False
                paracaidas_abierto = False
                angulo_servo = 0
                tiempo_transcurrido = 0
                tiempo_inicio = 0
            elif accion == 'start':
                if not sensando:  # Inicia solo si no está sensando
                    activar_buzzer(1, 0.1)
                    tiempo_inicio = time.time() - tiempo_transcurrido
                    sensando = True
            elif accion == 'pause':
                if sensando:  # Pausa solo si está sensando
                    activar_buzzer(1, 0.2)
                    tiempo_transcurrido = time.time() - tiempo_inicio
                    sensando = False
            elif accion == 'servo0':
                mover_servo(0)
            elif accion == 'servo90':
                mover_servo(90)


        # Actualizar tiempo y generar la página web
        tiempo_formateado = formatear_tiempo(time.time() - tiempo_inicio) if sensando else formatear_tiempo(tiempo_transcurrido)
        pagina = generar_pagina_web(tiempo_formateado)

        conn.send('HTTP/1.1 200 OK\nContent-Type: text/html\n\n')
        conn.sendall(pagina)
        conn.close()

# Iniciar el servidor
manejar_solicitudes()