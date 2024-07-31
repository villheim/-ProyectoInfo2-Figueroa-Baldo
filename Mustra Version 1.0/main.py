import network
import socket
from machine import Pin, I2C, ADC
import time
from imu import MPU6050
from dht import DHT11

# Configurar la conexión WiFi
ssid = 'PicoNano'
password = 'wkax0081'

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

# Esperar a que la conexión WiFi esté lista
while not wlan.isconnected():
    pass

print('Conectado a WiFi:', wlan.ifconfig())

# Configurar sensores
i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=400000)
imu = MPU6050(i2c)
sensor_temp = ADC(4)
led = Pin("LED", Pin.OUT)
ldr = ADC(28)
pin = Pin(20, Pin.IN, Pin.PULL_UP)
dht11 = DHT11(pin, None, dht11=True)
vibration_sensor = Pin(15, Pin.IN)

# Crear la página web
def web_page():
    ax = imu.accel.x
    ay = imu.accel.y
    az = imu.accel.z
    gx = imu.gyro.x
    gy = imu.gyro.y
    gz = imu.gyro.z

    valor = sensor_temp.read_u16() * 3.3 / 65535
    temp_cpu = 27 - (valor - 0.706) / 0.001721

    if temp_cpu >= 22:
        led.value(1)
    else:
        led.value(0)

    lucecita = ldr.read_u16()
    luz = round(lucecita / 65535 * 100)

    T, H = dht11.read()
    
    vibration = vibration_sensor.value()
    vibration_status = "Detectada" if vibration == 1 else "No detectada"
    
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
    <title>Sensor Data</title>
    <meta http-equiv="refresh" content="2">
    </head>
    <body>
    <h1>Datos de Sensores</h1>
    <p>Ax = {ax}, Ay = {ay}, Az = {az}</p>
    <p>Gx = {gx}, Gy = {gy}, Gz = {gz}</p>
    <p>Temperatura CPU: {temp_cpu}°C</p>
    <p>Valor LDR: {lucecita}</p>
    <p>Luz Solar: {luz}%</p>
    <p>Temperatura: {T}°C</p>
    <p>Humedad: {H}%</p>
    <p>Vibración: {vibration_status}</p>
    </body>
    </html>
    """
    return html

# Crear el socket y esperar conexiones
try:
    addr = socket.getaddrinfo('0.0.0.0', 8081)[0][-1]
    s = socket.socket()
    s.bind(addr)
    s.listen(1)
    print('Escuchando en', addr)

    while True:
        cl, addr = s.accept()
        print('Cliente conectado desde', addr)
        cl_file = cl.makefile('rwb', 0)
        while True:
            line = cl_file.readline()
            if not line or line == b'\r\n':
                break
        response = web_page()
        cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
        cl.send(response)
        cl.close()
except OSError as e:
    print('Error al crear el socket:', e)
finally:
    if 's' in locals():
        s.close()
