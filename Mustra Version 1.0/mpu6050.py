from imu import MPU6050
import time
from machine import Pin, I2C

# Usamos pines GP26 (SDA) y GP27 (SCL) en I2C1
i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=400000)
imu = MPU6050(i2c)

while True:
    print('Ax=', imu.accel.x, ", Ay=", imu.accel.y, ", Az=", imu.accel.z, "\tGx=", imu.gyro.x, ", Gy=", imu.gyro.y, ", Gz=", imu.gyro.z)
    # Retardo de 0.3 segundos para estabilizar el dato leído
    time.sleep(0.3)
