# Codigo en Micropython para ESP32

from machine import UART, Pin, PWM
import time

uart = UART(1, baudrate=115200, tx=17, rx=16)

# Servo dirección
servo = PWM(Pin(13), freq=50)
def set_servo(angle_deg):
    dc = int(40 + (115-40)*(angle_deg/180))
    servo.duty(dc)

# Motor DC (L298N)
in1 = Pin(25, Pin.OUT); in2 = Pin(26, Pin.OUT)
en  = PWM(Pin(27), freq=1000)
def set_motor(dirc, speed):
    if dirc=='FORWARD': in1.on(); in2.off(); en.duty(int(speed*1023))
    elif dirc=='REVERSE': in1.off(); in2.on(); en.duty(int(speed*1023))
    else: in1.off(); in2.off(); en.duty(0)

while True:
    if uart.any():
        line = uart.readline().decode().strip()
        parts = line.split(':')
        if parts[0]=='DIR':
            ang = float(parts[1])
            set_servo((ang+1)*90)
        elif parts[0]=='ACT':
            cmd, val = parts[1], float(parts[2])
            set_motor(cmd, val)
    time.sleep(0.01)
