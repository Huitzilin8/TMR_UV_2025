# control/serial_comm.py
import serial
import time

class ESP32Controller:
    def __init__(self, port='/dev/ttyTHS1', baud=115200):
        self.u = serial.Serial(port, baud, timeout=1)
        time.sleep(2)

    def send_direction(self, angle_norm):
        # angle_norm: -1.0 … +1.0
        self.u.write(f"DIR:{angle_norm:.2f}\n".encode())

    def send_action(self, cmd, val=0.0):
        # cmd en {'FORWARD','REVERSE','STOP'}
        self.u.write(f"ACT:{cmd}:{val:.2f}\n".encode())

    def close(self):
        self.u.close()
