import serial

class ESP32Controller:
    def __init__(self, port='/dev/ttyTHS1', baud=115200):
        self.esp = serial.Serial(port, baud, timeout=1)

    def send_direction(self, correction):
        if correction is not None:
            command = f"DIR:{correction}\n"
            self.esp.write(command.encode())

    def send_action(self, action):
        command = f"ACT:{action}\n"
        self.esp.write(command.encode())

    def close(self):
        self.esp.close()
