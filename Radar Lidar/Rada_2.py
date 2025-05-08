import math
import sys
from rplidar import RPLidar
import numpy as np
import threading
import time

# Configuración del puerto - ajusta según tu sistema
PORT_NAME = '/dev/ttyUSB0'  # En Windows podría ser 'COM3', etc.

class LidarVisualizer:
    def __init__(self, width=500, height=500):
        # Configuración del LIDAR
        self.lidar = None
        self.scan_data = []
        self.max_distance = 6000  # mm
        self.width = width
        self.height = height
        self.scale_factor = min(width, height) / 2 / self.max_distance

        # Variables para el mapeo
        self.occupancy_grid = np.zeros((width, height), dtype=np.uint8)
        self.grid_resolution = 10  # mm por píxel

        # Iniciar conexión con el LIDAR
        self.connect_lidar()

        # Iniciar hilo de recolección de datos
        self.running = True
        self.data_thread = threading.Thread(target=self.collect_data_continuously, daemon=True)
        self.data_thread.start()
        self.alert_status = None

    def connect_lidar(self):
        try:
            self.lidar = RPLidar(PORT_NAME)
            print("Conectado al LIDAR. Iniciando motor...")

            # Obtener información del dispositivo
            info = self.lidar.get_info()
            print(f"Información del dispositivo:")
            print(f"  Modelo: {info.model}")
            print(f"  Firmware: {info.firmware}")
            print(f"  Hardware: {info.hardware}")
            print(f"  Número de serie: {info.serialnumber}")

            # Obtener estado de salud
            health = self.lidar.get_health()
            print(f"Estado de salud: {health.status}, Código de error: {health.error_code}")

            # Iniciar motor
            self.lidar.start_motor()

        except Exception as e:
            print(f"Error al conectar con el LIDAR: {e}")
            sys.exit(1)

    def collect_data_continuously(self):
        while self.running:
            try:
                # Limpiar datos anteriores
                self.scan_data = []

                # Obtener un escaneo completo (una vuelta)
                for i, scan_point in enumerate(self.lidar.iter_scan()):
                    if not self.running:
                        break
                    self.scan_data.append(scan_point)
                    if i > 1000:  # Ajustar según la resolución deseada
                        break

                # Actualizar el mapa de ocupación
                self.update_occupancy_grid()

                time.sleep(0.05)  # Breve pausa entre escaneos

            except Exception as e:
                print(f"Error durante el escaneo: {e}")
                time.sleep(1)  # Esperar antes de reintentar

    def update_occupancy_grid(self):
        # Limpiar el grid (0 = libre, 1 = ocupado)
        self.occupancy_grid.fill(0)

        # Centro de la pantalla (origen del LIDAR)
        center_x, center_y = self.width // 2, self.height // 2

        for point in self.scan_data:
            if point.distance > 0:
                # Convertir coordenadas polares a cartesianas
                angle_rad = math.radians(point.angle)
                distance_px = point.distance / self.grid_resolution

                x = int(center_x + distance_px * math.sin(angle_rad))
                y = int(center_y - distance_px * math.cos(angle_rad))

                # Asegurarse de que las coordenadas están dentro de los límites
                if 0 <= x < self.width and 0 <= y < self.height:
                    self.occupancy_grid[x, y] = 1

    def polar_to_cartesian(self, angle, distance):
        """Convierte coordenadas polares a cartesianas centradas en la pantalla"""
        x = self.width // 2 + distance * self.scale_factor * math.sin(math.radians(angle))
        y = self.height // 2 - distance * self.scale_factor * math.cos(math.radians(angle))
        return int(x), int(y)

    def estimate_lidar_data(self):

        # Variables para alertas
        alert_medium = False
        alert_critical = False

        # Dibujar puntos del LIDAR
        for point in self.scan_data:
            if point.distance > 0:
                x, y = self.polar_to_cartesian(point.angle, point.distance)
                # Colorear según la distancia
                if point.distance < 200:
                    alert_critical = True
                elif point.distance < 1000:
                    alert_medium = True
        
        if alert_critical:
            self.alert_status = "ALERTA CRÍTICA"
        elif alert_medium:
            self.alert_status = "ALERTA MEDIA"
        else:
            self.alert_status = None

    def run(self):
        while self.running:
            self.estimate_lidar_data()
            if self.alert_status is not None:
                print(f"ALERTA: {self.alert_status}")
            else:
                print(f"Puntos encontrados: {len(self.scan_data)}")

            time.sleep(0.1) 

        self.cleanup()

    def cleanup(self):
        print("Deteniendo el programa...")
        self.running = False

        if self.lidar:
            print("Deteniendo motor y desconectando...")
            self.lidar.stop_motor()
            self.lidar.disconnect()

        sys.exit()

if __name__ == '__main__':
    visualizer = LidarVisualizer()
    visualizer.run()
