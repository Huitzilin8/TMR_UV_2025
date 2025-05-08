import pygame
import math
import sys
from rplidar import RPLidar
import numpy as np
import threading
import time

# Configuración del puerto - ajusta según tu sistema
PORT_NAME = 'COM3'  # En Windows podría ser 'COM3', etc.

class LidarVisualizer:
    def __init__(self, width=500, height=500):
        # Inicializar Pygame
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("RPLIDAR C1 - Visualización en Tiempo Real")

        # Colores
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.YELLOW = (255, 255, 0)

        # Configuración del LIDAR
        self.lidar = None
        self.scan_data = []
        self.max_distance = 6000  # mm
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

    def draw_lidar_data(self):
        # Dibujar fondo
        self.screen.fill(self.BLACK)

        # Dibujar rejilla polar
        self.draw_polar_grid()

        # Variables para alertas
        alert_medium = False
        alert_critical = False

        # Dibujar puntos del LIDAR
        for point in self.scan_data:
            if point.distance > 0:
                x, y = self.polar_to_cartesian(point.angle, point.distance)

                # Colorear según la distancia
                if point.distance < 200:
                    color = self.RED
                    alert_critical = True
                elif point.distance < 1000:
                    color = self.YELLOW
                    alert_medium = True
                else:
                    color = self.GREEN

                pygame.draw.circle(self.screen, color, (x, y), 2)

        # Mostrar alertas en pantalla
        font_alert = pygame.font.SysFont('Arial', 24)
        if alert_critical:
            alert_text = font_alert.render('¡PELIGRO! Obstáculo muy cercano', True, self.RED)
            self.screen.blit(alert_text, (10, 40))
        elif alert_medium:
            alert_text = font_alert.render('Advertencia: Obstáculo cercano', True, self.YELLOW)
            self.screen.blit(alert_text, (10, 40))

        # Mostrar información general
        font_info = pygame.font.SysFont('Arial', 16)
        info_text = f"Puntos: {len(self.scan_data)} | Escala: 1px = {1/self.scale_factor:.1f}mm"
        info_surface = font_info.render(info_text, True, self.WHITE)
        self.screen.blit(info_surface, (10, 10))

        pygame.display.flip()

    def draw_polar_grid(self):
        center_x, center_y = self.width // 2, self.height // 2

        # Dibujar círculos concéntricos
        for r in range(1000, self.max_distance, 1000):
            radius = int(r * self.scale_factor)
            pygame.draw.circle(self.screen, (50, 50, 50), (center_x, center_y), radius, 1)

        # Dibujar líneas radiales cada 30 grados
        for angle in range(0, 360, 30):
            end_x = center_x + self.max_distance * self.scale_factor * math.sin(math.radians(angle))
            end_y = center_y - self.max_distance * self.scale_factor * math.cos(math.radians(angle))
            pygame.draw.line(self.screen, (50, 50, 50), (center_x, center_y), (end_x, end_y), 1)

    def run(self):
        clock = pygame.time.Clock()

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.draw_lidar_data()
            clock.tick(30)  # Limitar a 30 FPS

        self.cleanup()

    def cleanup(self):
        print("Deteniendo el programa...")
        self.running = False

        if self.lidar:
            print("Deteniendo motor y desconectando...")
            self.lidar.stop_motor()
            self.lidar.disconnect()

        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    visualizer = LidarVisualizer()
    visualizer.run()
