import subprocess
import sys
import os
import threading
import queue
import time

# Función para ejecutar el script de detección de carriles (en Python 2.7)
def run_lane_detection():
    """Ejecutar el script de detección de carriles usando Python 2.7."""
    subprocess.Popen([sys.executable.replace('python3', 'python2.7'), "lane_detection.py"])  # Ejecutar con Python 2.7

# Función para ejecutar el script de comunicación serial (en Python 3)
def run_serial_comm():
    """Ejecutar el script de comunicación serial usando Python 3."""
    subprocess.Popen([sys.executable.replace('python2.7', 'python3'), "serial_comm.py"])  # Ejecutar con Python 3

# Función para procesar los datos de LiDAR
def process_lidar_data():
    """Leer datos de LiDAR y enviar comandos al ESP32 (para mover el vehículo si es necesario)."""
    while True:
        # Aquí va el código para procesar datos de LiDAR (ver el script LiDAR)
        # Lo ideal sería leer un archivo o puerto serial para obtener los datos de LiDAR
        # Luego, basándonos en esos datos, enviar comandos a ESP32 (similar a los carriles).
        time.sleep(1)  # Pausa entre lecturas

# Función principal que gestiona la cola de datos y llama a las funciones correspondientes
def main():
    # Crear una cola para pasar los datos entre procesos
    data_queue = queue.Queue()

    # Iniciar los procesos en paralelo
    threading.Thread(target=run_lane_detection, daemon=True).start()  # Ejecutar detección de carriles
    threading.Thread(target=run_serial_comm, daemon=True).start()     # Ejecutar comunicación serial
    threading.Thread(target=process_lidar_data, daemon=True).start()  # Ejecutar procesamiento de LiDAR

    while True:
        # Lógica principal del control (sin que bloquee el flujo)
        # Por ejemplo, verificar el estado del vehículo, procesar los datos de los carriles y LiDAR
        time.sleep(1)  # Hacer una pequeña pausa para no sobrecargar la CPU

if __name__ == "__main__":
    main()
