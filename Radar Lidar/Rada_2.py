import math
import sys
import threading
import time
import queue  # <--- Añadir import
from rplidar import RPLidar
# Quitamos numpy si ya no hacemos el mapeo aquí
# import numpy as np

# Configuración del puerto - puede pasarse al __init__ o dejarse global si prefieres
# PORT_NAME = 'COM3'

class LidarController: # <--- Renombrar clase
    """
    Controla la conexión, recolección de datos y detención del LIDAR RPLidar.
    Diseñado para ser ejecutado en un hilo gestionado externamente.
    """
    # Quitar parámetros de visualización del __init__ si no se usan aquí
    def __init__(self, port='COM3', data_queue=None, stop_event=None):
        """
        Inicializa el controlador LIDAR.

        Args:
            port (str): El puerto serial donde está conectado el LIDAR (ej. 'COM3' o '/dev/ttyUSB0').
            data_queue (queue.Queue): La cola donde se pondrán los datos del escaneo.
            stop_event (threading.Event): El evento que señalará cuándo detener la recolección.
        """
        self.port = port
        self.lidar = None
        self.scan_data = []  # Podrías quitar esto si sólo envías a la cola

        # --- Argumentos para control y comunicación ---
        # Si no se proporcionan, crear unos por defecto (aunque lo normal es que se pasen)
        self.data_queue = data_queue if data_queue is not None else queue.Queue()
        self.stop_event = stop_event if stop_event is not None else threading.Event()
        # ----------------------------------------------

        self.data_thread = None # Hilo que ejecutará _collect_data_loop
        self.is_collecting = False # Flag para controlar el bucle interno

        # --- Quitar la conexión y el inicio del hilo del __init__ ---
        # self.connect_lidar() # NO conectar aquí
        # self.data_thread = threading.Thread(target=self.collect_data_continuously, daemon=True) # NO iniciar hilo aquí
        # self.data_thread.start() # NO iniciar hilo aquí

    def connect_lidar(self):
        """Intenta conectar con el LIDAR e iniciar el motor."""
        if self.lidar is not None:
            print("[LidarCtrl] Ya conectado.")
            return True
        try:
            print(f"[LidarCtrl] Conectando a LIDAR en {self.port}...")
            self.lidar = RPLidar(self.port)
            # Opcional: Obtener info y health si lo necesitas
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            print(f"[LidarCtrl] Info: {info}")
            print(f"[LidarCtrl] Salud: {health}")
            if health.status != 'Good':
                 print(f"[LidarCtrl] ADVERTENCIA: Estado de salud del LIDAR no es 'Good': {health.status}")
                 # Podrías decidir no continuar si la salud es mala
                 # self.lidar.disconnect()
                 # self.lidar = None
                 # return False
            print("[LidarCtrl] Iniciando motor...")
            self.lidar.start_motor()
            print("[LidarCtrl] Conexión exitosa y motor iniciado.")
            return True
        except Exception as e:
            print(f"[LidarCtrl] Error al conectar/iniciar LIDAR en {self.port}: {e}")
            if self.lidar:
                self.lidar.disconnect() # Intenta desconectar si se creó el objeto
            self.lidar = None
            return False

    def _collect_data_loop(self):
        """
        Bucle ejecutado en un hilo separado para recolectar datos continuamente.
        Esta función es el 'target' del self.data_thread.
        Utiliza self.stop_event y self.data_queue pasados en __init__.
        """
        self.is_collecting = True
        print("[LidarCtrl-Thread] Iniciando bucle de recolección...")
        iterator = None # Definir fuera del try para el finally

        try:
            # Asegurarse que el LIDAR está listo
            if self.lidar is None:
                 print("[LidarCtrl-Thread] Error: LIDAR no conectado al iniciar recolección.")
                 self.is_collecting = False
                 return # Salir del hilo

            # max_buf_meas controla cuántas mediciones puede almacenar antes de descartar viejas
            # Un valor más alto puede ayudar si el procesamiento es lento, pero consume más memoria.
            iterator = self.lidar.iter_scans(max_buf_meas=5000)

            while self.is_collecting and not self.stop_event.is_set():
                try:
                    # next(iterator) bloquea hasta que se complete una vuelta (360 grados)
                    scan = next(iterator)
                    # scan es una lista de tuplas: (quality, angle, distance)

                    # Filtrar puntos inválidos (distancia 0) si es necesario
                    # (Aunque RPLidar ya suele hacerlo, no está de más verificar)
                    valid_scan = [(q, a, d) for q, a, d in scan if d > 0]

                    # self.scan_data = valid_scan # Opcional: guardar localmente

                    # --- Enviar datos a la cola ---
                    if self.data_queue is not None and valid_scan:
                        try:
                            # Poner en la cola sin bloquear. Si está llena, se perderá el dato.
                            self.data_queue.put({'tipo': 'lidar', 'datos': valid_scan}, block=False)
                        except queue.Full:
                            print("[LidarCtrl-Thread] ADVERTENCIA: Cola de datos LIDAR llena. Descartando escaneo.")
                            pass # Continuar sin el dato
                    # -----------------------------

                except StopIteration:
                    print("[LidarCtrl-Thread] iter_scans detenido inesperadamente.")
                    self.is_collecting = False # Detener el bucle
                    break
                except Exception as e:
                    print(f"[LidarCtrl-Thread] Error durante escaneo: {e}")
                    # Aquí podrías intentar reiniciar el iterador o manejar el error
                    time.sleep(1) # Esperar antes de reintentar

        except Exception as e:
            print(f"[LidarCtrl-Thread] Error crítico en hilo de recolección: {e}")
        finally:
            self.is_collecting = False
            print("[LidarCtrl-Thread] Bucle de recolección finalizado.")
            # NO llamar a self.stop() desde aquí para evitar deadlock

    # --- ESTE ES EL MÉTODO CLAVE ---
    def start_collection(self):
        """
        Inicia la conexión (si no está hecha) y el hilo de recolección de datos.
        Esta función SÍ usa self.stop_event y self.data_queue (pasados en __init__).

        Returns:
            bool: True si la recolección se inició correctamente, False en caso contrario.
        """
        # 1. Verificar si ya está corriendo
        if self.is_collecting or (self.data_thread and self.data_thread.is_alive()):
            print("[LidarCtrl] La recolección ya está en curso.")
            return True # Ya está iniciado

        # 2. Conectar si es necesario
        if self.lidar is None:
            if not self.connect_lidar():
                print("[LidarCtrl] Fallo al conectar. No se puede iniciar la recolección.")
                return False # No se pudo conectar

        # 3. Limpiar la señal de parada (por si acaso se reutiliza el evento)
        self.stop_event.clear()
        self.is_collecting = False # Asegurar que el flag esté listo

        # 4. Crear e iniciar el hilo de recolección
        print("[LidarCtrl] Creando e iniciando hilo de recolección de datos...")
        self.data_thread = threading.Thread(target=self._collect_data_loop,
                                              name="LidarCollectionInternal")
                                              # daemon=True es útil, pero gestionaremos con join
        self.data_thread.daemon = True
        self.data_thread.start()

        # Pequeña pausa para dar tiempo al hilo a arrancar y potencialmente fallar rápido
        time.sleep(0.5)
        if not self.data_thread.is_alive():
             print("[LidarCtrl] Error: El hilo de recolección no pudo iniciarse correctamente.")
             self.is_collecting = False
             return False

        print("[LidarCtrl] Hilo de recolección iniciado.")
        return True

    def stop(self):
        """
        Detiene el hilo de recolección, el motor del LIDAR y desconecta.
        Debe ser llamado desde el hilo que creó la instancia de LidarController.
        """
        print("[LidarCtrl] Solicitando detención...")

        # 1. Señalar al hilo interno que se detenga
        self.is_collecting = False # Actualizar flag
        self.stop_event.set()      # Activar evento (despierta a wait() y al bucle)

        # 2. Esperar a que el hilo de recolección termine
        if self.data_thread is not None and self.data_thread.is_alive():
            print("[LidarCtrl] Esperando finalización del hilo de recolección...")
            self.data_thread.join(timeout=3.0) # Esperar hasta 3 segundos
            if self.data_thread.is_alive():
                print("[LidarCtrl] ADVERTENCIA: Hilo de recolección no terminó limpiamente.")
            else:
                print("[LidarCtrl] Hilo de recolección finalizado.")
        self.data_thread = None # Limpiar referencia al hilo

        # 3. Detener motor y desconectar LIDAR
        if self.lidar is not None:
            print("[LidarCtrl] Deteniendo motor y desconectando LIDAR...")
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print("[LidarCtrl] LIDAR detenido y desconectado.")
            except Exception as e:
                print(f"[LidarCtrl] Error durante la detención/desconexión: {e}")
            finally:
                 self.lidar = None # Limpiar referencia al objeto LIDAR
        else:
             print("[LidarCtrl] LIDAR ya estaba desconectado.")


# --- Bloque de Prueba (Opcional) ---
# Eliminar o comentar este bloque cuando uses LidarController desde main_autonomo.py
if __name__ == '__main__':
    print("--- Probando LidarController directamente ---")
    test_stop_event = threading.Event()
    test_data_queue = queue.Queue()

    # Crear instancia
    lidar_controller = LidarController(port='COM3', data_queue=test_data_queue, stop_event=test_stop_event)

    # Iniciar recolección
    if lidar_controller.start_collection():
        print("Recolección iniciada. Presiona Ctrl+C para detener.")

        try:
            # Simular el hilo principal procesando datos
            while not test_stop_event.is_set():
                try:
                    data = test_data_queue.get(timeout=1.0)
                    print(f"Dato recibido: {data['tipo']}, {len(data['datos'])} puntos.")
                    # Aquí podrías hacer algún procesamiento básico
                    test_data_queue.task_done()
                except queue.Empty:
                    # print("Esperando datos...")
                    pass
                except KeyboardInterrupt: # Capturar Ctrl+C aquí también
                     print("\nCtrl+C detectado en prueba. Deteniendo...")
                     test_stop_event.set()
                     break

        finally:
            # Detener el LIDAR (esto ocurriría en el finally del main_autonomo.py)
            print("Deteniendo LidarController desde la prueba...")
            lidar_controller.stop()
            print("--- Prueba Finalizada ---")
    else:
        print("No se pudo iniciar la recolección.")
