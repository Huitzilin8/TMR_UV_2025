# main_autonomo.py
import threading
import time
import queue
import sys
import cv2
import numpy as np # Solo se usa para visualización, borrar en producción

# --- Importar las funciones/clases modificadas ---
try:
    from detecciones.senalamientos_cuda import run_deteccion 
    from nav.Rada_2 import LidarController 
    from detecciones.deteccion_carriles import run_deteccion_carriles

except ImportError as e:
    print(f"Error importando módulos: {e}")
    sys.exit(1)

# --- Cola Compartida y Evento de Detención ---
datos_sensores_queue = queue.Queue(maxsize=100)
stop_event = threading.Event()

# --- Funciones Wrapper para los Hilos ---

# Wrapper para Señalamientos
def hilo_deteccion_wrapper(stop_event, data_queue):
    thread_name = threading.current_thread().name
    print(f"[{thread_name}] Iniciando...")
    try:
        run_deteccion(stop_event, data_queue)
    except Exception as e: print(f"[{thread_name}] Error crítico: {e}")
    finally: print(f"[{thread_name}] Terminado.")

def hilo_lidar_wrapper(stop_event, data_queue, lidar_port='COM3'):
    thread_name = threading.current_thread().name
    print(f"[{thread_name}] Iniciando...")
    lidar_ctrl = None
    try:
        lidar_ctrl = LidarController(port=lidar_port, data_queue=data_queue, stop_event=stop_event)
        if lidar_ctrl.start_collection():
            print(f"[{thread_name}] Recolección LIDAR iniciada.")
            stop_event.wait()
        else:
            print(f"[{thread_name}] Error: No se pudo iniciar LIDAR.")
    except Exception as e: print(f"[{thread_name}] Error crítico: {e}")
    finally:
        if lidar_ctrl:
             print(f"[{thread_name}] Deteniendo LIDAR...")
             lidar_ctrl.stop()
        print(f"[{thread_name}] Terminado.")

def hilo_carriles_wrapper(stop_event, data_queue, camera_index=0, target_size=(640, 480)):
    """Punto de entrada para el hilo de detección de carriles."""
    thread_name = threading.current_thread().name
    print(f"[{thread_name}] Iniciando...")
    try:
        # Llama a la función principal del módulo de carriles
        run_deteccion_carriles(stop_event, data_queue, camera_index, target_size)
    except Exception as e:
        print(f"[{thread_name}] Error crítico: {e}")
    finally:
        print(f"[{thread_name}] Terminado.")

# --- Programa Principal ---
if __name__ == "__main__":
    print("[Principal] Iniciando sistema...")
    print("[Principal] Presiona Ctrl+C para detener.")

    # --- Parámetros ---
    LIDAR_PORT = 'COM3' # Puerto LIDAR
    CAMERA_INDEX_SIGNALS = 0 # Índice cámara para señales/semáforos
    CAMERA_INDEX_LANES = 0   # Índice cámara para carriles
    TARGET_FRAME_SIZE = (640, 480) # Tamaño al que se procesarán los frames de carril

    # ---> ¡IMPORTANTE! Chequeo de Índice de Cámara <---
    if CAMERA_INDEX_SIGNALS == CAMERA_INDEX_LANES:
        print("[Principal] ADVERTENCIA: ¡La misma cámara está asignada a Señalamientos y Carriles!")
        print("Esto causará conflictos. Debes usar cámaras diferentes o refactorizar para")
        print("tener un solo hilo de captura que distribuya los frames.")
        # Podrías decidir salir o continuar bajo tu propio riesgo
        # sys.exit(1)

    # --- Crear los Hilos ---
    hilo_senalamientos = threading.Thread(
        target=hilo_deteccion_wrapper,
        args=(stop_event, datos_sensores_queue), 
        name="Sensor-Senalamientos",
        daemon=True)

    hilo_lidar = threading.Thread(
        target=hilo_lidar_wrapper,
        args=(stop_event, datos_sensores_queue, LIDAR_PORT),
        name="Sensor-LIDAR",
        daemon=True)

    hilo_carriles = threading.Thread(
        target=hilo_carriles_wrapper,
        args=(stop_event, datos_sensores_queue, CAMERA_INDEX_LANES, TARGET_FRAME_SIZE),
        name="Sensor-Carriles",
        daemon=True)

    # --- Iniciar los Hilos ---
    print("[Principal] Iniciando hilos...")
    hilo_senalamientos.start()
    time.sleep(0.5) # Pausa opcional
    hilo_lidar.start()
    time.sleep(0.5) # Pausa opcional
    hilo_carriles.start()
    print("[Principal] Todos los hilos iniciados.")

    # --- Variables para el estado/control del vehículo (Ejemplos) ---
    estado_semaforo_actual = None
    obstaculo_lidar_cercano = False
    correccion_direccion_actual = 0.0

    # --- Opcional: Preparar Ventana para Visualización ---
    WINDOW_NAME = "Salida Combinada"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    # Placeholder para el frame que se mostrará
    display_frame = np.zeros((TARGET_FRAME_SIZE[1], TARGET_FRAME_SIZE[0], 3), dtype=np.uint8)

    # --- Bucle Principal de Procesamiento ---
    try:
        while not stop_event.is_set():
            # Reiniciar el frame de visualización en cada iteración
            display_frame.fill(0)

            try:
                # Procesar todos los datos disponibles en la cola sin bloquear mucho tiempo
                while True: # Procesar todo lo que haya en la cola
                    dato_sensor = datos_sensores_queue.get(block=False) # No bloquear si está vacía
                    tipo_dato = dato_sensor.get('tipo')
                    datos = dato_sensor.get('datos', {})

                    # --- Lógica de Actualización de Estado basada en Sensores ---
                    if tipo_dato == 'senalamientos':
                        print(f"[Principal] Recibido Señalamientos: {[dato for dato in datos]} dets")
                         # ... tu lógica para interpretar 'datos' de señalamientos ...

                    elif tipo_dato == 'lidar':
                         # Actualiza si hay un obstáculo cercano basado en 'datos' LIDAR
                         # print(f"[Principal] Recibido LIDAR: {len(datos)} puntos")
                         obstaculo_lidar_cercano = False # Resetear en cada nuevo scan
                         dist_min = 500 # mm
                         ang_rng = 15 # grados
                         for q, angle, dist in datos:
                             if (0 <= angle <= ang_rng) or (360 - ang_rng <= angle < 360):
                                 if 0 < dist < dist_min:
                                     obstaculo_lidar_cercano = True
                                     break
                         # if obstaculo_lidar_cercano: print("[Principal] Obstáculo LIDAR detectado!")

                    elif tipo_dato == 'carriles':
                         # Actualiza la corrección de dirección y opcionalmente dibuja
                         if 'error' in datos:
                             print(f"[Principal] Error Carriles: {datos['error']}")
                             correccion_direccion_actual = 0.0 # O mantener la última?
                         else:
                             correccion_px = datos.get('correccion_px')
                             centros = datos.get('centros', [])
                             frame_size = datos.get('target_size', TARGET_FRAME_SIZE)

                             if correccion_px is not None:
                                 # Convertir corrección en píxeles a un valor de control (ej. ángulo de giro)
                                 # Esto es muy dependiente de tu sistema de control
                                 # Ejemplo muy simple:
                                 max_correccion_px = frame_size[0] / 4 # Max desvío esperado
                                 correccion_direccion_actual = -float(correccion_px) / max_correccion_px # Normalizar y invertir?
                                 correccion_direccion_actual = np.clip(correccion_direccion_actual, -1.0, 1.0) # Limitar a [-1, 1]
                                 # print(f"[Principal] Corrección Carril (px): {correccion_px:.1f}, Control: {correccion_direccion_actual:.2f}")
                             else:
                                 # Perdimos el carril, ¿qué hacer? Mantener dirección? Frenar?
                                 correccion_direccion_actual = 0.0 # Ejemplo: ir recto
                                 # print("[Principal] Carril no detectado claramente.")


                             # ---> Dibujar Carriles en Ventana (Opcional) <---
                             if centros:
                                 # Dibujar líneas entre centros
                                 for i in range(1, len(centros)):
                                     cv2.line(display_frame, tuple(map(int,centros[i-1])), tuple(map(int,centros[i])), (0, 255, 0), 2)
                                 # Dibujar círculos en los centros
                                 for center in centros:
                                     cv2.circle(display_frame, tuple(map(int,center)), 5, (0, 0, 255), -1)
                             # Dibujar línea central y corrección
                             cam_center_x = frame_size[0] // 2
                             cv2.line(display_frame, (cam_center_x, frame_size[1]), (cam_center_x, frame_size[1]-50), (255, 255, 0), 1)
                             if correccion_px is not None:
                                 target_x_viz = cam_center_x + int(correccion_px)
                                 cv2.line(display_frame, (target_x_viz, frame_size[1]-20), (target_x_viz, frame_size[1]-70), (0, 255, 255), 2)



                    datos_sensores_queue.task_done() # Marcar como procesado

            except queue.Empty:
                 # No hay más datos en la cola por ahora
                 pass


            # --- Lógica de Control Principal (Toma de Decisiones) ---
            # Basado en estado_semaforo_actual, obstaculo_lidar_cercano, correccion_direccion_actual
            # decidir la acción del vehículo (acelerar, frenar, girar).

            # Ejemplo muy básico:
            velocidad_deseada = 0.5 # Velocidad base (ej. 0 a 1)
            angulo_giro = correccion_direccion_actual * 0.8 # Factor para convertir corrección a ángulo (ej. -1 a 1)

            if obstaculo_lidar_cercano: # Prioridad alta
                print("[Principal] ¡ACCIÓN! FRENAR por obstáculo LIDAR.")
                velocidad_deseada = 0.0
                angulo_giro = 0.0 # O intentar esquivar si hay lógica para ello
            # elif estado_semaforo_actual == 'Rojo' or estado_semaforo_actual == 'Stop': # Ejemplo
            #     print("[Principal] ¡ACCIÓN! FRENAR por señal/semáforo.")
            #     velocidad_deseada = 0.0
            #     angulo_giro = 0.0

            # ---> AQUÍ ENVIARÍAS comandos a los actuadores del vehículo <---
            # enviar_comando_motor(velocidad_deseada)
            # enviar_comando_direccion(angulo_giro)

            # --- Visualización (Opcional) ---
            # Añadir texto de estado al frame de visualización
            cv2.putText(display_frame, f"LIDAR Obstacle: {'SI' if obstaculo_lidar_cercano else 'NO'}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            cv2.putText(display_frame, f"Lane Correction: {correccion_direccion_actual:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            # Mostrar el frame combinado
            cv2.imshow(WINDOW_NAME, display_frame)


            # --- Espera y Comprobación de Hilos ---
            if cv2.waitKey(5) & 0xFF == ord('q'): # Permitir salir con 'q'
                print("[Principal] 'q' presionada. Deteniendo...")
                stop_event.set()
                break

            # Verificar si los hilos hijos siguen vivos
            if not hilo_senalamientos.is_alive() or not hilo_lidar.is_alive() or not hilo_carriles.is_alive():
                 if not stop_event.is_set():
                     print("[Principal] ¡ERROR! Hilo de sensor terminado inesperadamente.")
                     stop_event.set() # Detener todo
                     break

            # Pequeña pausa para no quemar CPU en el hilo principal
            time.sleep(0.01) # Ajustar si es necesario


    except KeyboardInterrupt:
        print("\n[Principal] Ctrl+C detectado. Deteniendo hilos...")
        stop_event.set()

    finally:
        # --- Esperar a que los Hilos Terminen ---
        print("[Principal] Esperando finalización de hilos...")
        hilo_senalamientos.join(timeout=5.0)
        hilo_lidar.join(timeout=5.0)
        hilo_carriles.join(timeout=5.0) # Esperar al hilo de carriles

        # Verificar si terminaron
        if hilo_senalamientos.is_alive(): print("[P] ADVERTENCIA: Hilo Señalamientos no terminó.")
        if hilo_lidar.is_alive(): print("[P] ADVERTENCIA: Hilo LIDAR no terminó.")
        if hilo_carriles.is_alive(): print("[P] ADVERTENCIA: Hilo Carriles no terminó.")

        print("[Principal] Sistema detenido.")
        cv2.destroyAllWindows() # Cerrar ventana de OpenCV