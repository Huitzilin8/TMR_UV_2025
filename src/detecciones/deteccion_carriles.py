# deteccion_carriles.py
import cv2
import numpy as np
import time
import queue  # Importar queue
import threading # Importar threading para obtener nombre del hilo
import zmq
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.SUB) # Or zmq.PULL
socket.connect("tcp://localhost:5555")
socket.subscribe("") # Subscribe to all messages

# --- Funciones de Procesamiento (Modificadas para claridad y eficiencia) ---
def preprocess_frame(frame, target_size=(640, 480), roi_ratio=0.5, threshold_value=180):
    """
    Preprocesa un frame: redimensiona, selecciona ROI, convierte a gris, umbraliza y erosiona.
    Args:
        frame: El frame de entrada BGR.
        target_size: Tupla (ancho, alto) a la que se redimensionará.
        roi_ratio: Proporción de la altura desde abajo para la ROI (0.5 = mitad inferior).
        threshold_value: Valor de umbral para la binarización.
    Returns:
        Imagen binaria (parte de la ROI procesada).
    """
    resized_frame = cv2.resize(frame, target_size)
    height, width = resized_frame.shape[:2]

    # Calcular inicio de la ROI (Región de Interés)
    roi_start_row = int(height * (1 - roi_ratio))
    roi = resized_frame[roi_start_row:, :]

    # Convertir a escala de grises
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Aplicar umbral binario
    _, binary = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)

    # Erosión ligera para reducir ruido (opcional, ajustar kernel/iteraciones)
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(binary, kernel, iterations=1)

    return eroded # Devuelve solo la ROI binaria procesada

def get_lane_centers(binary_roi, roi_start_row_in_resized, nwindows=10, margin=50, minpix=50):
    """
    Encuentra los puntos centrales del carril usando ventanas deslizantes en la ROI binaria.
    Args:
        binary_roi: La imagen binaria de la región de interés inferior.
        roi_start_row_in_resized: La fila donde comenzó la ROI en el frame redimensionado.
        nwindows: Número de ventanas deslizantes verticales.
        margin: Mitad del ancho de la ventana.
        minpix: Número mínimo de píxeles para recentrar la ventana.
    Returns:
        Lista de tuplas (x, y_original) representando los centros del carril en
        coordenadas del frame redimensionado.
    """
    roi_height, width = binary_roi.shape
    histogram = np.sum(binary_roi[roi_height//2:, :], axis=0) # Histograma en la mitad inferior de la ROI

    midpoint = width // 2
    # Puntos base para iniciar las búsquedas izquierda y derecha
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Si los picos están muy cerca o en el borde, podría indicar un mal inicio.
    # Se podría añadir lógica para manejar estos casos si fuera necesario.

    window_height = roi_height // nwindows
    # Obtener coordenadas (y, x) de los píxeles blancos en la ROI
    nonzero = binary_roi.nonzero()
    nonzeroy = np.array(nonzero[0]) # Coordenadas Y dentro de la ROI
    nonzerox = np.array(nonzero[1]) # Coordenadas X dentro de la ROI

    leftx_current = leftx_base
    rightx_current = rightx_base
    centers_in_resized = [] # Almacenará centros en coordenadas del frame redimensionado

    # Iterar sobre las ventanas desde abajo hacia arriba
    for window in range(nwindows):
        # Calcular límites verticales de la ventana (en coordenadas ROI)
        win_y_low = roi_height - (window + 1) * window_height
        win_y_high = roi_height - window * window_height
        # Calcular límites horizontales basados en la posición actual
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # Identificar píxeles blancos dentro de la ventana (en coordenadas ROI)
        # .nonzero()[0] devuelve los índices de los elementos True
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        center_x_in_window = -1 # Valor inválido inicial

        # Si se encontraron suficientes píxeles, recalcular la posición X de la ventana para la siguiente iteración
        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))

        # Calcular el centro del carril en esta ventana
        if len(good_left_inds) >= minpix and len(good_right_inds) >= minpix:
            # Tenemos ambas líneas
            center_x_in_window = int((leftx_current + rightx_current) / 2)
        elif len(good_left_inds) >= minpix:
            # Solo vemos la línea izquierda, estimamos el centro
            center_x_in_window = leftx_current + margin # Asumiendo que 'margin' es la mitad del ancho del carril
        elif len(good_right_inds) >= minpix:
            # Solo vemos la línea derecha, estimamos el centro
            center_x_in_window = rightx_current - margin # Asumiendo que 'margin' es la mitad del ancho del carril

        # Si se calculó un centro válido para esta ventana
        if center_x_in_window != -1:
            # Calcular la coordenada Y promedio de la ventana en el frame redimensionado original
            center_y_in_resized = roi_start_row_in_resized + (win_y_low + win_y_high) // 2
            centers_in_resized.append((center_x_in_window, center_y_in_resized))

    # Devolver lista de centros [(x1, y1_resized), (x2, y2_resized), ...]
    return centers_in_resized


# --- Función Principal para el Hilo de Detección de Carriles ---
def run_deteccion_carriles(stop_event, data_queue, camera_index=0, target_size=(640, 480)):
    """
    Función principal que se ejecuta en el hilo de detección de carriles.
    Captura frames, los procesa y envía los resultados a la cola compartida.
    """
    thread_name = threading.current_thread().name
    print(f"[{thread_name}] Iniciando. Cámara índice: {camera_index}, Tamaño destino: {target_size}")

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[{thread_name}] Error crítico: No se pudo abrir la cámara {camera_index}.")
        return # Terminar el hilo si no hay cámara

    # Parámetros de procesamiento
    resized_width, resized_height = target_size
    roi_start_row = resized_height // 2 # La ROI empieza en la mitad vertical del frame redimensionado

    print(f"[{thread_name}] Cámara abierta. Iniciando bucle...")

    while not stop_event.is_set():
        # Receive frame data
        multipart_message = socket.recv_multipart()
        meta_str = multipart_message[0].decode('utf-8')
        frame_bytes = multipart_message[1]

        # Reconstruct frame
        h, w, c = map(int, meta_str.split(','))
        frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((h, w, c))

        try:
            # 1. Preprocesar el frame
            processed_roi = preprocess_frame(frame, target_size=target_size, roi_ratio=0.5)

            # 2. Obtener centros del carril
            # Pasar la fila de inicio de la ROI en coordenadas del frame redimensionado
            centers = get_lane_centers(processed_roi,
                                       roi_start_row_in_resized=roi_start_row,
                                       nwindows=10, margin=70, minpix=40) # Ajustar nwindows, margin, minpix según sea necesario

            # 3. Calcular la corrección de dirección
            correction_px = None # Valor por defecto
            target_y_threshold = resized_height * 0.8 # Punto objetivo (ej. 80% hacia abajo en la imagen)
            closest_center_to_target = None
            min_y_diff = float('inf')

            if centers:
                 # Encontrar el centro detectado más cercano (por debajo) a nuestro umbral Y objetivo
                 for cx, cy in reversed(centers): # Buscar desde abajo hacia arriba
                      if cy > roi_start_row and cy < target_y_threshold : # Buscar puntos en la parte inferior
                           closest_center_to_target = (cx, cy)
                           break # Tomar el primero encontrado desde abajo

                 if closest_center_to_target:
                      target_x = closest_center_to_target[0]
                      camera_center_x = resized_width // 2
                      # Corrección necesaria: > 0 -> ir a la derecha, < 0 -> ir a la izquierda
                      correction_px = target_x - camera_center_x

            # 4. Preparar datos para la cola
            output_data = {
                'tipo': 'carriles',
                'datos': {
                    'centros': centers,         # Lista de tuplas (x, y) en coords redimensionadas
                    'correccion_px': correction_px, # Valor numérico (o None)
                    'target_size': target_size    # Incluir tamaño para referencia en el hilo principal
                }
            }

            # 5. Enviar datos a la cola
            try:
                data_queue.put(output_data, block=False) # No esperar si la cola está llena
            except queue.Full:
                print(f"[{thread_name}] Advertencia: Cola de datos llena, descartando frame de carriles.")
                pass

        except Exception as e:
            print(f"[{thread_name}] Error en procesamiento de frame: {e}")
            # Opcional: Enviar un mensaje de error a la cola
            error_data = {'tipo': 'carriles', 'datos': {'error': str(e)}}
            try: data_queue.put(error_data, block=False)
            except queue.Full: pass

        # Pequeña pausa para evitar consumo excesivo de CPU y permitir que otros hilos se ejecuten
        time.sleep(0.02) # Ajustar según sea necesario (ej. 1/fps_deseado)

    # --- Limpieza al Finalizar ---
    print(f"[{thread_name}] Señal de parada recibida. Liberando cámara...")
    cap.release()
    print(f"[{thread_name}] Cámara liberada. Hilo terminado.")