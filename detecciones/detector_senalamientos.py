import cv2
from ultralytics import YOLO
import math
import torch
from analisis_detecciones import analizar_color_semaforo_hsv, ColorSemaforo # <--- Importar la función

# --- Configuración ---
model = YOLO('yolo11m.pt')
if torch.cuda.is_available():
    model.to('cuda')
    print("Modelo cargado en GPU.")

# Clases a detectar: Solo nos interesa 'traffic light' (ID 9) para el análisis
# Mantenemos 'stop sign' (ID 11) para visualización si quieres
target_classes_ids = [9, 11]
traffic_light_id = 9 # ID específico del semáforo
class_names = model.names
ESTADOS_SEMAFORO = {
    ColorSemaforo.ROJO: "Rojo",
    ColorSemaforo.AMARILLO: "Amarillo",
    ColorSemaforo.VERDE: "Verde",
    ColorSemaforo.INDETERMINADO: "Indeterminado",
}

confidence_threshold = 0.5

# Colores para los bounding boxes (BGR)
color_semaforo_default = (0, 255, 0) # Verde por defecto
color_stop_sign = (0, 0, 255)       # Rojo para señal de alto
color_luz = {
    "Rojo": (0, 0, 255),
    "Amarillo": (0, 255, 255),
    "Verde": (0, 255, 0),
    "Apagado": (128, 128, 128), # Gris
    "Indeterminado": (255, 0, 0) # Azul
}

def gstreamer_pipeline(sensor_id=0, capture_width=1280, capture_height=720, display_width=640, display_height=480, framerate=30):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, format=NV12, framerate={framerate}/1 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw, width={display_width}, height={display_height}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink"
    )


# --- Inicialización de la Cámara ---
cap = cv2.VideoCapture(gstreamer_pipeline(sensor_id=0), cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Error: No se pudo abrir la cámara web.")
    exit()

# --- Bucle Principal ---
while True:
    success, frame = cap.read()
    if not success:
        print("Error: No se pudo leer el fotograma.")
        break

    # Clonar el frame original para dibujar sobre él, manteniendo el original limpio si es necesario
    draw_frame = frame.copy()

    results = model(frame, stream=True, verbose=False)

    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls_id = int(box.cls[0])

            # Filtrar por clases objetivo
            if cls_id in target_classes_ids:
                confidence = math.ceil((box.conf[0] * 100)) / 100
                if confidence >= confidence_threshold:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # --- Análisis específico para semáforos ---
                    estado_luz = 0
                    box_color = color_semaforo_default # Color por defecto para semáforo

                    if cls_id == traffic_light_id:
                        # Extraer el ROI (Region of Interest) del semáforo
                        # Asegurarse que las coordenadas no se salgan de la imagen
                        y1_roi = max(0, y1)
                        y2_roi = min(frame.shape[0], y2)
                        x1_roi = max(0, x1)
                        x2_roi = min(frame.shape[1], x2)

                        if y1_roi < y2_roi and x1_roi < x2_roi: # Comprobar que el ROI tenga tamaño válido
                           roi_semaforo = frame[y1_roi:y2_roi, x1_roi:x2_roi]
                           # Llamar a la función de análisis del otro archivo
                           estado_luz = analizar_color_semaforo_hsv(roi_semaforo)
                           # Cambiar color del bounding box según el estado de la luz
                           box_color = color_luz.get(estado_luz, color_semaforo_default)
                        else:
                           estado_luz = ColorSemaforo.INDETERMINADO
                           box_color = color_luz["Indeterminado"]

                    elif cls_id == 11: # Stop sign
                         box_color = color_stop_sign


                    # --- Dibujar ---
                    cv2.rectangle(draw_frame, (x1, y1), (x2, y2), box_color, 2)

                    # Preparar etiqueta
                    label = f'{class_names[cls_id]}'
                    if estado_luz: # Añadir estado solo si es un semáforo analizado
                        label += f' ({ESTADOS_SEMAFORO[estado_luz]})'
                    label += f': {confidence:.2f}'

                    (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Texto un poco más pequeño
                    cv2.rectangle(draw_frame, (x1, y1 - text_height - baseline), (x1 + text_width, y1), box_color, -1)
                    cv2.putText(draw_frame, label, (x1, y1 - baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA) # Texto negro

    # Mostrar el fotograma procesado
    cv2.imshow('Webcam YOLO - Analisis Semaforo', draw_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Liberar Recursos ---
cap.release()
cv2.destroyAllWindows()
print("Programa finalizado.")
