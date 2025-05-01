import cv2
from ultralytics import YOLO
import math
import torch
from analisis_detecciones import analizar_color_semaforo_hsv, ColorSemaforo

model = YOLO('yolov8n.pt')
if torch.cuda.is_available():
    model.to('cuda')
    print("model load in GPU.")
else:
    print("Usando CPU.")

# 'trafic_light' y 'stop_sign' pueden cambiar por model
target_classes_ids = [9, 11]  # 9 = traffic light, 11 = stop sign
traffic_light_id = 9
class_names = model.names

ESTADOS_SEMAFORO = {
    ColorSemaforo.ROJO: "Rojo",
    ColorSemaforo.AMARILLO: "Amarillo",
    ColorSemaforo.VERDE: "Verde",
    ColorSemaforo.INDETERMINADO: "Indeterminado",
}

confidence_threshold = 0.5  # coeficiente de confianza min para deteccion

color_semaforo_default = (0, 255, 0)
color_stop_sign = (0, 0, 255)
color_luz = {
    "Rojo": (0, 0, 255),
    "Amarillo": (0, 255, 255),
    "Verde": (0, 255, 0),
    "Apagado": (128, 128, 128),
    "Indeterminado": (255, 0, 0),
}

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: No se pudo abrir la cámara web.")
    exit()

while True:
    success, frame = cap.read()
    if not success:
        print("Error: No se pudo leer el fotograma.")
        break

    draw_frame = frame.copy()

    # procesamiento con yolo
    results = model.predict(frame, stream=True, verbose=False,
                            device=0 if torch.cuda.is_available() else 'cpu')

    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls_id = int(box.cls[0])
            confidence = float(box.conf[0])

            if cls_id in target_classes_ids and confidence >= confidence_threshold:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                estado_luz = 0
                box_color = color_semaforo_default

                if cls_id == traffic_light_id:
                    y1_roi = max(0, y1)
                    y2_roi = min(frame.shape[0], y2)
                    x1_roi = max(0, x1)
                    x2_roi = min(frame.shape[1], x2)

                    if y1_roi < y2_roi and x1_roi < x2_roi:
                        roi = frame[y1_roi:y2_roi, x1_roi:x2_roi]
                        estado_luz = analizar_color_semaforo_hsv(roi)
                        box_color = color_luz.get(
                            estado_luz, color_semaforo_default)
                    else:
                        estado_luz = ColorSemaforo.INDETERMINADO
                        box_color = color_luz["Indeterminado"]
                elif cls_id == 11:
                    box_color = color_stop_sign

                cv2.rectangle(draw_frame, (x1, y1), (x2, y2), box_color, 2)
                label = f'{class_names[cls_id]}'
                if estado_luz:
                    label += f' ({ESTADOS_SEMAFORO[estado_luz]})'
                label += f': {confidence:.2f}'

                (text_width, text_height), baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(draw_frame, (x1, y1 - text_height -
                              baseline), (x1 + text_width, y1), box_color, -1)
                cv2.putText(draw_frame, label, (x1, y1 - baseline),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    cv2.imshow('camara de jetson live', draw_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Limpiar ---
cap.release()
cv2.destroyAllWindows()
print("Programa finalizado.")
