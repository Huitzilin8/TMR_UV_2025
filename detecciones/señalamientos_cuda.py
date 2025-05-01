import cv2
from ultralytics import YOLO
import math
import torch
from analisis_detecciones import analizar_color_semaforo_hsv, ColorSemaforo


model = YOLO('yolo11m.pt')
if torch.cuda.is_available():
    model.to('cuda')
    print("Modelo cargado en GPU.")

# combrobarcion cv2 cuda
use_cuda_cv = cv2.cuda.getCudaEnabledDeviceCount() > 0
if use_cuda_cv:
    print("OpenCV con soporte CUDA disponible.")
else:
    print("OpenCV sin soporte CUDA. Se usará CPU para operaciones cv2.")

target_classes_ids = [9, 11]
traffic_light_id = 9
class_names = model.names

ESTADOS_SEMAFORO = {
    ColorSemaforo.ROJO: "Rojo",
    ColorSemaforo.AMARILLO: "Amarillo",
    ColorSemaforo.VERDE: "Verde",
    ColorSemaforo.INDETERMINADO: "Indeterminado",
}

confidence_threshold = 0.5
color_semaforo_default = (0, 255, 0)
color_stop_sign = (0, 0, 255)
color_luz = {
    "Rojo": (0, 0, 255),
    "Amarillo": (0, 255, 255),
    "Verde": (0, 255, 0),
    "Apagado": (128, 128, 128),
    "Indeterminado": (255, 0, 0)
}

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: No se pudo abrir la cámara.")
    exit()

while True:
    success, frame = cap.read()
    if not success:
        print("Error: No se pudo leer el fotograma.")
        break

    draw_frame = frame.copy()

    # aceleracion con gpu
    if use_cuda_cv:
        gpu_frame = cv2.cuda_GpuMat()
        gpu_frame.upload(frame)

    results = model(frame, stream=True, verbose=False)

    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls_id = int(box.cls[0])

            if cls_id in target_classes_ids:
                confidence = math.ceil((box.conf[0] * 100)) / 100
                if confidence >= confidence_threshold:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    estado_luz = 0
                    box_color = color_semaforo_default

                    if cls_id == traffic_light_id:
                        y1_roi = max(0, y1)
                        y2_roi = min(frame.shape[0], y2)
                        x1_roi = max(0, x1)
                        x2_roi = min(frame.shape[1], x2)

                        if y1_roi < y2_roi and x1_roi < x2_roi:
                            # extract roi from gpu frame
                            # y1_roi = max(0, y1)
                            if use_cuda_cv:
                                roi_gpu = gpu_frame.rowRange(
                                    y1_roi, y2_roi).colRange(x1_roi, x2_roi)
                                roi_cpu = roi_gpu.download()
                            else:
                                roi_cpu = frame[y1_roi:y2_roi, x1_roi:x2_roi]

                            estado_luz = analizar_color_semaforo_hsv(roi_cpu)
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
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    cv2.imshow('camara jetson', draw_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("Programa finalizado.")
