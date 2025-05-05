# detecciones/detector_senalamientos.py
import cv2
from ultralytics import YOLO
from analisis_detecciones import analizar_color_semaforo_hsv

model = YOLO('yolo11m.pt')  # ajusta ruta
if model.device != 'cpu':
    model.to('cuda')

def run_deteccion_senalamientos(stop_event, data_queue, camera_index=0):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("[Señal] No se pudo abrir cámara")
        return

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret: continue

        results = model(frame)[0]
        detections = []

        for *box, conf, cls in results.boxes.data.tolist():
            cls_id = int(cls)
            if cls_id==9:  # semáforo
                x1,y1,x2,y2 = map(int, box)
                sub = frame[y1:y2, x1:x2]
                color = analizar_color_semaforo_hsv(sub)
                detections.append({'tipo':'semaforo', 'color': color})
            elif cls_id==11:  # stop
                detections.append({'tipo':'stop_sign'})

        # Empujar a la cola
        data_queue.put({'tipo':'senalamientos','datos':detections})

    cap.release()
