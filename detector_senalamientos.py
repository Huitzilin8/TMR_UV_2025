# detecciones/detector_senalamientos.py
import cv2
from ultralytics import YOLO
from analisis_detecciones import analizar_color_semaforo_hsv

model = YOLO('yolo11m.pt')  # ajusta ruta
if model.device != 'cpu':
    model.to('cuda')

def run_deteccion_senalamientos(stop_event, data_queue, camera_index=0, visualize=False):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("[Señal] No se pudo abrir cámara")
        return

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret: continue

        results = model(frame)[0]
        detections = []
        vis = frame.copy()
        
        for *box, conf, cls in results.boxes.data.tolist():
            cls_id = int(cls)
            x1,y1,x2,y2 = map(int, box)
            if cls_id == 9:  # semáforo
                sub = frame[y1:y2, x1:x2]
                color = analizar_color_semaforo_hsv(sub)
                detections.append({'tipo':'semaforo','color':color,'box':(x1,y1,x2,y2)})
                if visualize:
                    cv2.rectangle(vis,(x1,y1),(x2,y2),(0,255,0),2)
                    cv2.putText(vis,color,(x1,y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
            elif cls_id == 11:  # stop
                detections.append({'tipo':'stop_sign','box':(x1,y1,x2,y2)})
                if visualize:
                    cv2.rectangle(vis,(x1,y1),(x2,y2),(0,0,255),2)

        data_queue.put({'tipo':'senalamientos','datos':detections})

        if visualize:
            cv2.imshow("Visión Señalamientos", vis)
            cv2.waitKey(1)

    cap.release()
    if visualize: cv2.destroyWindow("Visión Señalamientos")
