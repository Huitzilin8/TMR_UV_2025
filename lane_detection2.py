# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time
import queue
import threading

# Clase para lectura de cámara CSI en hilo
class Camera:
    def __init__(self, video_path=None):
        self.video_capture = video_path
        self.frame = None
        self.grabbed = False
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def open(self, sensor_id=0):
        sensor_mode = 3
        capture_width = 720
        capture_height = 480
        display_width = 640
        display_height = 480
        framerate = 20
        flip_method = 0
        pipeline = (
            "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id, sensor_mode,
                capture_width, capture_height,
                framerate, flip_method,
                display_width, display_height
            )
        )
        try:
            if self.video_capture is None:
                self.video_capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                self.grabbed, self.frame = self.video_capture.read()
        except RuntimeError:
            print("Unable to open camera with pipeline:\n" + pipeline)
            self.video_capture = None

    def start(self):
        if self.running or self.video_capture is None:
            return
        self.running = True
        self.read_thread = threading.Thread(target=self._update)
        self.read_thread.daemon = True
        self.read_thread.start()

    def _update(self):
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Error reading from camera")

    def read(self):
        with self.read_lock:
            if self.frame is None:
                return False, None
            return self.grabbed, self.frame.copy()

    def stop(self):
        self.running = False
        if self.read_thread is not None:
            self.read_thread.join()

    def release(self):
        if self.video_capture is not None:
            self.video_capture.release()
            self.video_capture = None


# Preprocesamiento de frame
def preprocess_frame(frame):
    frame = cv2.resize(frame, (640, 480))
    roi = frame[240:, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(binary, kernel, iterations=1)
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[240:, :] = eroded
    return mask


# Detección de centros de carril con filtro de carriles exteriores
def get_lane_centers(binary_img, nwindows=10, margin=50, minpix=50):
    height, width = binary_img.shape
    histogram = np.sum(binary_img[height // 2:, :], axis=0)
    midpoint = width // 2
    leftx = np.argmax(histogram[:midpoint])
    rightx = np.argmax(histogram[midpoint:]) + midpoint
    window_h = height // nwindows
    nonzero = binary_img.nonzero()
    ys, xs = nonzero[0], nonzero[1]
    current_left, current_right = leftx, rightx
    centers = []

    for w in range(nwindows):
        y_low = height - (w + 1) * window_h
        y_high = height - w * window_h
        lx_low, lx_high = current_left - margin, current_left + margin
        rx_low, rx_high = current_right - margin, current_right + margin

        # Dibujar ventanas para depuración
        cv2.rectangle(binary_img, (lx_low, y_low), (lx_high, y_high), 150, 2)
        cv2.rectangle(binary_img, (rx_low, y_low), (rx_high, y_high), 150, 2)

        left_inds = ((ys >= y_low) & (ys < y_high) & (xs >= lx_low) & (xs < lx_high)).nonzero()[0]
        right_inds = ((ys >= y_low) & (ys < y_high) & (xs >= rx_low) & (xs < rx_high)).nonzero()[0]

        if len(left_inds) > minpix:
            current_left = int(xs[left_inds].mean())
        if len(right_inds) > minpix:
            current_right = int(xs[right_inds].mean())

        if left_inds.size and right_inds.size:
            cx = (current_left + current_right) // 2
        elif left_inds.size:
            cx = current_left + margin
        elif right_inds.size:
            cx = current_right - margin
        else:
            continue

        cy = (y_low + y_high) // 2
        centers.append((cx, cy))

    # Filtrar las líneas más cercanas al centro
    centers.sort(key=lambda x: abs(x[0] - width // 2))  # Ordenar por cercanía al centro
    filtered_centers = centers[:2]  # Tomar las dos más cercanas al centro
    return filtered_centers

def send_to_esp(correction,queue):
    # Simular el envio de datos al esp32
    # Mandar la correcion de direccion al esp32 a tra ves de la cola
    queue.put(correction,adelante)
    print("Correcion enviada a la cola")

# Función principal que inicia la cámara y procesa los frames
def start_cameras(queue):
    left_cam = Camera()
    left_cam.open(0)
    left_cam.start()

    right_cam = Camera()
    right_cam.open(1)
    right_cam.start()

    if not left_cam.video_capture or not left_cam.video_capture.isOpened() or \
       not right_cam.video_capture or not right_cam.video_capture.isOpened():
        print("Unable to open both cameras")
        return

    cv2.namedWindow("CSI Cameras", cv2.WINDOW_AUTOSIZE)

    last_left_center = None
    last_right_center = None

    try:
        while True:
            ret_l, left_frame = left_cam.read()
            ret_r, right_frame = right_cam.read()
            if not ret_l or not ret_r:
                break

            proc_l = preprocess_frame(left_frame)
            proc_r = preprocess_frame(right_frame)

            centers_l = get_lane_centers(proc_l)
            centers_r = get_lane_centers(proc_r)

            # Dibujar resultados
            for i in range(1, len(centers_l)):
                cv2.line(left_frame, centers_l[i-1], centers_l[i], (0,255,0), 2)
                cv2.circle(left_frame, centers_l[i], 3, (0,255,255), -1)
            for i in range(1, len(centers_r)):
                cv2.line(right_frame, centers_r[i-1], centers_r[i], (0,255,0), 2)
                cv2.circle(right_frame, centers_r[i], 3, (0,255,255), -1)

            combined = np.hstack((left_frame, right_frame))
            cv2.imshow("CSI Cameras", combined)

            # Enviar la corrección a través del puerto serial
            if len(centers_l) > 0 or len(centers_r) > 0:
                target_center = (centers_l[0][0] + centers_r[0][0]) // 2 if len(centers_l) and len(centers_r) else 0
                camera_center = combined.shape[1] // 2
                correction = target_center - camera_center

                # Si no se detectaron ambos carriles, hacer ajustes
                if last_left_center is not None and last_right_center is not None:
                    if len(centers_l) == 1:
                        # Ajustar hacia el carril detectado (izquierda o derecha)
                        if target_center < camera_center:
                            correction = -20  # Ajuste para la izquierda
                        else:
                            correction = 20  # Ajuste para la derecha
                    elif len(centers_l) == 0:
                        # No se detectaron carriles, mantener el vehículo recto
                        correction = 0

                last_left_center = centers_l[0] if len(centers_l) > 0 else last_left_center
                last_right_center = centers_r[-1] if len(centers_r) > 0 else last_right_center
		
		if correction > 10:
		    send_to_esp('d')
		elif correction < -10:
		    send_to_esp('i')
		else:
		    send_to_esp('a')
		
            if cv2.waitKey(30) & 0xFF == 27:
                break
    finally:
        left_cam.stop()
        left_cam.release()
        right_cam.stop()
        right_cam.release()
        cv2.destroyAllWindows()


# Llamada a la función principal
if __name__ == "__main__":
    data_queue = queue.Queue()
    start_cameras(queue)  # Puede ingresarse una ruta como parametro para habilitar procesamiento de videos
