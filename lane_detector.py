# -*- coding: utf-8 -*-

import cv2
import threading
import numpy as np
import time

left_camera = None
right_camera = None


class Camera:
    def __init__(self):
        self.video_capture = None
        self.frame = None
        self.grabbed = False
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def open(self, sensor_id=0):
        sensor_mode = 3
        capture_width = 1280
        capture_height = 720
        display_width = 640
        display_height = 360
        framerate = 20
        flip_method = 2
        gstreamer_pipeline_string = (
            "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                sensor_mode,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )

        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )

        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)
            return
        self.grabbed, self.frame = self.video_capture.read()

    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop(self):
        self.running = False
        self.read_thread.join()

    def updateCamera(self):
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        if self.read_thread != None:
            self.read_thread.join()


def preprocess_frame(frame):
    frame = cv2.resize(frame, (640, 480))
    roi = frame[240:, :]  # Solo analizar la mitad inferior

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
    # Umbral medio-alto: detecta blancos incluso con algo de sombra
    _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

    # Erosión ligera para reducir ruido fino
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(binary, kernel, iterations=1)

    result = np.zeros((480, 640), dtype=np.uint8)
    result[240:, :] = eroded
    return result


def get_lane_centers(binary_img, nwindows=10, margin=50, minpix=50):
    height, width = binary_img.shape
    histogram = np.sum(binary_img[height//2:, :], axis=0)

    midpoint = width // 2
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    window_height = height // nwindows
    nonzero = binary_img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    leftx_current = leftx_base
    rightx_current = rightx_base
    centers = []

    for window in range(nwindows):
        win_y_low = height - (window + 1) * window_height
        win_y_high = height - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        cv2.rectangle(binary_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), 150, 2)
        cv2.rectangle(binary_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), 150, 2)

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))

        if len(good_left_inds) > 0 and len(good_right_inds) > 0:
            center = int((leftx_current + rightx_current) / 2)
        elif len(good_left_inds) > 0:
            center = leftx_current + margin
        elif len(good_right_inds) > 0:
            center = rightx_current - margin
        else:
            continue

        centers.append((center, (win_y_low + win_y_high) // 2))

    return centers


def start_cameras():
    left_camera = Camera()
    left_camera.open(0)
    left_camera.start()

    right_camera = Camera()
    right_camera.open(1)
    right_camera.start()

    cv2.namedWindow("CSI Cameras", cv2.WINDOW_AUTOSIZE)

    if not left_camera.video_capture.isOpened() or not right_camera.video_capture.isOpened():
        print("Unable to open any cameras")
        SystemExit(0)

    while cv2.getWindowProperty("CSI Cameras", 0) >= 0:
        _, left_image = left_camera.read()
        _, right_image = right_camera.read()

        processed_left = preprocess_frame(left_image)
        processed_right = preprocess_frame(right_image)

        try:
            centers_left = get_lane_centers(processed_left)
            centers_right = get_lane_centers(processed_right)

            for i in range(1, len(centers_left)):
                cv2.line(left_image, centers_left[i-1], centers_left[i], (0, 255, 0), 2)
                cv2.circle(left_image, centers_left[i], 3, (0, 255, 255), -1)

            for i in range(1, len(centers_right)):
                cv2.line(right_image, centers_right[i-1], centers_right[i], (0, 255, 0), 2)
                cv2.circle(right_image, centers_right[i], 3, (0, 255, 255), -1)

            camera_images = np.hstack((left_image, right_image))
            cv2.imshow("CSI Cameras", camera_images)

        except Exception as e:
            cv2.putText(left_image, "Error en detección", (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            cv2.putText(right_image, "Error en detección", (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

        keycode = cv2.waitKey(30) & 0xFF
        if keycode == 27:
            break

    left_camera.stop()
    left_camera.release()
    right_camera.stop()
    right_camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    start_cameras()
