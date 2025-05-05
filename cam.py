import cv2

def gstreamer_pipeline(sensor_id=0, capture_width=1280, capture_height=720, display_width=640, display_height=480, framerate=30):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, format=NV12, framerate={framerate}/1 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw, width={display_width}, height={display_height}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink"
    )

def main():
    cap0 = cv2.VideoCapture(gstreamer_pipeline(sensor_id=0), cv2.CAP_GSTREAMER)
    cap1 = cv2.VideoCapture(gstreamer_pipeline(sensor_id=1), cv2.CAP_GSTREAMER)

    if not cap0.isOpened():
        print("No se pudo abrir la cámara 0")
    if not cap1.isOpened():
        print("No se pudo abrir la cámara 1")

    while cap0.isOpened() or cap1.isOpened():
        if cap0.isOpened():
            ret0, frame0 = cap0.read()
            if ret0:
                cv2.imshow('Cámara 0', frame0)
        if cap1.isOpened():
            ret1, frame1 = cap1.read()
            if ret1:
                cv2.imshow('Cámara 1', frame1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

