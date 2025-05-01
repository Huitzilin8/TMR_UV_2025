import cv2
import numpy as np

# --------------------------
# Sliding windows sin ajuste polinomial
# --------------------------
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

    centers = []  # Para almacenar el centro del carril en cada ventana

    for window in range(nwindows):
        win_y_low = height - (window + 1) * window_height
        win_y_high = height - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

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
            centers.append((center, (win_y_low + win_y_high) // 2))

    return centers

# --------------------------
# Captura de video
# --------------------------
cap = cv2.VideoCapture(0)  # Puede cambiarse por un video o cámara IP

if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(binary, kernel, iterations=1)

    try:
        centers = get_lane_centers(eroded.copy(), nwindows=10)

        # Dibujar el camino conectando los centros
        for i in range(1, len(centers)):
            cv2.line(frame, centers[i-1], centers[i], (0, 255, 0), 2)
            cv2.circle(frame, centers[i], 3, (0, 255, 255), -1)

        # Elegir un punto intermedio como referencia para dirección
        if len(centers) >= 4:
            target = centers[3]  # Por ejemplo, la cuarta ventana desde abajo
            camera_center = frame.shape[1] // 2
            correction = target[0] - camera_center

            cv2.line(frame, (camera_center, frame.shape[0]), (camera_center, frame.shape[0]-50), (255,255,255), 2)
            cv2.line(frame, (target[0], target[1]), (target[0], target[1]-30), (0,255,255), 2)
            cv2.putText(frame, f"Correccion: {correction:.2f}px", (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        else:
            cv2.putText(frame, "Carril no suficientemente detectado", (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

    except Exception as e:
        cv2.putText(frame, "Error en deteccion", (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

    cv2.imshow("Deteccion de Carriles Simplificada", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
