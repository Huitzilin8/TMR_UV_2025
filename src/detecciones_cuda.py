import cv2
import numpy as np
import enum

MAIN = False


class ColorSemaforo(enum.Enum):
    ROJO = 1
    AMARILLO = 2
    VERDE = 3
    INDETERMINADO = 0


def analizar_color_semaforo_hsv(roi_semaforo) -> ColorSemaforo:
    if roi_semaforo is None or roi_semaforo.size == 0:
        return ColorSemaforo.INDETERMINADO
    if MAIN:
        print(f"semafro: {roi_semaforo.size} pixeles")
        cv2.imshow(f"semaforo", roi_semaforo)

    hsv = cv2.cuda_GpuMat()  # aceleracion con gpu
    hsv.upload(roi_semaforo)
    hsv = cv2.cuda.cvtColor(hsv, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 150, 120])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 150, 120])
    upper_red2 = np.array([180, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    lower_green = np.array([40, 100, 100])
    upper_green = np.array([85, 255, 255])
    # generar masks en gpu
    mask_red1 = cv2.cuda.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.cuda.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.cuda.bitwise_or(mask_red1, mask_red2)
    mask_yellow = cv2.cuda.inRange(hsv, lower_yellow, upper_yellow)
    mask_green = cv2.cuda.inRange(hsv, lower_green, upper_green)
    # cargar masks a cpu
    mask_red = mask_red.download()
    mask_yellow = mask_yellow.download()
    mask_green = mask_green.download()

    pixels_red = cv2.countNonZero(mask_red)
    if MAIN:
        print(f"Rojo: {pixels_red} píxeles")
        cv2.imshow("Rojo", mask_red)
    pixels_yellow = cv2.countNonZero(mask_yellow)
    if MAIN:
        print(f"Amarillo: {pixels_yellow} píxeles")
        cv2.imshow("Amarillo", mask_yellow)
    pixels_green = cv2.countNonZero(mask_green)
    if MAIN:
        print(f"Verde: {pixels_green} píxeles")
        while True:
            cv2.imshow("Verde", mask_green)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    min_pixels_threshold = int(
        roi_semaforo.shape[0] * roi_semaforo.shape[1] * 0.03)
    colors = {ColorSemaforo.ROJO: pixels_red,
              ColorSemaforo.AMARILLO: pixels_yellow, ColorSemaforo.VERDE: pixels_green}
    active_colors = {color: pixels for color,
                     pixels in colors.items() if pixels > min_pixels_threshold}

    if not active_colors:
        return ColorSemaforo.INDETERMINADO
    dominant_color = max(active_colors, key=active_colors.get)

    return dominant_color


# --- (Opcional) Código de prueba ---
if __name__ == '__main__':
    MAIN = True
    test_image = cv2.imread('cores-do-semaforo.jpg')
    if test_image is not None:
        color = analizar_color_semaforo_hsv(test_image)
        print(f"Color detectado en imagen de prueba: {color}")
    else:
        print("No se pudo cargar la imagen de prueba.")
    print("Archivo analisis_semaforo.py cargado. Contiene la función analizar_color_semaforo_hsv.")
    print("Ejecuta el archivo principal para usarlo.")
