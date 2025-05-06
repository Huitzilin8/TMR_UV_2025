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
    """
    Analiza el ROI de un semáforo para determinar el color de la luz activa (Rojo, Amarillo, Verde)
    utilizando el espacio de color HSV.

    Args:
        roi_semaforo: La imagen (en formato BGR de OpenCV) del área del semáforo detectado.

    Returns:
        ColorSemaforo: Enum que representa el color detectado (Rojo, Amarillo, Verde o Indeterminado).
    """
    if roi_semaforo is None or roi_semaforo.size == 0:
        return ColorSemaforo.INDETERMINADO # ROI vacío o no válido
    if MAIN:
        print(f"semafro: {roi_semaforo.size} pixeles")
        cv2.imshow(f"semaforo", roi_semaforo)

    # Convertir el ROI a espacio de color HSV
    hsv = cv2.cvtColor(roi_semaforo, cv2.COLOR_BGR2HSV)

    # --- Definir rangos de color en HSV ---
    # Estos rangos pueden necesitar ajustes finos dependiendo de la cámara y las condiciones de luz

    # Rojo (nota: el rojo puede estar en dos rangos en el espacio HUE)
    lower_red1 = np.array([0, 150, 120])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 150, 120])
    upper_red2 = np.array([180, 255, 255])

    # Amarillo
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255]) # Ampliado un poco el rango de HUE

    # Verde
    lower_green = np.array([40, 100, 100]) # Ajustado el límite inferior de HUE
    upper_green = np.array([85, 255, 255]) # Ajustado el límite superior de HUE


    # --- Crear máscaras para cada color ---
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2) # Combinar los dos rangos de rojo

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # --- Calcular cuántos píxeles coinciden con cada máscara ---
    # Usaremos el número de píxeles blancos en cada máscara
    pixels_red = cv2.countNonZero(mask_red)
    if MAIN:
        print(f"Rojo: {pixels_red} píxeles") # Mostrar el conteo de píxeles rojos (opcional)
        cv2.imshow("Rojo", mask_red)
    pixels_yellow = cv2.countNonZero(mask_yellow)
    if MAIN:
        print(f"Amarillo: {pixels_yellow} píxeles")
        cv2.imshow(f"Amarill", mask_yellow)
    pixels_green = cv2.countNonZero(mask_green)
    if MAIN:
        print(f"Verde: {pixels_green} píxeles")
        while True:
            cv2.imshow("Verde", mask_green)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    # --- Determinar el color dominante ---
    # Establecer un umbral mínimo de píxeles para considerar una luz como activa
    # Esto ayuda a evitar detecciones erróneas por ruido o reflejos pequeños
    # Puedes ajustar este valor (depende del tamaño esperado de la luz en el ROI)
    min_pixels_threshold = int(roi_semaforo.shape[0] * roi_semaforo.shape[1] * 0.03) # Ejemplo: 3% del area del ROI

    colors = {ColorSemaforo.ROJO: pixels_red, ColorSemaforo.AMARILLO: pixels_yellow, ColorSemaforo.VERDE: pixels_green}

    # Filtrar colores que no superan el umbral
    active_colors = {color: pixels for color, pixels in colors.items() if pixels > min_pixels_threshold}

    if not active_colors:
        return ColorSemaforo.INDETERMINADO# Ningún color supera el umbral mínimo

    # Devolver el color con la mayor cantidad de píxeles activos
    dominant_color = max(active_colors, key=active_colors.get)

    return dominant_color

# --- (Opcional) Código de prueba ---
if __name__ == '__main__':
    # Puedes poner aquí una imagen de prueba de un semáforo si quieres probar la función directamente
    # Ejemplo:
    MAIN = True
    test_image = cv2.imread('cores-do-semaforo.jpg')
    if test_image is not None:
        color = analizar_color_semaforo_hsv(test_image)
        print(f"Color detectado en imagen de prueba: {color}")
    else:
        print("No se pudo cargar la imagen de prueba.")
    print("Archivo analisis_semaforo.py cargado. Contiene la función analizar_color_semaforo_hsv.")
    print("Ejecuta el archivo principal (detector_webcam_con_analisis.py) para usarlo.")
