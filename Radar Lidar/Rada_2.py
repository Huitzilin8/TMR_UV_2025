import cv2 as cv

# Definimos la ruta a los archivos Haar XML
face_cascade_path = 'TAREAS/haarcascade_frontalface_alt.xml'
eyes_cascade_path = 'TAREAS/haarcascade_eye_tree_eyeglasses.xml'

# Cargamos los clasificadores Haar
face_cascade = cv.CascadeClassifier(face_cascade_path)
eyes_cascade = cv.CascadeClassifier(eyes_cascade_path)

# Verificamos que los clasificadores se hayan cargado correctamente
if face_cascade.empty():
    print('Error: No se pudo cargar el clasificador de rostros.')
    exit(0)
if eyes_cascade.empty():
    print('Error: No se pudo cargar el clasificador de ojos.')
    exit(0)

# Creamos la funcion para detectar rostros y ojos en un fotograma
def detect_and_display(frame):
    # Convertimos el fotograma a escala de grises
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)

    # Detectamos rostros
    faces = face_cascade.detectMultiScale(frame_gray)
    for (x, y, w, h) in faces:

        # Dibujamos una elipse alrededor del rostro detectado
        center = (x + w // 2, y + h // 2)
        frame = cv.ellipse(frame, center, (w // 2, h // 2), 0, 0, 360, (0, 255, 0), 4)

        # Definimos la region de interes
        faceROI = frame_gray[y:y + h, x:x + w]

        # Detectamos ojos dentro del rostro
        eyes = eyes_cascade.detectMultiScale(faceROI)
        for (x2, y2, w2, h2) in eyes:
            eye_center = (x + x2 + w2 // 2, y + y2 + h2 // 2)
            radius = int(round((w2 + h2) * 0.25))
            frame = cv.circle(frame, eye_center, radius, (128, 0, 0), 4)

    # Mostramos el fotograma con las detecciones
    cv.imshow('Deteccion de Rostros y Ojos', frame)

# Abrimos la camara para capturar los frames
camera_device = 0
cap = cv.VideoCapture(camera_device)
if not cap.isOpened():
    print('Error: No se pudo abrir la cámara.')
    exit(0)

# Realizamos un bucle hasta apretar Esc
while True:
    ret, frame = cap.read()
    if frame is None:
        print('Error: No se capturó un fotograma. Saliendo...')
        break

    # Detectamos el rostro y los ojos
    detect_and_display(frame)

    # Definimos que al apretar Esc salgamos del programa
    if cv.waitKey(10) == 27:
        break

# Liberamos los recursos
cap.release()
cv.destroyAllWindows()
