# -*- coding: utf-8 -*-
import serial
import time
import queue

# Configuración de la conexión serial con ESP32
PUERTO_SERIAL = '/dev/ttyTHS1'  # O '/dev/ttyUSB0', '/dev/ttyACM0'
BAUD_RATE = 9600                # Debe coincidir con la configuración en el ESP32

ser = None  # Inicializamos la variable ser

# Función para enviar los datos de dirección y velocidad al ESP32
def enviar_datos_esp(direccion):
    try:
        if ser is None or not ser.is_open:
            ser.open()

        dato_a_enviar = f"{direccion}\n"
        ser.write(dato_a_enviar.encode('utf-8'))
        print(f"Datos enviados: {dato_a_enviar.strip()}")
    except Exception as e:
        print(f"Error al enviar datos: {e}")

# Función para leer datos de la cola y enviarlos a través del puerto serial
def procesar_datos_serial(queue):
    try:
        while True:
            if not queue.empty():
                correction = queue.get()  # Obtener la corrección de la dirección
                enviar_datos_esp(correction)
            time.sleep(0.1)  # Hacer una pequeña pausa para no sobrecargar la CPU

    except Exception as e:
        print(f"Error al procesar los datos seriales: {e}")

def main():
    global ser
    # Conectar al puerto serial
    try:
        ser = serial.Serial(PUERTO_SERIAL, BAUD_RATE, timeout=1)
        print(f"Conectado a {ser.name}")
        time.sleep(2)  # Esperar un poco para que la conexión serial se establezca
    except serial.SerialException as e:
        print(f"Error al abrir el puerto serial: {e}")
    except Exception as e:
        print(f"Error en la conexión: {e}")

    # Crear una cola para pasar los datos entre procesos
    data_queue = queue.Queue()

    # Iniciar el procesamiento de carriles
    from lane_detection import start_cameras
    threading.Thread(target=start_cameras, args=(data_queue,), daemon=True).start()

    # Iniciar el proceso de comunicación serial
    procesar_datos_serial(data_queue)

if __name__ == "__main__":
    main()
