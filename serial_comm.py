# -*- coding: utf-8 -*-
import serial
import time

# Configuración de la conexión serial con ESP32
PUERTO_SERIAL = '/dev/ttyTHS1'  # O '/dev/ttyUSB0', '/dev/ttyACM0'
BAUD_RATE = 9600                # Debe coincidir con la configuración en el ESP32

ser = None  # Inicializamos la variable ser

# Función para enviar los datos de dirección y velocidad al ESP32
def enviar_datos_esp(velocidad, direccion):
    try:
        if ser is None or not ser.is_open:
            ser.open()

        dato_a_enviar = f"{velocidad},{direccion}\n"
        ser.write(dato_a_enviar.encode('utf-8'))
        print(f"Datos enviados: {dato_a_enviar.strip()}")
    except Exception as e:
        print(f"Error al enviar datos: {e}")


def main():
    # Conectar al puerto serial
    try:
        ser = serial.Serial(PUERTO_SERIAL, BAUD_RATE, timeout=1)
        print(f"Conectado a {ser.name}")
        time.sleep(2)  # Esperar un poco para que la conexión serial se establezca
    except serial.SerialException as e:
        print(f"Error al abrir el puerto serial: {e}")
    except Exception as e:
        print(f"Error en la conexión: {e}")

    # Enviar dirección y velocidad (ejemplo de valores)
    velocidad = 'i'  # Puedes usar este valor dependiendo de la detección
    direccion = 'd'  # Ejemplo de dirección (izquierda/derecha)
    enviar_datos_esp(velocidad, direccion)

    if ser and ser.is_open:
        ser.close()

if __name__ == "__main__":
    main()
