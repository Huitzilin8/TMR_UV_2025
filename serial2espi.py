# -*- coding: utf-8 -*-
import serial
import time

# --- Variables de control que ya tienes ---
velocidad = 'd'  # Ejemplo de valor de velocidad
direccion = 'd'   # Ejemplo de valor de dirección
# -----------------------------------------

# --- Configuración de la conexión serial ---
# Reemplaza 'COM3' (Windows) o '/dev/ttyUSB0' (Linux/Mac)
# con el puerto serial correcto de tu ESP32.
# Puedes encontrarlo en el IDE de Arduino bajo Herramientas > Puerto.
PUERTO_SERIAL = '/dev/ttyTHS1' # O '/dev/ttyUSB0', '/dev/ttyACM0', etc.
BAUD_RATE = 9600    # Debe coincidir con la configuración en el ESP32

ser = None  # Inicializamos la variable ser

try:
    print("Intentando conectar a: " + PUERTO_SERIAL + " a " + str(BAUD_RATE) + " baudios.")
    ser = serial.Serial(PUERTO_SERIAL, BAUD_RATE, timeout=1)
    print("Conectado a " + ser.name)
    time.sleep(2) # Esperar un poco para que la conexión serial se establezca

    # Preparamos el dato a enviar. Formato: "velocidad,direccion\n"
    dato_a_enviar = str(velocidad)

    print("Enviando datos: '" + dato_a_enviar.strip() + "'")
    ser.write(dato_a_enviar.encode('utf-8')) # En Python 2, write espera un str (bytes)
                                         # .encode('ascii') es buena práctica para asegurar compatibilidad

    print("Datos enviados.")

except serial.SerialException as e:
    print("Error al abrir el puerto serial: " + str(e))
except Exception as e:
    print("Ocurrió un error: " + str(e))
finally:
    if ser and ser.is_open:
        ser.close()
        print("Conexión serial cerrada.")
