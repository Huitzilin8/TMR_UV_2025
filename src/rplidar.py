import serial
import time
import struct
import math
from collections import namedtuple

# Definición de constantes del protocolo
SYNC_BYTE = 0xA5
SYNC_BYTE2 = 0x5A

# Comandos
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_EXPRESS_SCAN = 0x82
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
CMD_GET_SAMPLERATE = 0x59
CMD_GET_LIDAR_CONF = 0x84

# Modos de respuesta
RESP_SINGLE = 0x0
RESP_MULTIPLE = 0x1

# Estructuras de datos
ScanPoint = namedtuple('ScanPoint', ['quality', 'angle', 'distance', 'start_flag'])
DeviceInfo = namedtuple('DeviceInfo', ['model', 'firmware', 'hardware', 'serialnumber'])
HealthStatus = namedtuple('HealthStatus', ['status', 'error_code'])

class RPLidar:
    """Clase para interactuar con RPLIDAR C1"""
    
    def __init__(self, port, baudrate=460800, timeout=1):
        """Inicializa la conexión con el LIDAR"""
        self.serial = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.motor_running = False
        self.scanning = False
        self._clear_buffer()
        
    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        self.disconnect()
        
    def _clear_buffer(self):
        """Limpia el buffer de entrada"""
        self.serial.reset_input_buffer()
        
    def disconnect(self):
        """Cierra la conexión serial"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            
    def _send_packet(self, cmd, payload=None):
        """Envía un paquete de comando al LIDAR"""
        if payload is None:
            payload = []
        
        # Construir el paquete
        packet = bytearray()
        packet.append(SYNC_BYTE)
        packet.append(cmd)
        packet.append(len(payload))
        packet.extend(payload)
        
        # Calcular checksum
        checksum = 0
        for b in packet:
            checksum ^= b
        packet.append(checksum)
        
        # Enviar paquete
        self.serial.write(packet)
        
    def _read_response_descriptor(self, timeout=1):
        """Lee el descriptor de respuesta del LIDAR con timeout"""
        start_time = time.time()
        
        # Buscar bytes de sincronización
        while time.time() - start_time < timeout:
            # Buscar el primer byte de sincronización
            sync1 = self.serial.read(1)
            if not sync1:
                continue
                
            if ord(sync1) == SYNC_BYTE:
                # Encontrado SYNC_BYTE, buscar SYNC_BYTE2
                sync2 = self.serial.read(1)
                if sync2 and ord(sync2) == SYNC_BYTE2:
                    # Leer el resto del descriptor (5 bytes)
                    descriptor = self.serial.read(5)
                    if len(descriptor) == 5:
                        # Parsear descriptor
                        data_length = (descriptor[0] | (descriptor[1] << 8)) & 0x3FFF  # 14 bits
                        send_mode = (descriptor[1] >> 6) & 0x3  # 2 bits
                        data_type = descriptor[2]  # 1 byte
                        
                        return {
                            'data_length': data_length,
                            'send_mode': send_mode,
                            'data_type': data_type
                        }
        
        raise RuntimeError("Timeout waiting for response descriptor")
        
    def _read_response_data(self, length, timeout=1):
        """Lee los datos de respuesta del LIDAR con timeout"""
        data = bytearray()
        start_time = time.time()
        
        while len(data) < length and (time.time() - start_time) < timeout:
            remaining = length - len(data)
            chunk = self.serial.read(remaining)
            if chunk:
                data.extend(chunk)
                
        if len(data) != length:
            raise RuntimeError(f"Incomplete response data (expected {length}, got {len(data)})")
            
        return data
        
    def start_motor(self):
        """Inicia el motor del LIDAR"""
        self._clear_buffer()
        self.motor_running = True
        
    def stop_motor(self):
        """Detiene el motor del LIDAR"""
        self._send_packet(CMD_STOP)
        self.motor_running = False
        self.scanning = False
        time.sleep(0.2)  # Esperar como indica el protocolo
        
    def reset(self):
        """Reinicia el LIDAR"""
        self._send_packet(CMD_RESET)
        self.motor_running = False
        self.scanning = False
        time.sleep(1.0)  # Esperar 1 segundo para reinicio completo
        
    def get_info(self):
        """Obtiene información del dispositivo"""
        self._clear_buffer()
        self._send_packet(CMD_GET_INFO)
        
        try:
            descriptor = self._read_response_descriptor()
            
            if descriptor['data_length'] != 20:
                raise RuntimeError("Unexpected data length for info response")
                
            data = self._read_response_data(20)
            
            model = data[0]
            firmware_minor = data[1]
            firmware_major = data[2]
            hardware = data[3]
            serialnumber = data[4:20]
            
            return DeviceInfo(
                model=model,
                firmware=(firmware_major, firmware_minor),
                hardware=hardware,
                serialnumber=serialnumber
            )
        except RuntimeError as e:
            self.reset()
            raise RuntimeError(f"Failed to get device info: {str(e)}")
        
    def get_health(self):
        """Obtiene el estado de salud del dispositivo"""
        self._clear_buffer()
        self._send_packet(CMD_GET_HEALTH)
        
        try:
            descriptor = self._read_response_descriptor()
            
            if descriptor['data_length'] != 3:
                raise RuntimeError("Unexpected data length for health response")
                
            data = self._read_response_data(3)
            
            status = data[0]
            error_code = (data[1] | (data[2] << 8))
            
            return HealthStatus(
                status=status,
                error_code=error_code
            )
        except RuntimeError as e:
            self.reset()
            raise RuntimeError(f"Failed to get health status: {str(e)}")
        
    def get_samplerate(self):
        """Obtiene la tasa de muestreo"""
        self._clear_buffer()
        self._send_packet(CMD_GET_SAMPLERATE)
        
        try:
            descriptor = self._read_response_descriptor()
            
            if descriptor['data_length'] != 4:
                raise RuntimeError("Unexpected data length for samplerate response")
                
            data = self._read_response_data(4)
            
            t_standard = (data[0] | (data[1] << 8))
            t_express = (data[2] | (data[3] << 8))
            
            return {
                'standard': t_standard,
                'express': t_express
            }
        except RuntimeError as e:
            self.reset()
            raise RuntimeError(f"Failed to get sample rate: {str(e)}")
        
    def start_scan(self, express_mode=False):
        """Inicia el modo de escaneo"""
        self._clear_buffer()
        
        if express_mode:
            # Modo express scan (más rápido)
            payload = [0x00, 0x00, 0x00, 0x00, 0x00]  # working_mode=0 (legacy)
            self._send_packet(CMD_EXPRESS_SCAN, payload)
        else:
            # Modo scan estándar
            self._send_packet(CMD_SCAN)
            
        self.scanning = True
        
        try:
            descriptor = self._read_response_descriptor(timeout=2)
            
            if express_mode and descriptor['data_length'] != 84:
                raise RuntimeError("Unexpected data length for express scan response")
            elif not express_mode and descriptor['data_length'] != 5:
                raise RuntimeError("Unexpected data length for standard scan response")
                
            return descriptor
        except RuntimeError as e:
            self.stop()
            raise RuntimeError(f"Failed to start scan: {str(e)}")
            
    def iter_scan(self, max_buf_meas=5000, express_mode=False):
        """Iterador para obtener puntos de escaneo"""
        if not self.scanning:
            self.start_scan(express_mode)
            
        if express_mode:
            return self._iter_express_scan(max_buf_meas)
        else:
            return self._iter_standard_scan(max_buf_meas)
            
    def _iter_standard_scan(self, max_buf_meas):
        """Iterador para escaneo estándar"""
        while self.scanning:
            try:
                # Leer 5 bytes por punto de escaneo
                data = self._read_response_data(5, timeout=0.5)
                
                # Parsear datos
                quality = data[0] >> 2
                start_flag = (data[0] >> 0) & 0x1
                check_bit = (data[0] >> 1) & 0x1
                
                if not check_bit:
                    continue  # Punto inválido
                    
                angle_q6 = (data[1] | (data[2] << 8)) >> 1
                angle = angle_q6 / 64.0
                
                distance_q2 = (data[3] | (data[4] << 8))
                distance = distance_q2 / 4.0
                
                yield ScanPoint(
                    quality=quality,
                    angle=angle,
                    distance=distance,
                    start_flag=bool(start_flag)
                )
            except RuntimeError:
                if not self.scanning:
                    break
                # Reintentar o manejar el error según sea necesario
                time.sleep(0.1)
                continue
                
    def _iter_express_scan(self, max_buf_meas):
        """Iterador para escaneo express (más rápido)"""
        while self.scanning:
            try:
                # Leer paquete completo (84 bytes)
                data = self._read_response_data(84, timeout=0.5)
                
                # Verificar bytes de sincronización
                sync1 = data[0] >> 4
                sync2 = data[1] >> 4
                
                if sync1 != 0xA or sync2 != 0x5:
                    continue  # Paquete inválido
                    
                # Calcular checksum (XOR de todos los bytes excepto los primeros 2)
                checksum = 0
                for b in data[2:]:
                    checksum ^= b
                    
                checksum_low = data[0] & 0x0F
                checksum_high = data[1] & 0x0F
                received_checksum = (checksum_high << 4) | checksum_low
                
                if checksum != received_checksum:
                    continue  # Checksum inválido
                    
                # Obtener ángulo de inicio
                start_angle_q6 = (data[2] | (data[3] << 8))
                start_angle = start_angle_q6 / 64.0
                
                # Leer cabinas (40 puntos por paquete)
                cabins = []
                for i in range(40):
                    offset = 4 + (i * 2)
                    cabin = data[offset] | (data[offset + 1] << 8)
                    cabins.append(cabin)
                    
                # Procesar cabinas
                for i, cabin in enumerate(cabins):
                    if cabin == 0:
                        continue  # Punto inválido
                        
                    angle = start_angle + (i * (360.0 / 40))  # Simplificación
                    angle = angle % 360.0
                    
                    distance = cabin  # distancia en mm
                    
                    yield ScanPoint(
                        quality=0,  # No disponible en express scan
                        angle=angle,
                        distance=distance,
                        start_flag=False
                    )
            except RuntimeError:
                if not self.scanning:
                    break
                # Reintentar o manejar el error según sea necesario
                time.sleep(0.1)
                continue
                
    def stop(self):
        """Detiene el escaneo y el motor"""
        if self.scanning:
            self.stop_motor()
        self._clear_buffer()