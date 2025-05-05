import threading, queue, time, cv2, numpy as np
from detecciones.detector_senalamientos import run_deteccion_senalamientos
from trayectoria.lane_detector import get_lane_correction_and_centers
from nav.Rada_2 import LidarController
from control.serial_comm import ESP32Controller

# Cola global y evento de stop
data_q    = queue.Queue(maxsize=100)
stop_evt  = threading.Event()

# Hilos
def hilo_senalamientos(): run_deteccion_senalamientos(stop_evt, data_q, camera_index=0)
def hilo_lidar(): 
    l = LidarController(port='/dev/ttyUSB0', data_queue=data_q, stop_event=stop_evt)
    if l.start_collection(): l.run()
def hilo_carriles():
    cap = cv2.VideoCapture(0)
    while not stop_evt.is_set():
        ret, frame = cap.read()
        if not ret: continue
        corr, centers = get_lane_correction_and_centers(frame)
        data_q.put({'tipo':'carriles','datos':{'correccion':corr,'centros':centers}})
    cap.release()

if __name__=='__main__':
    esp = ESP32Controller()
    # lanzar hilos daemon
    for fn in (hilo_senalamientos, hilo_lidar, hilo_carriles):
        t = threading.Thread(target=fn, daemon=True); t.start(); time.sleep(0.2)

    # estado global
    obstacle=False; sem_color=None; lane_corr=0.0

     try:
        while True:
            # Leer cola
            try:
                item = data_q.get(block=False)
                tp, dt = item['tipo'], item['datos']
                if tp=='lidar':
                    front = [d for (a,d) in dt if -10<=a<=10]
                    obstacle = any(d<300 for d in front)
                elif tp=='senalamientos':
                    pass  # aquí podrías procesar semáforo
                elif tp=='carriles':
                    lane_corr = dt['correccion'] or 0.0
            except queue.Empty:
                pass

            # Lógica simple
            if obstacle:
                esp.send_action('STOP')
            else:
                esp.send_direction(lane_corr/160.0)
                esp.send_action('FORWARD', 0.5)

            if cv2.waitKey(1)&0xFF==ord('q'):
                stop_evt.set()
                break
            time.sleep(0.01)
    except KeyboardInterrupt:
        stop_evt.set()
    finally:
        esp.send_action('STOP')
        esp.close()
        cv2.destroyAllWindows()
