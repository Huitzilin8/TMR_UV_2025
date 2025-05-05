# nav/Rada_2.py
from rplidar import RPLidar

class LidarController:
    def __init__(self, port, data_queue, stop_event):
        self.lidar = RPLidar(port)
        self.q = data_queue
        self.stop = stop_event

    def start_collection(self):
        self.lidar.start_motor()
        self.generator = self.lidar.iter_scans()
        return True

    def run(self):
        for scan in self.generator:
            if self.stop.is_set(): break
            # scan = [(quality, angle, dist), ...]
            # resumimos a solo (angle,dist)
            pts = [(float(a), float(d)) for (_,a,d) in scan]
            self.q.put({'tipo':'lidar','datos':pts})

    def stop(self):
        self.lidar.stop_motor()
        self.lidar.disconnect()
