# trabaja lane detection y serial comm (basicos)

import subprocess
import sys

def run_lane_detection():
    subprocess.call([sys.executable, "lane_detection.py"])  # Ejecutar script Python 2.7

def run_serial_comm():
    subprocess.call([sys.executable.replace('python2.7', 'python3'), "serial_comm.py"])  # Ejecutar script Python 3

if __name__ == "__main__":
    # Ejecutar ambos scripts en paralelo
    run_lane_detection()
    run_serial_comm()
