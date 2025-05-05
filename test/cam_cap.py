# --- process_a_capture.py (Run with system Python 3.6/3.8) ---
import cv2
import zmq
import numpy as np
import time

# GStreamer pipeline (as in the previous example)
pipeline = "nvarguscamerasrc ! ..." # Your full pipeline string
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# ZeroMQ setup
context = zmq.Context()
socket = context.socket(zmq.PUB) # Or zmq.PUSH
socket.bind("tcp://*:5555") # Bind to a port

if not cap.isOpened():
    print("Error: Cannot open camera")
    exit()

print("Capture process started. Publishing frames...")
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Send frame data (you might need to serialize/compress)
    # Simple approach: send metadata and raw bytes
    h, w, c = frame.shape
    socket.send_multipart([
        f"{h},{w},{c}".encode('utf-8'), # Send dimensions
        frame.tobytes()                  # Send raw pixel data
    ])
    # time.sleep(0.01) # Optional: control rate if needed

cap.release()
socket.close()
context.term()
print("Capture process finished.")
