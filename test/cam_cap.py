# --- process_a_capture.py (Run with system Python 3.6/3.8) ---
import cv2
import zmq
import numpy as np
import time

# Adjust these based on your camera's capabilities and desired output
capture_width = 1280
capture_height = 720
display_width = 1280 # Can be same or different for display scaling
display_height = 720
framerate = 30
flip_method = 0  # 0=Normal, 1=Rotate 180, 2=Horiz Flip, 3=Vert+Horiz Flip

window_title = "CSI Camera Feed"

# --- Construct the GStreamer pipeline string ---
# This pipeline uses nvarguscamerasrc (NVIDIA's optimized source for CSI)
# and nvvidconv for efficient format conversion.
pipeline = (
    f"nvarguscamerasrc ! "
    f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, format=(string)NV12, framerate=(fraction){framerate}/1 ! "
    f"nvvidconv flip-method={flip_method} ! "
    f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
    f"videoconvert ! "
    f"video/x-raw, format=(string)BGR ! appsink drop=true"
)
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
