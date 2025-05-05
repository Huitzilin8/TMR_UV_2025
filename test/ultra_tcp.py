import cv2
import zmq
import numpy as np
# import ultralytics # Your ultralytics imports

# ZeroMQ setup
context = zmq.Context()
socket = context.socket(zmq.SUB) # Or zmq.PULL
socket.connect("tcp://localhost:5555")
socket.subscribe("") # Subscribe to all messages

print("Ultralytics process started. Waiting for frames...")
while True:
    # Receive frame data
    multipart_message = socket.recv_multipart()
    meta_str = multipart_message[0].decode('utf-8')
    frame_bytes = multipart_message[1]

    # Reconstruct frame
    h, w, c = map(int, meta_str.split(','))
    frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((h, w, c))

    # --- Perform Ultralytics detection here ---
    # results = model(frame)
    # processed_frame = ... # Draw boxes etc.
    print(f"Received frame with shape: {frame.shape}") # Replace with actual processing

    # Display (optional)
    cv2.imshow("Received Frame", frame) # Or processed_frame
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
socket.close()
context.term()
print("Ultralytics process finished.")
