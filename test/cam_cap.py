import cv2

# --- Configuration ---
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

print("Using GStreamer pipeline:")
print(pipeline)

# --- Attempt to open the camera using the GStreamer pipeline ---
# Note: cv2.CAP_GSTREAMER tells OpenCV to interpret the string as a pipeline
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# --- Check if the camera opened successfully ---
if not cap.isOpened():
    print("Error: Could not open camera using GStreamer pipeline.")
    print("Check pipeline string, camera connection, and GStreamer/OpenCV integration.")
    exit()
else:
    print("CSI Camera opened successfully via GStreamer.")

# --- Main Loop: Capture and Display Frames ---
try:
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        # Check if the frame was read successfully
        if not ret:
            print("Error: Failed to grab frame. Exiting.")
            break

        # Display the frame in a window
        cv2.imshow(window_title, frame)

        # Wait for 1ms and check if the 'q' key was pressed to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exit key pressed. Closing.")
            break
finally:
    # --- Release resources ---
    print("Releasing camera resource.")
    cap.release()
    print("Destroying OpenCV windows.")
    cv2.destroyAllWindows()

print("Script finished.")
