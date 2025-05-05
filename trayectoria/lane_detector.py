import cv2
import numpy as np

def get_lane_correction(frame):
    frame = cv2.resize(frame, (640, 480))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(binary, kernel, iterations=1)

    height, width = eroded.shape
    histogram = np.sum(eroded[height//2:, :], axis=0)

    midpoint = width // 2
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    window_height = height // 10
    nonzero = eroded.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    leftx_current = leftx_base
    rightx_current = rightx_base
    centers = []

    for window in range(10):
        win_y_low = height - (window + 1) * window_height
        win_y_high = height - window * window_height
        win_xleft_low = leftx_current - 50
        win_xleft_high = leftx_current + 50
        win_xright_low = rightx_current - 50
        win_xright_high = rightx_current + 50

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]

        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        if len(good_left_inds) > 0:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > 0:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))

        if len(good_left_inds) > 0 and len(good_right_inds) > 0:
            center = int((leftx_current + rightx_current) / 2)
            centers.append((center, (win_y_low + win_y_high) // 2))

    if len(centers) >= 4:
        target = centers[3]
        correction = target[0] - frame.shape[1] // 2
        return correction
    else:
        return None
