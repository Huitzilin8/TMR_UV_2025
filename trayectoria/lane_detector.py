# trayectoria/lane_detector.py
import cv2
import numpy as np

def get_lane_correction_and_centers(frame, 
        target_size=(640,480), threshold=180, nwindows=10, margin=50, visualize=False):
    """
    Retorna:
      - correction (int px) o None
      - centers: lista de (x,y) de cada ventana
    """
    frame = cv2.resize(frame, target_size)
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, bin = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    ker = np.ones((3,3), np.uint8)
    er  = cv2.erode(bin, ker, iterations=1)

    h, w = er.shape
    hist = np.sum(er[h//2:,:], axis=0)
    mid  = w//2
    leftx_base  = np.argmax(hist[:mid])
    rightx_base = np.argmax(hist[mid:]) + mid

    win_h = h // nwindows
    nonzero = er.nonzero()
    ys, xs = nonzero[0], nonzero[1]

    leftx, rightx = leftx_base, rightx_base
    centers = []

    for win in range(nwindows):
        ylow = h - (win+1)*win_h
        yhigh= h - win*win_h
        xl, xh = leftx-margin,  leftx+margin
        rl, rh = rightx-margin, rightx+margin

        goodL = np.where((ys>=ylow)&(ys<yhigh)&(xs>=xl)&(xs<xh))[0]
        goodR = np.where((ys>=ylow)&(ys<yhigh)&(xs>=rl)&(xs<rh))[0]

        if len(goodL)>0: leftx  = int(xs[goodL].mean())
        if len(goodR)>0: rightx = int(xs[goodR].mean())

        if len(goodL)>0 and len(goodR)>0:
            c = (leftx+rightx)//2
            centers.append((c, (ylow+yhigh)//2))

    if len(centers)>=4:
        target = centers[3]
        correction = target[0] - (w//2)
        return correction, centers
            
# Visualización opcional
    if visualize:
        vis = frame.copy()
        for i in range(1, len(centers)):
            cv2.line(vis, centers[i-1], centers[i], (0,255,0), 2)
        for c in centers:
            cv2.circle(vis, c, 4, (0,0,255), -1)
        cv2.imshow("Visión Carriles", vis)
        cv2.waitKey(1)
                
    return None, centers
