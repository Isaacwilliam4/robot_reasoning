import cv2
import numpy as np
from collections import deque

# Webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam.")

# Background subtractor (works well for live demos)
bg = cv2.createBackgroundSubtractorMOG2(history=300, varThreshold=25, detectShadows=True)

# Simple temporal smoothing of the "occupied" signal (belief-like)
window = deque(maxlen=10)

print("Press 'q' to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)  # mirror for natural interaction
    h, w = frame.shape[:2]

    # Define a central "corridor" ROI (like the robot's forward path)
    roi_w, roi_h = int(w * 0.35), int(h * 0.55)
    roi_x1 = (w - roi_w) // 2
    roi_y1 = int(h * 0.25)
    roi_x2 = roi_x1 + roi_w
    roi_y2 = roi_y1 + roi_h

    # Foreground mask
    fg = bg.apply(frame)

    # Clean up mask (remove noise)
    fg = cv2.medianBlur(fg, 5)
    _, fg = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)  # remove shadows
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=1)
    fg = cv2.morphologyEx(fg, cv2.MORPH_DILATE, np.ones((7, 7), np.uint8), iterations=1)

    # Measure occupancy in ROI
    roi_mask = fg[roi_y1:roi_y2, roi_x1:roi_x2]
    occ_ratio = np.count_nonzero(roi_mask) / roi_mask.size

    # Binary detection with a threshold (this is your "belief boundary")
    occupied = occ_ratio > 0.03  # tune live in class (0.02–0.06 typical)
    window.append(1 if occupied else 0)

    # Smoothed confidence (belief-like probability estimate)
    confidence = sum(window) / len(window)

    # Decision rule: stop when confident
    stop = confidence >= 0.6  # tune live

    # Visualization
    vis = frame.copy()
    cv2.rectangle(vis, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 255, 255), 2)
    cv2.putText(vis, f"ROI occupancy: {occ_ratio:.3f}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(vis, f"Confidence: {confidence:.2f}", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    status = "STOP (OCCUPIED)" if stop else "GO (FREE)"
    color = (0, 0, 255) if stop else (0, 255, 0)
    cv2.putText(vis, status, (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)

    cv2.imshow("Webcam Obstacle Detection (ROI)", vis)
    cv2.imshow("Foreground Mask", fg)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
