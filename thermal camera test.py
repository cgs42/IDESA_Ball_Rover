import cv2
import numpy as np

# Try 0 first; if black screen try 1 or 2
CAMERA_INDEX = 1

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

if not cap.isOpened():
    raise RuntimeError("Could not open Arducam USB camera")

# Optional: set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale (intensity ≈ heat proxy)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Improve contrast
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)

    # Apply thermal color map
    thermal = cv2.applyColorMap(gray, cv2.COLORMAP_JET)

    # Show result
    cv2.imshow("Arducam Thermal View (Simulated)", thermal)

    # Exit with Q or ESC
    key = cv2.waitKey(1)
    if key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()
