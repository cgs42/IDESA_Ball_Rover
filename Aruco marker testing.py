import cv2
import cv2.aruco as aruco
import numpy as np
import time
import matplotlib.pyplot as plt

# ===============================
# USER SETTINGS
# ===============================
TARGET_ID = 0
TEST_DURATION = 30        # seconds
PROCESSING_PERIOD = 0.25 # seconds

# ===============================
# Camera calibration
# ===============================
camera_calibration = np.load('Sample_Calibration.npz')
CM = camera_calibration['CM']
dist_coef = camera_calibration['dist_coef']

# ===============================
# ArUco setup (NEW API)
# ===============================
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# ===============================
# OpenCV setup
# ===============================
cap = cv2.VideoCapture(1)
cv2.namedWindow("Frame", cv2.WINDOW_AUTOSIZE)

# ===============================
# Data storage
# ===============================
timestamps = []
detections = []

start_time = time.time()
last_loop = start_time

print("Starting 30-second ArUco detection test...")

while True:
    now = time.time()
    elapsed = now - start_time

    if elapsed >= TEST_DURATION:
        print("Test complete.")
        break

    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(
    gray,
    aruco_dict,
    parameters=parameters
)


    seen = 0
    if ids is not None and TARGET_ID in ids.flatten():
        seen = 1
        aruco.drawDetectedMarkers(frame, corners, ids)

    # Store data
    timestamps.append(elapsed)
    detections.append(seen)

    # Display
    cv2.putText(frame, f"ID {TARGET_ID} detected: {seen}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0) if seen else (0, 0, 255),
                2)

    cv2.putText(frame, f"Time left: {TEST_DURATION - elapsed:.1f}s",
                (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2)

    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Test aborted by user.")
        break

    # Fixed processing rate
    loop_time = now - last_loop
    if loop_time < PROCESSING_PERIOD:
        time.sleep(PROCESSING_PERIOD - loop_time)
    last_loop = time.time()

# ===============================
# Cleanup OpenCV
# ===============================
cap.release()
cv2.destroyAllWindows()

# ===============================
# Convert to NumPy
# ===============================
timestamps = np.array(timestamps)
detections = np.array(detections)

# ===============================
# Statistics
# ===============================
total_samples = len(detections)
detections_seen = np.sum(detections)
dropouts = np.sum((detections[:-1] == 1) & (detections[1:] == 0))

print("\n===== RESULTS =====")
print(f"Samples: {total_samples}")
print(f"Detected frames: {detections_seen}")
print(f"Detection rate: {detections_seen / total_samples * 100:.1f}%")
print(f"Dropouts (1 → 0): {dropouts}")

# ===============================
# Plot AFTER test (SAFE)
# ===============================
plt.figure()
plt.step(timestamps, detections, where='post')
plt.ylim(-0.1, 1.1)
plt.xlabel("Time (s)")
plt.ylabel("Detection (1 = seen)")
plt.title(f"ArUco ID {TARGET_ID} Detection Over 30s")
plt.grid(True)
plt.show()
