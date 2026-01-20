import cv2
import cv2.aruco as aruco
import numpy as np
import time
import socket
import struct

# # -----------------------------
# # USER PARAMETERS
# # -----------------------------
# MARKER_SIZE = {
#     106: 100.0,   # mm
#     3:   67.0     # mm
# }

# # Camera calibration (replace with your own!)
# camera_matrix = np.array([[800, 0, 320],
#                           [0, 800, 240],
#                           [0,   0,   1]], dtype=float)

# dist_coeffs = np.zeros((5, 1))

# # -----------------------------
# # UDP SETUP
# # -----------------------------
# UDP_IP = "138.38.226.46"   # <-- change to your robot's IP
# UDP_PORT = 25000
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# # -----------------------------
# # INITIALISE
# # -----------------------------
# cap = cv2.VideoCapture(1)
# dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
# parameters = aruco.DetectorParameters()

# last_print_time = 0

# def rotation_vector_to_matrix(rvec):
#     R, _ = cv2.Rodrigues(rvec)
#     return R

# def get_marker_pose(corners, marker_id):
#     size = MARKER_SIZE[marker_id]
#     rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, size, camera_matrix, dist_coeffs)
#     return rvec[0][0], tvec[0][0]

# def draw_axis_manual(img, rvec, tvec, length=50):
#     axis = np.float32([
#         [0, 0, 0],
#         [length, 0, 0],
#         [0, length, 0],
#         [0, 0, length]
#     ])

#     imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)

#     origin = tuple(imgpts[0].ravel().astype(int))
#     x = tuple(imgpts[1].ravel().astype(int))
#     y = tuple(imgpts[2].ravel().astype(int))
#     z = tuple(imgpts[3].ravel().astype(int))

#     cv2.line(img, origin, x, (0, 0, 255), 3)
#     cv2.line(img, origin, y, (0, 255, 0), 3)
#     cv2.line(img, origin, z, (255, 0, 0), 3)

# # -----------------------------
# # MAIN LOOP
# # -----------------------------
# while True:
#     ret, frame = cap.read()
#     if not ret:
#         continue

#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)

#     marker_106_pose = None
#     marker_3_pose = None

#     if ids is not None:
#         aruco.drawDetectedMarkers(frame, corners, ids)

#         ids_list = ids.flatten().tolist()

#         # If marker 106 is not seen -> send distance=0, angle=180
#         if 106 not in ids_list:
#             packet = struct.pack("ff", float(0.0), float(180.0))
#             sock.sendto(packet, (UDP_IP, UDP_PORT))
#         else:
#             for i, marker_id in enumerate(ids_list):
#                 c = corners[i]

#                 if marker_id in MARKER_SIZE:
#                     rvec, tvec = get_marker_pose(c, marker_id)
#                     draw_axis_manual(frame, rvec, tvec, MARKER_SIZE[marker_id] / 2)

#                     if marker_id == 106:
#                         marker_106_pose = (rvec, tvec)
#                     elif marker_id == 3:
#                         marker_3_pose = (rvec, tvec)

#             # ANGLE + DISTANCE CALCULATION
#             if marker_106_pose and marker_3_pose:
#                 rvec106, tvec106 = marker_106_pose
#                 rvec3, tvec3 = marker_3_pose

#                 # Distance in mm
#                 distance_mm = np.linalg.norm(tvec3 - tvec106)

#                 # Signed angle
#                 R106 = rotation_vector_to_matrix(rvec106)
#                 y_axis_106 = R106[:, 1]
#                 z_axis_106 = R106[:, 2]

#                 vec_106_to_3 = (tvec3 - tvec106).astype(float)
#                 vec_106_to_3 /= np.linalg.norm(vec_106_to_3)

#                 a = y_axis_106 / np.linalg.norm(y_axis_106)
#                 b = vec_106_to_3 / np.linalg.norm(vec_106_to_3)

#                 numerator = np.dot(z_axis_106, np.cross(a, b))
#                 denominator = np.dot(a, b)
#                 angle_rad = np.arctan2(numerator, denominator)
#                 angle_deg = np.degrees(angle_rad)

#                 # If magnitude of angle > 30 -> send distance = 0
#                 send_distance = 0.0 if abs(angle_deg) > 30.0 else float(distance_mm)
#                 packet = struct.pack("ff", float(send_distance), float(angle_deg))
#                 sock.sendto(packet, (UDP_IP, UDP_PORT))

#                 # Optional: print once per second
#                 now = time.time()
#                 if now - last_print_time >= 1.0:
#                     print(f"Distance: {distance_mm:.1f} mm | Angle: {angle_deg:.2f}°")
#                     last_print_time = now
#     else:
#         # No markers detected at all -> treat as 106 out of frame
#         packet = struct.pack("ff", float(0.0), float(180.0))
#         sock.sendto(packet, (UDP_IP, UDP_PORT))

#     cv2.imshow("Aruco Tracking", frame)
#     if cv2.waitKey(1) & 0xFF == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()




# Script with ioannis changes
# -----------------------------
# USER PARAMETERS
# -----------------------------
MARKER_SIZE = {
    0: 100.0,   # mm
    3:   67.0     # mm
}

# Camera calibration (replace with your own!)
camera_matrix = np.array([[800, 0, 320],
                          [0, 800, 240],
                          [0,   0,   1]], dtype=float)

dist_coeffs = np.zeros((5, 1))

# -----------------------------
# INITIALISE
# -----------------------------
cap = cv2.VideoCapture(0)
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters()

last_print_time = 0

def rotation_vector_to_matrix(rvec):
    R, _ = cv2.Rodrigues(rvec)
    return R

def get_marker_pose(corners, marker_id):
    size = MARKER_SIZE[marker_id]
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, size, camera_matrix, dist_coeffs)
    return rvec[0][0], tvec[0][0]

def draw_axis_manual(img, rvec, tvec, length=50):
    """Draw XYZ axes manually using projectPoints."""
    axis = np.float32([
        [0, 0, 0],
        [length, 0, 0],
        [0, length, 0],
        [0, 0, length]
    ])

    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)

    origin = tuple(imgpts[0].ravel().astype(int))
    x = tuple(imgpts[1].ravel().astype(int))
    y = tuple(imgpts[2].ravel().astype(int))
    z = tuple(imgpts[3].ravel().astype(int))

    cv2.line(img, origin, x, (0, 0, 255), 3)   # X = red
    cv2.line(img, origin, y, (0, 255, 0), 3)   # Y = green
    cv2.line(img, origin, z, (255, 0, 0), 3)   # Z = blue

# -----------------------------
# MAIN LOOP
# -----------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)

    marker_106_pose = None
    marker_3_pose = None

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i]

            if marker_id in MARKER_SIZE:
                rvec, tvec = get_marker_pose(c, marker_id)

                # Draw axes manually
                draw_axis_manual(frame, rvec, tvec, MARKER_SIZE[marker_id] / 2)

                if marker_id == 0:  # 106:
                    marker_106_pose = (rvec, tvec)
                elif marker_id == 3:
                    marker_3_pose = (rvec, tvec)

        # -----------------------------
        # ANGLE CALCULATION
        # -----------------------------
        if marker_106_pose and marker_3_pose:
            rvec106, tvec106 = marker_106_pose
            rvec3, tvec3 = marker_3_pose

            R106 = rotation_vector_to_matrix(rvec106)
            y_axis_106 = R106[:, 1]

            vec_106_to_3 = (tvec3 - tvec106).astype(float)
            vec_106_to_3 /= np.linalg.norm(vec_106_to_3)

            dot = np.dot(y_axis_106, vec_106_to_3)
            dot = np.clip(dot, -1.0, 1.0)

            # -----------------------------------------
            a = y_axis_106 / np.linalg.norm(y_axis_106)
            b = vec_106_to_3 / np.linalg.norm(vec_106_to_3)
            n = R106[:, 2]                       # choose marker 106 Z-axis as rotation normal
            ig_angle_rad = np.arctan2(np.dot(n, np.cross(a, b)), np.dot(a, b))
            ig_angle_deg_signed = np.degrees(ig_angle_rad)   # range (-180, 180]
            # -----------------------------------------

            now = time.time()
            if now - last_print_time >= 1.0:
                print(f"Signed angle between marker 106 Y-axis and path to marker 3: {ig_angle_deg_signed:.2f}°")
                last_print_time = now

    cv2.imshow("Aruco Tracking", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()