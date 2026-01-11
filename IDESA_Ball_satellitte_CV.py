# UI is split into two columns. The left column shows the live satellite feed. 
# When the button 'Find Minerals' is pressed, the robot will 'collect minerals' defined by aruco code IDs 2 to 5. 
# It does this by creating the most optimal path between identified minerals and drawing it on the GUI.
# It then communicates the desired position it would like the robot to go to over the MQTT. Once the first position is reached it will then send the next position and so on until all minerals have been collected.
# If a crisis event aruco code (desfined by IDs 10,11,12) is identified the robot will respond accordingly. The crisis event colour will appear below the camera feed to inform the user.
# The right column contains the manual override D-pad which when toggled on allows the user to manually control the robot.

# Crisis events are defined as follows:
# ID 10 (appears as orange in GUI) - A localised weather event is occuring, continue to collect minerals but avoid the event and adapt the path around it. 
# ID 11 (appears as red in GUI) - A severe weather event is occuring, stop all mineral collection and return to base. 
# ID 12 (appears as green in GUI) - An indentifiable moving object has been spotted by the satellite, interrupt mineral collection and go and investigate the object.

import tkinter as tk
from PIL import Image, ImageTk
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import queue
import threading
import socket
import struct

# =========================================================
# CAMERA & ARUCO SETUP
# =========================================================
camera_calibration = np.load('workdir/Calibration.npz')
CM = camera_calibration['CM']
dist_coef = camera_calibration['dist_coef']

marker_size = 40
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

cap = cv2.VideoCapture(1)

# =========================================================
# UDP SETUP
# =========================================================
UDP_IP = "172.26.198.126"
UDP_PORT = 25000
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

UDP_RATE_HZ = 1.0
last_udp_time = 0.0

# =========================================================
# MINERAL STATE
# =========================================================
MINERAL_IDS = [1, 2, 3, 4, 5]
COLLECTION_RADIUS_PX = 30
TARGET_PAUSE_SEC = 5.0
DETECTION_TIME_SEC = 5.0

mineral_map = {}
remaining_minerals = []
path_order = []

current_target_id = None
collected_mineral_id = None

system_active = False
waiting_for_next_target = False
target_reached_time = 0.0

detecting_minerals = False
detection_start_time = 0.0

# =========================================================
# PATH PLANNING
# =========================================================
def compute_shortest_path(start_pos, mineral_items):
    path = []
    current = start_pos
    items = mineral_items.copy()

    while items:
        mid, pos = min(
            items,
            key=lambda x: math.hypot(current[0] - x[1][0], current[1] - x[1][1])
        )
        path.append(mid)
        current = pos
        items = [i for i in items if i[0] != mid]

    return path

# =========================================================
# TKINTER UI
# =========================================================
root = tk.Tk()
root.title("IDESA Ball Rover UI")
root.geometry("1000x600")
root.configure(bg="black")

left_frame = tk.Frame(root, bg="black")
left_frame.pack(side="left", fill="both", expand=True)

def toggle_mineral_find():
    global system_active, mineral_map, remaining_minerals, path_order
    global current_target_id, waiting_for_next_target
    global detecting_minerals, detection_start_time

    system_active = not system_active

    if system_active:
        print("\n[SYSTEM] MINERAL FIND STARTED")
        mineral_map.clear()
        remaining_minerals.clear()
        path_order.clear()
        current_target_id = None
        waiting_for_next_target = False
        detecting_minerals = True
        detection_start_time = time.time()
        button.config(text="End Mineral Find")
    else:
        print("\n[SYSTEM] MINERAL FIND STOPPED")
        button.config(text="Find Minerals")
        current_target_id = None
        waiting_for_next_target = False
        detecting_minerals = False

button = tk.Button(
    left_frame,
    text="Find Minerals",
    bg="#222",
    fg="white",
    font=("Arial", 14),
    command=toggle_mineral_find
)
button.pack(pady=10)

camera_label = tk.Label(left_frame, bg="black")
camera_label.pack()

# =========================================================
# THREADING
# =========================================================
frame_queue = queue.Queue(maxsize=1)
stop_event = threading.Event()

# =========================================================
# CAMERA THREAD
# =========================================================
def camera_thread():
    global last_udp_time, current_target_id
    global waiting_for_next_target, target_reached_time
    global collected_mineral_id, detecting_minerals

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        robot_pos = None
        robot_yaw = None

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, marker_size, CM, dist_coef
            )

            for i, mid in enumerate(ids.flatten()):
                c = corners[i][0]
                cx, cy = np.mean(c[:, 0]), np.mean(c[:, 1])

                if mid == 0:
                    robot_pos = (cx, cy)
                    R, _ = cv2.Rodrigues(rvecs[i])
                    robot_yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))

                if detecting_minerals and mid in MINERAL_IDS:
                    mineral_map.setdefault(mid, (cx, cy))

                frame = cv2.drawFrameAxes(
                    frame, CM, dist_coef, rvecs[i], tvecs[i], 100
                )

            frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # Finish detection phase
        if detecting_minerals and time.time() - detection_start_time >= DETECTION_TIME_SEC:
            detecting_minerals = False
            remaining_minerals[:] = list(mineral_map.keys())

            if robot_pos:
                path_order[:] = compute_shortest_path(
                    robot_pos,
                    [(mid, mineral_map[mid]) for mid in remaining_minerals]
                )
                print("[PATH] Optimal order:", path_order)

        # Select target
        if system_active and not detecting_minerals and current_target_id is None and not waiting_for_next_target:
            if path_order:
                current_target_id = path_order[0]
                print("[NAV] Target:", current_target_id)

        # Collection
        if system_active and robot_pos and current_target_id and not waiting_for_next_target:
            tx, ty = mineral_map[current_target_id]
            if math.hypot(robot_pos[0] - tx, robot_pos[1] - ty) < COLLECTION_RADIUS_PX:
                collected_mineral_id = current_target_id
                if current_target_id in remaining_minerals:
                    remaining_minerals.remove(current_target_id)
                if path_order and path_order[0] == current_target_id:
                    path_order.pop(0)
                current_target_id = None
                waiting_for_next_target = True
                target_reached_time = time.time()
                print("[COLLECT] Mineral reached")

        # Pause
        if waiting_for_next_target and time.time() - target_reached_time >= TARGET_PAUSE_SEC:
            waiting_for_next_target = False
            collected_mineral_id = None

        # UDP
        if system_active and robot_pos and robot_yaw is not None and current_target_id:
            now = time.time()
            if now - last_udp_time >= 1.0 / UDP_RATE_HZ:
                tx, ty = mineral_map[current_target_id]
                dx, dy = tx - robot_pos[0], ty - robot_pos[1]
                target_angle = math.degrees(math.atan2(dx, -dy))
                relative_angle = (target_angle - robot_yaw + 180) % 360 - 180
                udp_sock.sendto(
                    struct.pack('<fff', robot_pos[0], robot_pos[1], relative_angle),
                    (UDP_IP, UDP_PORT)
                )
                print(f"[UDP] x={robot_pos[0]:.1f} y={robot_pos[1]:.1f} angle={relative_angle:.1f}")
                last_udp_time = now

        if not frame_queue.full():
            frame_queue.put((frame, robot_pos))

# =========================================================
# UI LOOP
# =========================================================
def update_ui():
    if not frame_queue.empty():
        frame, robot_pos = frame_queue.get()

        # Draw full path
        if robot_pos and path_order:
            pts = [robot_pos] + [mineral_map[mid] for mid in path_order]
            for i in range(len(pts) - 1):
                cv2.line(frame,
                         (int(pts[i][0]), int(pts[i][1])),
                         (int(pts[i+1][0]), int(pts[i+1][1])),
                         (255, 0, 0), 2)

        # Countdown
        if waiting_for_next_target:
            remaining = max(0, int(TARGET_PAUSE_SEC - (time.time() - target_reached_time)))
            cv2.putText(frame, f"Next target in {remaining}s",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 255), 2)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = ImageTk.PhotoImage(Image.fromarray(frame).resize((480, 360)))
        camera_label.imgtk = img
        camera_label.config(image=img)

    root.after(30, update_ui)

# =========================================================
# CLEANUP
# =========================================================
def on_close():
    stop_event.set()
    cap.release()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

threading.Thread(target=camera_thread, daemon=True).start()
update_ui()
root.mainloop()

