import tkinter as tk
from PIL import Image, ImageTk
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import queue
import json
import threading
import paho.mqtt.client as mqtt
import socket 
# =========================================================
# CAMERA & ARUCO SETUP
# =========================================================
camera_calibration = np.load('workdir/Calibration.npz')
CM = camera_calibration['CM']
dist_coef = camera_calibration['dist_coef']

marker_size = 40
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

cap = cv2.VideoCapture(0)

# =========================================================
# UDP SETUP
# UDP is used to send lower level messages constantly as it is fast.
# This includes the constant update of the robots location and its angle
# =========================================================
UDP_IP = "172.26.198.126"  # Computer's IP
UDP_PORT1 = 25000
UDP_PORT2 = 50001
MESSAGE = message 

# =========================================================
# MQTT SETUP
# The MQTT server will be used to send more top level pieces of information.
# This includes the desired position of the minerals for the robot to go to,a nd the crisis event type.
# =========================================================
BROKER = "fesv-mqtt.bath.ac.uk"
PORT = 31415
USERNAME = "student"
PASSWORD = "HousekeepingGlintsStreetwise"

TOPIC_DESIRED_POSITION = "CottonCandyGrapes/BallRobot/DesiredPosition"
TOPIC_CURRENT_POSITION = "CottonCandyGrapes/BallRobot/CurrentPosition"
TOPIC_CURRENT_ANGLE = "CottonCandyGrapes/BallRobot/CurrentAngle"

MQTT_RATE_HZ = 10
last_mqtt_time = 0.0

mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(USERNAME, PASSWORD)
mqtt_client.connect(BROKER, PORT, 60)
mqtt_client.loop_start()

print("[MQTT] Connected to broker")

# =========================================================
# TKINTER UI
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
# =========================================================
root = tk.Tk()
root.title("IDESA Ball Rover UI")
root.geometry("1000x600")
root.configure(bg="black")

left_frame = tk.Frame(root, bg="black")
right_frame = tk.Frame(root, bg="black")
left_frame.pack(side="left", fill="both", expand=True)
right_frame.pack(side="right", fill="both", expand=True)

tk.Button(
    left_frame, text="Find Minerals", bg="#222", fg="white", font=("Arial", 14)
).pack(pady=10)

camera_label = tk.Label(left_frame, bg="black")
camera_label.pack(pady=10)

tk.Label(
    right_frame, text="Manual Override",
    bg="black", fg="white", font=("Arial", 16)
).pack(pady=20)

manual_override = False

# Joystick on right side for manual override. Can only be used when enabled.
class JoystickUI:
    def __init__(self, parent, q):
        self.q = q
        self.is_on = False
        self.canvas_size = 200
        self.radius = 80

        self.canvas = tk.Canvas(
            parent, width=200, height=200,
            bg="lightgrey", highlightthickness=0
        )
        self.canvas.pack(pady=10)

        c = 100
        self.center = c
        self.canvas.create_oval(c-80, c-80, c+80, c+80, outline="black")

        self.canvas.bind("<B1-Motion>", self.move)
        self.canvas.bind("<ButtonRelease-1>", self.release)

    def set_enabled(self, enabled):
        self.is_on = enabled
        print(f"[JOYSTICK] {'ENABLED' if enabled else 'DISABLED'}")

    def move(self, event):
        if not self.is_on:
            return
        dx = event.x - self.center
        dy = event.y - self.center
        dist = math.sqrt(dx**2 + dy**2)
        angle = math.degrees(math.atan2(-dy, dx))
        speed = min(dist / self.radius, 1.0) * 100
        print(f"[JOYSTICK] angle={angle:.1f} speed={speed:.1f}")
        self.q.put((angle, speed))

    def release(self, event):
        if self.is_on:
            print("[JOYSTICK] CENTER speed=0")
            self.q.put((0, 0))

control_queue = queue.Queue()
joystick = JoystickUI(right_frame, control_queue)

def toggle_manual():
    global manual_override
    manual_override = not manual_override
    toggle_button.config(text="ON" if manual_override else "OFF")
    joystick.set_enabled(manual_override)

toggle_button = tk.Button(
    right_frame, text="OFF", bg="#222", fg="white",
    font=("Arial", 14), command=toggle_manual
)
toggle_button.pack(pady=10)

# =========================================================
# THREAD COMMUNICATION
# =========================================================
frame_queue = queue.Queue(maxsize=1)
stop_event = threading.Event()

# =========================================================
# CAMERA + MQTT THREAD
# =========================================================
def camera_thread():
    global last_mqtt_time

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            continue

        h, w, _ = frame.shape
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters
        )

        robot_pos = None
        robot_angle = None
        minerals = []

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, marker_size, CM, dist_coef
            )

            for i, mid in enumerate(ids.flatten()):
                c = corners[i][0]
                cx, cy = np.mean(c[:, 0]), np.mean(c[:, 1])
                x, y = cx / w, cy / h

                if mid == 1:
                    robot_pos = (float(x), float(y))
                    R, _ = cv2.Rodrigues(rvecs[i])
                    robot_angle = float(
                        math.degrees(math.atan2(R[1, 0], R[0, 0]))
                    )

                elif 2 <= mid <= 5:
                    minerals.append((int(mid), float(x), float(y)))

                frame = cv2.drawFrameAxes(
                    frame, CM, dist_coef, rvecs[i], tvecs[i], 100
                )

            frame = aruco.drawDetectedMarkers(frame, corners, ids)

        now = time.time()
        if now - last_mqtt_time >= 1.0 / MQTT_RATE_HZ:
            last_mqtt_time = now

            if robot_pos:
                payload = {"x": robot_pos[0], "y": robot_pos[1]}
                print(f"[MQTT] RobotPos → {payload}")
                mqtt_client.publish(
                    TOPIC_CURRENT_POSITION, json.dumps(payload)
                )

            if robot_angle is not None:
                payload = {"angle": robot_angle}
                print(f"[MQTT] RobotAngle → {payload}")
                mqtt_client.publish(
                    TOPIC_CURRENT_ANGLE, json.dumps(payload)
                )

            if minerals:
                minerals.sort()
                mid, mx, my = minerals[0]
                payload = {"x": mx, "y": my, "id": mid}
                print(f"[MQTT] Target → {payload}")
                mqtt_client.publish(
                    TOPIC_DESIRED_POSITION, json.dumps(payload)
                )

        if not frame_queue.full():
            frame_queue.put(frame)

# =========================================================
# TKINTER UPDATE LOOP
# =========================================================
def update_ui():
    if not frame_queue.empty():
        frame = frame_queue.get()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = ImageTk.PhotoImage(
            Image.fromarray(frame).resize((480, 360))
        )
        camera_label.imgtk = img
        camera_label.config(image=img)

    root.after(30, update_ui)

# =========================================================
# CLEANUP
# =========================================================
def on_close():
    print("[SYSTEM] Shutting down...")
    stop_event.set()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    cap.release()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

threading.Thread(target=camera_thread, daemon=True).start()
update_ui()
root.mainloop()
