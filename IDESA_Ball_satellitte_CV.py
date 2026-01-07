import tkinter as tk
from PIL import Image, ImageTk
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import queue

# --------------------
# Camera / ArUco Setup
# --------------------
camera_calibration = np.load('Sample_Calibration.npz')
CM = camera_calibration['CM']
dist_coef = camera_calibration['dist_coef']

marker_size = 40
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

cap = cv2.VideoCapture(0)

# --------------------
# Tkinter Window
# --------------------
root = tk.Tk()
root.title("IDESA Ball Rover UI")
root.geometry("1000x600")
root.configure(bg="black")

left_frame = tk.Frame(root, bg="black")
right_frame = tk.Frame(root, bg="black")

left_frame.pack(side="left", fill="both", expand=True)
right_frame.pack(side="right", fill="both", expand=True)

# --------------------
# LEFT SIDE
# --------------------
tk.Button(
    left_frame,
    text="Find Minerals",
    bg="#222",
    fg="white",
    font=("Arial", 14)
).pack(pady=10)

camera_label = tk.Label(left_frame, bg="black")
camera_label.pack(pady=10)

# --------------------
# RIGHT SIDE
# --------------------
tk.Label(
    right_frame,
    text="Manual Override",
    bg="black",
    fg="white",
    font=("Arial", 16)
).pack(pady=20)

manual_override = False

# --------------------
# Joystick UI
# --------------------
class JoystickUI:
    def __init__(self, parent, q):
        self.q = q
        self.is_on = False

        self.canvas_size = 200
        self.radius = 80

        self.canvas = tk.Canvas(
            parent,
            width=self.canvas_size,
            height=self.canvas_size,
            bg="lightgrey",
            highlightthickness=0
        )
        self.canvas.pack(pady=10)

        c = self.canvas_size // 2
        arrow_h = 20
        arrow_w = 12
        margin = 15

        # Arrows
        self.canvas.create_polygon(
            (c, c - self.radius + margin,
             c - arrow_w, c - self.radius + margin + arrow_h,
             c + arrow_w, c - self.radius + margin + arrow_h),
            fill="black"
        )
        self.canvas.create_polygon(
            (c, c + self.radius - margin,
             c + arrow_w, c + self.radius - margin - arrow_h,
             c - arrow_w, c + self.radius - margin - arrow_h),
            fill="black"
        )
        self.canvas.create_polygon(
            (c - self.radius + margin, c,
             c - self.radius + margin + arrow_h, c - arrow_w,
             c - self.radius + margin + arrow_h, c + arrow_w),
            fill="black"
        )
        self.canvas.create_polygon(
            (c + self.radius - margin, c,
             c + self.radius - margin - arrow_h, c + arrow_w,
             c + self.radius - margin - arrow_h, c - arrow_w),
            fill="black"
        )

        self.center = c
        self.canvas.create_oval(
            c - self.radius, c - self.radius,
            c + self.radius, c + self.radius,
            outline="black"
        )

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
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.degrees(math.atan2(-dy, dx))
        speed = min(distance / self.radius, 1.0) * 100

        direction = self.get_direction(angle)
        speed = round(speed, 1)

        # OUTPUT TO TERMINAL
        print(f"JOYSTICK → DIRECTION={direction} | SPEED={speed}")

        self.q.put((direction, speed))

    def release(self, event):
        if self.is_on:
            print("JOYSTICK → CENTER | SPEED=0")
            self.q.put(("CENTER", 0))

    def get_direction(self, angle):
        if -22.5 <= angle < 22.5:
            return "RIGHT"
        elif 22.5 <= angle < 67.5:
            return "UP-RIGHT"
        elif 67.5 <= angle < 112.5:
            return "UP"
        elif 112.5 <= angle < 157.5:
            return "UP-LEFT"
        elif angle >= 157.5 or angle < -157.5:
            return "LEFT"
        elif -157.5 <= angle < -112.5:
            return "DOWN-LEFT"
        elif -112.5 <= angle < -67.5:
            return "DOWN"
        elif -67.5 <= angle < -22.5:
            return "DOWN-RIGHT"
        else:
            return "CENTER"

control_queue = queue.Queue()
joystick = JoystickUI(right_frame, control_queue)

def toggle_manual():
    global manual_override
    manual_override = not manual_override
    toggle_button.config(text="ON" if manual_override else "OFF")
    joystick.set_enabled(manual_override)

toggle_button = tk.Button(
    right_frame,
    text="OFF",
    bg="#222",
    fg="white",
    font=("Arial", 14),
    command=toggle_manual
)
toggle_button.pack(pady=10)

# --------------------
# Camera Update Loop
# --------------------
def update_camera():
    ret, frame = cap.read()
    if not ret:
        root.after(30, update_camera)
        return

    h, w, _ = frame.shape
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if ids is not None:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, marker_size, CM, dist_coef
        )
        for rvec, tvec in zip(rvecs, tvecs):
            frame = cv2.drawFrameAxes(frame, CM, dist_coef, rvec, tvec, 100)

    # Grid overlay
    for i in range(1, 7):
        cv2.line(frame, (int(w*i/7), 0), (int(w*i/7), h), (200,200,200), 1)
    for i in range(1, 5):
        cv2.line(frame, (0, int(h*i/5)), (w, int(h*i/5)), (200,200,200), 1)

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(frame).resize((480, 360))
    imgtk = ImageTk.PhotoImage(img)

    camera_label.imgtk = imgtk
    camera_label.config(image=imgtk)

    root.after(30, update_camera)

# --------------------
# Cleanup
# --------------------
def on_close():
    cap.release()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

update_camera()
root.mainloop()