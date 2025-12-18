import tkinter as tk
import math
import threading
import queue
import time
import paho.mqtt.client as mqtt

#MQTT broker details
BROKER = "fesv-mqtt.bath.ac.uk"
PORT = 31415
USERNAME = "student"
PASSWORD = "HousekeepingGlintsStreetwise"

TOPIC_SPEED = "CottonCandyGrapes/CriticalEventRobot/Speed"
TOPIC_DIRECTION = "CottonCandyGrapes/CriticalEventRobot/Direction"

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)

def mqtt_thread(q: queue.Queue):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.username_pw_set(USERNAME, password=PASSWORD)
    client.connect(BROKER, PORT, 60)
    client.loop_start()

    try:
        while True:
            try:
                direction, speed = q.get(timeout=0.5)
                # Publish speed as a number
                client.publish(TOPIC_SPEED, speed)
                # Publish direction as a string
                client.publish(TOPIC_DIRECTION, direction)
                print(f"Published: {direction}, {speed}")
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        client.loop_stop()
        client.disconnect()

# Joystick UI (pushes to queue) ---
class JoystickUI:
    def __init__(self, root, q):
        self.root = root
        self.q = q
        self.root.title("D-Pad")
        self.is_on = False

        self.toggle_btn = tk.Button(root, text="OFF", width=10, command=self.toggle)
        self.toggle_btn.pack(pady=10)

        self.canvas_size = 200
        self.radius = 80
        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="lightgrey")
        self.canvas.pack()

        # draw simple arrow triangles inside the D-pad circle
        c = self.canvas_size // 2
        arrow_h = 20
        arrow_w = 12
        margin = 15

        # Up arrow (triangle pointing up)
        tip_y_up = c - self.radius + margin
        base_y_up = tip_y_up + arrow_h
        up_points = (c, tip_y_up, c - arrow_w, base_y_up, c + arrow_w, base_y_up)
        self.canvas.create_polygon(up_points, fill="black", outline="")

        # Down arrow (triangle pointing down)
        tip_y_down = c + self.radius - margin
        base_y_down = tip_y_down - arrow_h
        down_points = (c, tip_y_down, c + arrow_w, base_y_down, c - arrow_w, base_y_down)
        self.canvas.create_polygon(down_points, fill="black", outline="")

        # Left arrow (triangle pointing left)
        tip_x_left = c - self.radius + margin
        base_x_left = tip_x_left + arrow_h
        left_points = (tip_x_left, c, base_x_left, c - arrow_w, base_x_left, c + arrow_w)
        self.canvas.create_polygon(left_points, fill="black", outline="")

        # Right arrow (triangle pointing right)
        tip_x_right = c + self.radius - margin
        base_x_right = tip_x_right - arrow_h
        right_points = (tip_x_right, c, base_x_right, c + arrow_w, base_x_right, c - arrow_w)
        self.canvas.create_polygon(right_points, fill="black", outline="")

        self.center = self.canvas_size // 2
        self.canvas.create_oval(self.center - self.radius, self.center - self.radius,
                                self.center + self.radius, self.center + self.radius,
                                outline="black")

        self.canvas.bind("<B1-Motion>", self.move)
        self.canvas.bind("<ButtonRelease-1>", self.release)

    def toggle(self):
        self.is_on = not self.is_on
        self.toggle_btn.config(text="ON" if self.is_on else "OFF")

    def move(self, event):
        if not self.is_on:
            return
        dx = event.x - self.center
        dy = event.y - self.center
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.degrees(math.atan2(-dy, dx))
        speed = min(distance / self.radius, 1.0) *100
        direction = self.get_direction(angle)
        self.q.put((direction, round(speed, 1)))

    def release(self, event):
        if self.is_on:
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
        elif 157.5 <= angle or angle < -157.5:
            return "LEFT"
        elif -157.5 <= angle < -112.5:
            return "DOWN-LEFT"
        elif -112.5 <= angle < -67.5:
            return "DOWN"
        elif -67.5 <= angle < -22.5:
            return "DOWN-RIGHT"
        else:
            return "CENTER"

if __name__ == "__main__":
    q = queue.Queue()
    t = threading.Thread(target=mqtt_thread, args=(q,), daemon=True)
    t.start()
    root = tk.Tk()
    app = JoystickUI(root, q)
    root.mainloop()
