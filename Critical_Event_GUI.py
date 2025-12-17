#this file is the user interface thread to control the critical event robot
# by Caitlin Grainger-Spivey

#first we import the necessary libraries
import tkinter as tk
import math

class JoystickUI:
    def __init__(self, root):
        self.root = root
        self.root.title("D-Pad")

        self.is_on = False
        self.toggle_btn = tk.Button(root, text="OFF", width=10, command=self.toggle)
        self.toggle_btn.pack(pady=10)

        # Canvas for joystick
        self.canvas_size = 200
        self.radius = 90
        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="lightgrey")
        self.canvas.pack()

        # Draw joystick boundary
        self.center = self.canvas_size // 2
        self.canvas.create_oval(self.center - self.radius, self.center - self.radius,
                    self.center + self.radius, self.center + self.radius,
                    outline="black")

        # Draw directional arrows (up, down, left, right) inside the boundary
        arrow_dist = int(self.radius * 0.65)   # distance from center to arrow tip
        arrow_size = 12                        # half-width/height of arrow base

        # Up arrow (triangle pointing up)
        up_tip = (self.center, self.center - arrow_dist)
        up_left = (self.center - arrow_size, self.center - arrow_dist + arrow_size)
        up_right = (self.center + arrow_size, self.center - arrow_dist + arrow_size)
        self.canvas.create_polygon(up_tip, up_left, up_right, fill="black", outline="black")

        # Down arrow (triangle pointing down)
        down_tip = (self.center, self.center + arrow_dist)
        down_left = (self.center - arrow_size, self.center + arrow_dist - arrow_size)
        down_right = (self.center + arrow_size, self.center + arrow_dist - arrow_size)
        self.canvas.create_polygon(down_tip, down_left, down_right, fill="black", outline="black")

        # Left arrow (triangle pointing left)
        left_tip = (self.center - arrow_dist, self.center)
        left_top = (self.center - arrow_dist + arrow_size, self.center - arrow_size)
        left_bottom = (self.center - arrow_dist + arrow_size, self.center + arrow_size)
        self.canvas.create_polygon(left_tip, left_top, left_bottom, fill="black", outline="black")

        # Right arrow (triangle pointing right)
        right_tip = (self.center + arrow_dist, self.center)
        right_top = (self.center + arrow_dist - arrow_size, self.center - arrow_size)
        right_bottom = (self.center + arrow_dist - arrow_size, self.center + arrow_size)
        self.canvas.create_polygon(right_tip, right_top, right_bottom, fill="black", outline="black")

        # Bind mouse events
        self.canvas.bind("<B1-Motion>", self.move)   # drag
        self.canvas.bind("<ButtonRelease-1>", self.release)  # release

    def toggle(self):
        self.is_on = not self.is_on
        self.toggle_btn.config(text="ON" if self.is_on else "OFF")

    def move(self, event):
        if not self.is_on:
            return

        # Vector from center
        dx = event.x - self.center
        dy = event.y - self.center

        # Distance and angle
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.degrees(math.atan2(-dy, dx))  # atan2(y,x), invert y for screen coords

        # Normalize speed (0–100%)
        speed = min(distance / self.radius, 1.0) * 100

        # Direction string
        direction = self.get_direction(angle)

        print(f"Direction: {direction}, Speed: {speed:.1f}%")

    def release(self, event):
        if self.is_on:
            print("Joystick released: CENTER, Speed: 0%")

    def get_direction(self, angle):
        # Map angle to direction (8-way)
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
    root = tk.Tk()
    app = JoystickUI(root)
    root.mainloop()
