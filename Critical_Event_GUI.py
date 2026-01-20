import tkinter as tk
import math
import threading
import time
import socket
import struct
try:
    import pygame
except Exception:
    pygame = None

# =========================================================
# UDP SETUP
# =========================================================
UDP_IP = "138.38.229.217"   # Update as needed
UDP_PORT = 25000

UDP_RATE_HZ = 2.0  # 2 messages per second

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Shared state
udp_enabled = False
left_cmd = 0.0
right_cmd = 0.0

# =========================================================
# JOYSTICK CONFIG (Nintendo controller)
# =========================================================
JOYSTICK_DEADZONE = 0.08
AXIS_FORWARD = 1  # typically left stick Y
AXIS_TURN = 0     # typically left stick X
INVERT_FORWARD = True  # push up = positive


def udp_thread():
    global left_cmd, right_cmd, udp_enabled

    interval = 1.0 / UDP_RATE_HZ

    while True:
        if udp_enabled:
            # Pack two float32 values into 8 bytes
            msg = struct.pack("ff", float(left_cmd), float(right_cmd))
            udp_sock.sendto(msg, (UDP_IP, UDP_PORT))
            print(f"Sent UDP (binary floats): L={left_cmd:.3f}, R={right_cmd:.3f}")

        time.sleep(interval)


def _apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    return value


def joystick_thread():
    global left_cmd, right_cmd, udp_enabled

    if pygame is None:
        print("[JOYSTICK] pygame not available. Install pygame to use a real controller.")
        return

    try:
        pygame.init()
        pygame.joystick.init()
    except Exception as e:
        print(f"[JOYSTICK] init failed: {e}")
        return

    js = None

    def _connect_first_joystick():
        nonlocal js
        try:
            if pygame.joystick.get_count() > 0:
                js = pygame.joystick.Joystick(0)
                js.init()
                print(f"[JOYSTICK] connected: {js.get_name()}")
        except Exception as e:
            print(f"[JOYSTICK] connect failed: {e}")
            js = None

    _connect_first_joystick()

    while True:
        try:
            pygame.event.pump()
        except Exception:
            pass

        if js is None or not js.get_init():
            _connect_first_joystick()

        if js is None:
            left_cmd = 0.0
            right_cmd = 0.0
            time.sleep(0.1)
            continue

        try:
            forward = float(js.get_axis(AXIS_FORWARD))
            turn = float(js.get_axis(AXIS_TURN))
        except Exception:
            forward = 0.0
            turn = 0.0

        if INVERT_FORWARD:
            forward = -forward

        forward = _apply_deadzone(forward, JOYSTICK_DEADZONE)
        turn = _apply_deadzone(turn, JOYSTICK_DEADZONE)

        # Expo shaping
        EXPO_FORWARD = 1.5
        EXPO_TURN = 2.0

        forward = math.copysign(abs(forward) ** EXPO_FORWARD, forward)
        turn = math.copysign(abs(turn) ** EXPO_TURN, turn)

        # Differential drive mix
        left = forward + turn
        right = forward - turn

        # Normalize
        max_mag = max(abs(left), abs(right), 1.0)
        left /= max_mag
        right /= max_mag

        if udp_enabled:
            left_cmd = left
            right_cmd = right
        else:
            left_cmd = 0.0
            right_cmd = 0.0

        time.sleep(0.01)


# =========================================================
# JOYSTICK UI
# =========================================================
class JoystickUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Differential Drive Joystick")

        self.is_on = False

        self.toggle_btn = tk.Button(root, text="OFF", width=10, command=self.toggle)
        self.toggle_btn.pack(pady=10)

        self.canvas_size = 200
        self.radius = 80
        self.center = self.canvas_size // 2

        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="lightgrey")
        self.canvas.pack()

        #draw arrow triangles inside d-pad circle
        c = self.canvas_size // 2
        arrow_height = 20
        arrow_width = 12
        margin = 15

        # Up arrow
        tip_y_up = c - self.radius + margin
        base_y_up = tip_y_up + arrow_height
        up_points = (c, tip_y_up, c - arrow_width, base_y_up, c + arrow_width, base_y_up)
        self.canvas.create_polygon(up_points, fill="black", outline = "")

        # Down arrow
        tip_y_down = c + self.radius - margin
        base_y_down = tip_y_down - arrow_height
        down_points = (c, tip_y_down, c - arrow_width, base_y_down, c + arrow_width, base_y_down)
        self.canvas.create_polygon(down_points, fill="black", outline = "")

        # Left arrow
        tip_x_left = c - self.radius + margin
        base_x_left = tip_x_left + arrow_height
        left_points = (tip_x_left, c, base_x_left, c - arrow_width, base_x_left, c + arrow_width)
        self.canvas.create_polygon(left_points, fill="black", outline = "")

        # Right arrow   
        tip_x_right = c + self.radius - margin
        base_x_right = tip_x_right - arrow_height
        right_points = (tip_x_right, c, base_x_right, c - arrow_width, base_x_right, c + arrow_width)   
        self.canvas.create_polygon(right_points, fill="black", outline = "")
    

        self.canvas.create_oval(
            self.center - self.radius,
            self.center - self.radius,
            self.center + self.radius,
            self.center + self.radius,
            outline="black"
        )

        # GUI joystick disabled; real controller used instead

    def toggle(self):
        global udp_enabled, left_cmd, right_cmd

        self.is_on = not self.is_on
        self.toggle_btn.config(text="ON" if self.is_on else "OFF")

        if self.is_on:
            udp_enabled = True
        else:
            # Stop robot and stop UDP
            left_cmd = 0.0
            right_cmd = 0.0
            udp_enabled = False

            # Send one final stop packet
            msg = struct.pack("ff", 0.0, 0.0)
            udp_sock.sendto(msg, (UDP_IP, UDP_PORT))
            print("Sent UDP stop packet (binary floats)")

    def expo(self, x, e):
        return math.copysign(abs(x) ** e, x)

    def move(self, event):
        global left_cmd, right_cmd

        if not self.is_on:
            return

        dx = event.x - self.center
        dy = event.y - self.center

        distance = math.sqrt(dx**2 + dy**2)
        if distance > self.radius:
            scale = self.radius / distance
            dx *= scale
            dy *= scale

        # Normalize joystick
        forward = -dy / self.radius   # up = positive
        turn = dx / self.radius       # right = positive

        forward = max(-1.0, min(1.0, forward))
        turn = max(-1.0, min(1.0, turn))

        # Expo shaping
        EXPO_FORWARD = 1.5
        EXPO_TURN = 2.0

        forward = self.expo(forward, EXPO_FORWARD)
        turn = self.expo(turn, EXPO_TURN)

        # Differential drive mix
        left = forward + turn
        right = forward - turn

        # Normalize
        max_mag = max(abs(left), abs(right), 1.0)
        left /= max_mag
        right /= max_mag

        left_cmd = left
        right_cmd = right

    def release(self, event):
        global left_cmd, right_cmd

        if self.is_on:
            left_cmd = 0.0
            right_cmd = 0.0


# =========================================================
# MAIN
# =========================================================
if __name__ == "__main__":
    # Start UDP sender thread
    t = threading.Thread(target=udp_thread, daemon=True)
    t.start()

    # Start joystick reader thread
    jt = threading.Thread(target=joystick_thread, daemon=True)
    jt.start()

    root = tk.Tk()
    app = JoystickUI(root)
    root.mainloop()
