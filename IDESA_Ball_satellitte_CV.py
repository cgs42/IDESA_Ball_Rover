import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import cv2.aruco as aruco
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time
import math
import queue
import threading
import socket
import struct
import paho.mqtt.client as mqtt

# =========================================================
# CONFIG FLAGS
# =========================================================
CRISIS_EVENTS_ENABLED = True   # Set to False to disable all crisis event behaviour (ID 10/11/9 avoidance, detours, visuals)
DEBUG_ANGLES = True          # Set True to overlay angle/vector debug info on the camera feed
# UDP smoothing (exponential moving average)
UDP_SMOOTHING_ENABLED = True
# alpha: 0.0 -> no update (frozen), 1.0 -> passthrough (no smoothing). Typical 0.2-0.6
UDP_SMOOTHING_ALPHA = 0.4

# =========================================================
# CAMERA & ARUCO SETUP
# =========================================================
camera_calibration = np.load('workdir/Calibration.npz')
CM = camera_calibration['CM']
dist_coef = camera_calibration['dist_coef']

marker_sizes = { 145: 100, 1: 75, 2: 75, 3: 75, 4: 75, 5: 75, 6: 75, 9: 75, 10: 75, 11: 75}  # aruco codes sizes in mm
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters()

# Attempt to open preferred camera index (1), otherwise try common indices
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    found = False
    for i in range(0, 5):
        try:
            tmp = cv2.VideoCapture(i)
            if tmp.isOpened():
                cap = tmp
                CAMERA_INDEX = i
                print(f"[CAM] Auto-selected camera index {i}")
                found = True
                break
            else:
                try:
                    tmp.release()
                except Exception:
                    pass
        except Exception:
            pass
    if not found:
        print("[CAM] Warning: no camera could be opened. Check camera index or permissions.")
last_print_time = 0.0
# store last detection data so UI can redraw overlays after filtering
last_marker_poses = {}
last_corners = None
last_ids = None

# =========================================================
# COLOUR DETECTION SETUP (HSV RANGES)
# =========================================================

# Orange crisis event
lower_orange = np.array([3, 0, 188])
upper_orange = np.array([39, 33, 255])

# Red crisis event (two ranges because hue wraps) # green coloured card
lower_red1 = np.array([88, 80, 80])
upper_red1 = np.array([100, 255, 255])
lower_red2 = np.array([88, 80, 80])
upper_red2 = np.array([100, 255, 255])

# Green crisis event
lower_green = np.array([103, 80, 80])
upper_green = np.array([108, 255, 255])

def find_blob_center(mask, min_area=300):
    """Return (cx, cy) of largest blob in mask, or None."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)

#Colour blob detection global variables
orange_blob = None
red_blob = None
green_blob = None
# Orange crisis visibility (True while blob is on screen)
orange_visible = False

# =========================================================
# UDP SETUP (AUTONOMOUS + JOYSTICK SHARE SAME SOCKET)
# =========================================================
UDP_IP = "138.38.226.46"  # amy's IP = "172.26.198.126", Caitlin's IP = "172.26.156.13" Dylan's IP = 172.26.73.75 little pi IP = "138.38.226.46" Big pi IP = "138.38.229.217"
UDP_PORT = 25000
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

LOCAL_BIND_IP = None  # set to a specific local IP to force outgoing interface, or None to auto-detect

UDP_RATE_HZ = 10.0  # messages per second for both autonomous and joystick (was 3.0)
last_udp_time = 0.0
UDP_DEBUG = False  # Set True to print detailed UDP send diagnostics for debugging
last_bound_local = None  # cache the last local IP we bound the udp socket to
# EMA state for autonomous sends (distance scalar, angle as vector cos/sin to avoid wrap)
ema_udp_distance = None
ema_udp_cos = None
ema_udp_sin = None

def ensure_bound_for_target(target_ip):
    """Ensure `udp_sock` is bound to the local interface that routes to target_ip.
    If `LOCAL_BIND_IP` is set, prefer that. Otherwise detect the outgoing local IP.
    """
    global udp_sock
    global last_bound_local
    try:
        if LOCAL_BIND_IP:
            desired_local = LOCAL_BIND_IP
        else:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.connect((target_ip, UDP_PORT))
                desired_local = s.getsockname()[0]
            finally:
                s.close()
        # If we've already bound to the desired local IP, skip detection and rebind
        if last_bound_local == desired_local:
            return

        try:
            cur_local = udp_sock.getsockname()[0]
        except Exception:
            cur_local = None

        if cur_local != desired_local:
            # create a new socket bound to desired_local
            new_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                new_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                new_sock.bind((desired_local, 0))
                try:
                    udp_sock.close()
                except Exception:
                    pass
                udp_sock = new_sock
                last_bound_local = desired_local
                if UDP_DEBUG:
                    print(f"[UDP BIND] bound sender socket to {desired_local}")
            except Exception as e:
                # binding failed; keep existing socket
                if UDP_DEBUG:
                    print("[UDP BIND ERROR]", e)
                try:
                    new_sock.close()
                except Exception:
                    pass
    except Exception:
        # detection failed; don't block sending
        return

def send_udp_packet(distance, angle):
    """Send two float32 values as 8 bytes over UDP."""
    try:
        # Force Python float (avoids numpy.float64 issues)
        d = float(distance)
        a = float(angle)

        # Pack into 8 bytes: <ff = little-endian float32, float32
        packet = struct.pack('<ff', d, a)

        try:
            ensure_bound_for_target(UDP_IP)
        except Exception:
            pass

        sent_bytes = udp_sock.sendto(packet, (UDP_IP, UDP_PORT))
        try:
            local_addr = udp_sock.getsockname()
        except Exception:
            local_addr = ('?', 0)

        # update last-sent values for UI display
        last_udp_distance = d
        last_udp_angle = a

        # Safely obtain current target mineral coordinates for logging
        try:
            tid = current_target_id
            if tid is None:
                target_coords = "None"
            else:
                pos = mineral_map.get(tid)
                if pos is None:
                    target_coords = f"id={tid}:None"
                else:
                    target_coords = f"id={tid}({float(pos[0]):.1f},{float(pos[1]):.1f},{float(pos[2]):.1f})"
        except Exception:
            target_coords = "Unknown"

        if UDP_DEBUG:
            print(f"[UDP SEND] from {local_addr} to {UDP_IP}:{UDP_PORT} bytes={sent_bytes} distance={d:.3f} angle={a:.3f} target={target_coords}")
        else:
            print(f"[UDP SEND] distance={d:.3f}, angle={a:.3f}, target={target_coords}")

        # Record plot data when plotting is enabled
        try:
            global plotting_enabled, plot_start_time, plot_times, plot_angles, plot_distances, MAX_PLOT_POINTS
            if plotting_enabled:
                if plot_start_time is None:
                    plot_start_time = time.time()
                t = time.time() - plot_start_time
                plot_times.append(t)
                plot_angles.append(a)
                plot_distances.append(d)
                # truncate to max points
                if len(plot_times) > MAX_PLOT_POINTS:
                    plot_times = plot_times[-MAX_PLOT_POINTS:]
                    plot_angles = plot_angles[-MAX_PLOT_POINTS:]
                    plot_distances = plot_distances[-MAX_PLOT_POINTS:]
        except Exception:
            pass

    except Exception as e:
        print("[UDP ERROR]", e)

# Startup diagnostic: try sending a single zero packet so we see any immediate errors
try:
    send_udp_packet(0.0, 0.0)
    print("[UDP TEST] startup packet attempted")
except Exception as e:
    print(f"[UDP TEST] startup send failed: {e}")

# =========================================================
# MQTT SETUP (MANUAL OVERRIDE MODE)
# =========================================================
Username = "student"
Password = "HousekeepingGlintsStreetwise"

Address = "fesv-mqtt.bath.ac.uk"
PORT = 31415
MainTopic = "CottonCandyGrapes/manualcontrol"  # used for manual override mode signal to publsih to
SubscribeTopic = "CottonCandyGrapes/MineralCount"  # used to subscribe and count number of actual minerals

mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(Username, Password)

try:
    mqtt_client.connect(Address, PORT, 60)
    mqtt_client.loop_start()
    print("[MQTT] Connected and loop started.")
except Exception as e:
    print(f"[MQTT] Connection error: {e}")

# MQTT mineral count handler: increment only when system active and in autonomous mode
def _update_mineral_label():
    try:
        status_mineral_label.config(text=f"Minerals: {mineral_count}")
    except Exception:
        pass

def _on_mineral_msg(client, userdata, msg):
    try:
        payload = msg.payload.decode('utf-8', errors='ignore').strip()
        if payload == '1':
            try:
                with mineral_count_lock:
                    # only count during active autonomous runs
                    if system_active and override_state == 0:
                        global mineral_count
                        mineral_count += 1
                try:
                    # schedule UI update on main thread
                    root.after(0, _update_mineral_label)
                except Exception:
                    pass
            except Exception:
                pass
    except Exception:
        pass

try:
    mqtt_client.message_callback_add(SubscribeTopic, _on_mineral_msg)
    mqtt_client.subscribe(SubscribeTopic)
    print(f"[MQTT] Subscribed to {SubscribeTopic} for mineral count")
except Exception:
    pass

# Shared override state: 0 = autonomous, 1 = manual override
override_state = 0  # Simulink will use this to switch modes

# =========================================================
# MINERAL STATE
# =========================================================
MINERAL_IDS = [1, 2, 3, 4, 5]  # aruco code IDs representing minerals
COLLECTION_RADIUS_MM = 35      # distance from robot centre to mineral centre to consider 'collected' (mm)
TARGET_PAUSE_SEC = 5.0         # time to wait at each target before moving to the next in seconds
DETECTION_TIME_SEC = 5.0       # time allowed for mineral detection phase to accomodate for detection flickering in seconds
BASE_ID = 6                    # aruco code ID representing home base
Robot_ID = 145                 # aruco code ID representing the robot
# orange_crisis_ID = 10          # ID 10 (appears as orange in GUI) - A localised weather event is occuring, continue to collect minerals but avoid the event and adapt the path around it. 
# red_crisis_ID = 8              # ID 11 (appears as red in GUI) - A severe weather event is occuring, stop all mineral collection and return to base. 
# green_crisis_ID = 9            # ID 9 (appears as green in GUI) - An indentifiable moving object has been spotted by the satellite, interrupt mineral collection and go and investigate the object.

WEATHER_AVOID_RADIUS_MM = 50   # radius around orange crisis event to avoid in mm (this already includes your desired clearance)

# set vectors and variables
mineral_map = {}               # id: (x, y, z) position in mm
remaining_minerals = []
path_order = []

# MQTT mineral counter
mineral_count = 0
mineral_count_lock = threading.Lock()

current_target_id = None
collected_mineral_id = None

system_active = False
waiting_for_next_target = False
target_reached_time = 0.0

detecting_minerals = False
detection_start_time = 0.0

base_added_to_path = False     # ensures base is added only once

weather_pos = None             # position of the orange crisis event (current)
prev_weather_pos = None        # previous position of the orange crisis event (for motion vector)

# Green investigation state
green_investigate = False
green_investigate_start = 0.0
green_target_pos = None
green_resume_target = None
green_last_seen = 0.0

# Temporary synthetic marker ID used to represent the green-event centre in `mineral_map`
GREEN_TEMP_ID = 9999

# How often to refresh the green synthetic target position (seconds)
GREEN_REFRESH_SEC = 1.0
green_last_update_time = 0.0

# Approx measured robot marker pixel side (updated when robot marker seen)
robot_pixel_size = None

# How long to keep following the green target after last seen (seconds)
GREEN_INVESTIGATE_TIMEOUT = 3.0

# Radius in mm for completing green investigation hold
GREEN_INVESTIGATE_RADIUS_MM = 300.0

# Track 5-second hold within green radius
green_investigate_hold = False
green_investigate_hold_start = None

# waypoint used for detour around weather event (None if going straight)
detour_waypoint = None         # (x_mm, y_mm)

# Request to return to base when base becomes visible
pending_return_to_base = False

#critical event UI state
critical_event_active = False
critical_event_colour = None
critical_event_last_toggle = 0.0
critical_event_visible = False
robot_pos = None
robot_yaw = None
robot_green_yaw = None

# Timestamp of last time we saw the robot marker (seconds since epoch).
# If the robot isn't seen for more than 2.0s we will send a zero fail-safe.
robot_last_seen_time = None


# =========================================================
# JOYSTICK SHARED STATE
# =========================================================
joystick_udp_enabled = False
left_cmd = 0.0
right_cmd = 0.0


# =========================================================
# critical event warning sound
# ========================================================
def play_critical_event_sound():
    try:
        import winsound
        winsound.Beep(1000, 200)  # Beep at 1000 Hz for 200 ms
    except:
        print("\a")  # Fallback to system beep
# =========================================================
# PATH PLANNING
# =========================================================
def compute_shortest_path(start_pos, mineral_items):
    path = []
    current = start_pos
    items = mineral_items.copy()

    while items:
        mid, pos = min(items,key=lambda x: math.hypot(current[0] - x[1][0], current[1] - x[1][1]))
        path.append(mid)
        current = pos
        items = [i for i in items if i[0] != mid]

    return path

# =========================================================
# CRISIS EVENT HANDLING
# =========================================================
def segment_intersects_circle(p, q, c, r):
    """
    Check if the segment from p to q intersects the circle centered at c with radius r.
    p, q, c are (x, y) in mm. r in mm.
    """
    px, py = p
    qx, qy = q
    cx, cy = c

    vx = qx - px
    vy = qy - py
    wx = cx - px
    wy = cy - py

    seg_len2 = vx*vx + vy*vy
    if seg_len2 == 0:
        # Degenerate segment, treat as a point
        dist = math.hypot(px - cx, py - cy)
        return dist < r

    t = (wx*vx + wy*vy) / seg_len2
    t = max(0.0, min(1.0, t))

    closest_x = px + t*vx
    closest_y = py + t*vy

    dist = math.hypot(closest_x - cx, closest_y - cy)
    return dist < r

def compute_tangent_waypoint(p, q, c, r, prev_c=None):
   
    #Compute a tangent waypoint around a circle centered at c (radius r),for a path from p (robot) to q (target). p, q, c are (x, y) mm, r in mm.
    # prev_c is previous circle center for motion direction (can be None). Returns (wx, wy) mm or None if we shouldn't detour.
    px, py = p
    qx, qy = q
    cx, cy = c

    # If the robot is inside the circle, detour is undefined
    if math.hypot(px - cx, py - cy) <= r + 1e-6:
        return None

    # Only detour if straight line would intersect/come too close to the circle
    if not segment_intersects_circle(p, q, c, r):
        return None

    # Circle-centered coordinates for tangent construction
    sx = px - cx
    sy = py - cy
    d = math.hypot(sx, sy)
    if d <= r:
        return None

    base_angle = math.atan2(sy, sx)
    delta = math.acos(r / d)

    theta1 = base_angle + delta
    theta2 = base_angle - delta

    t1x = cx + r * math.cos(theta1)
    t1y = cy + r * math.sin(theta1)
    t2x = cx + r * math.cos(theta2)
    t2y = cy + r * math.sin(theta2)

    # No motion history: pick the shorter total path
    if prev_c is None:
        d1 = math.hypot(px - t1x, py - t1y) + math.hypot(t1x - qx, t1y - qy)
        d2 = math.hypot(px - t2x, py - t2y) + math.hypot(t2x - qx, t2y - qy)
        return (t1x, t1y) if d1 <= d2 else (t2x, t2y)

    prev_cx, prev_cy = prev_c
    mvx = cx - prev_cx
    mvy = cy - prev_cy
    mv_norm = math.hypot(mvx, mvy)

    if mv_norm < 1e-6:
        # Hazard not really moving; fallback to shorter path
        d1 = math.hypot(px - t1x, py - t1y) + math.hypot(t1x - qx, t1y - qy)
        d2 = math.hypot(px - t2x, py - t2y) + math.hypot(t2x - qx, t2y - qy)
        return (t1x, t1y) if d1 <= d2 else (t2x, t2y)

    # Direction vectors from circle to tangent points
    d1x = t1x - cx
    d1y = t1y - cy
    d2x = t2x - cx
    d2y = t2y - cy

    # Prefer the side more "behind" the hazard motion:
    dot1 = (d1x * mvx + d1y * mvy)
    dot2 = (d2x * mvx + d2y * mvy)

    # Smaller dot product → more opposite hazard motion → more in its wake
    if dot1 <= dot2:
        return (t1x, t1y)
    else:
        return (t2x, t2y)

# =========================================================
# SIGNED ANGLE + DISTANCE CALCULATION (GENERAL FOR ANY MARKER)
# =========================================================
def compute_signed_angle_and_distance(rvec106, tvec106, rvecX, tvecX):
    # Euclidean distance magnitude (mm)
    distance_mm = float(np.linalg.norm(tvecX - tvec106))

    R106, _ = cv2.Rodrigues(rvec106)
    y_axis_106 = R106[:, 1]
    z_axis_106 = R106[:, 2]

    vec = (tvecX - tvec106).astype(float)
    norm_vec = np.linalg.norm(vec)
    if norm_vec < 1e-6:
        return 0.0, 0.0

    # forward unit vector (marker Y axis)
    a = y_axis_106.astype(float)
    a_unit = a / np.linalg.norm(a)

    # signed distance = projection of target vector onto forward axis (mm)
    signed_distance_mm = float(np.dot((tvecX - tvec106).astype(float), a_unit))

    # compute signed angle using marker Z,a,b as before
    b = vec / norm_vec
    numerator = np.dot(z_axis_106, np.cross(a_unit, b))
    denominator = np.dot(a_unit, b)
    angle_rad = np.arctan2(numerator, denominator)
    angle_deg = float(np.degrees(angle_rad))

    return signed_distance_mm, angle_deg

#========================================================
# MINERAL FIND TOGGLE
#=======================================================
def toggle_mineral_find():
    global system_active, mineral_map, remaining_minerals, path_order
    global current_target_id, waiting_for_next_target
    global detecting_minerals, detection_start_time, base_added_to_path
    global weather_pos, prev_weather_pos, detour_waypoint
    global plotting_enabled, plot_times, plot_angles, plot_distances, plot_start_time
    global ema_udp_distance, ema_udp_cos, ema_udp_sin, last_udp_time
    global mineral_count, mineral_count_lock

    system_active = not system_active

    if system_active:
        # print("\n[SYSTEM] MINERAL FIND STARTED")
        # reset state for a fresh run
        mineral_map.clear()
        remaining_minerals.clear()
        path_order.clear()
        current_target_id = None
        waiting_for_next_target = False
        detecting_minerals = True
        detection_start_time = time.time()
        base_added_to_path = False
        weather_pos = None
        prev_weather_pos = None
        detour_waypoint = None
        # enable plotting and reset buffers
        plotting_enabled = True
        plot_start_time = time.time()
        plot_times.clear()
        plot_angles.clear()
        plot_distances.clear()
        button.config(text="End Mineral Find")
        # reset MQTT mineral counter for a fresh run
        try:
            with mineral_count_lock:
                mineral_count = 0
        except Exception:
            pass
        # reset UDP EMA state when starting a fresh autonomous run
        try:
            ema_udp_distance = None
            ema_udp_cos = None
            ema_udp_sin = None
        except Exception:
            pass
    else:
        # print("\n[SYSTEM] MINERAL FIND STOPPED")
        # Reset all mission state when toggled off
        button.config(text="Find Minerals")
        mineral_map.clear()
        remaining_minerals.clear()
        path_order.clear()
        current_target_id = None
        waiting_for_next_target = False
        detecting_minerals = False
        detour_waypoint = None
        base_added_to_path = False
        weather_pos = None
        prev_weather_pos = None
        # disable plotting and clear buffers
        plotting_enabled = False
        plot_start_time = None
        plot_times.clear()
        plot_angles.clear()
        plot_distances.clear()
        # reset EMA smoothing when stopping
        try:
            ema_udp_distance = None
            ema_udp_cos = None
            ema_udp_sin = None
        except Exception:
            pass
        # send immediate stop packet to actuators to ensure robot halts
        try:
            send_udp_packet(0.0, 0.0)
            last_udp_time = time.time()
        except Exception:
            pass

# =========================================================
# TKINTER UI
# =========================================================
root = tk.Tk()
root.state('zoomed')  # full screen
root.title("IDESA Ball Rover UI")
root.configure(bg="black")

# -----------------------------
# LEFT + RIGHT FRAMES
# -----------------------------
main_frame = tk.Frame(root, bg="black")
main_frame.pack(fill="both", expand=True)

left_frame = tk.Frame(main_frame, bg="black")
left_frame.pack(side="left", fill="both", expand=True)

right_frame = tk.Frame(main_frame, bg="black")
right_frame.pack(side="right", fill="y")

# -----------------------------
# BLUEPRINT BACKGROUND
# -----------------------------
bg_canvas = tk.Canvas(left_frame, bg="#0B1020", highlightthickness=0)
bg_canvas.place(relwidth=1, relheight=1)

for i in range(0, 2000, 40):
    bg_canvas.create_line(i, 0, i, 2000, fill="#1A2940")
for j in range(0, 2000, 40):
    bg_canvas.create_line(0, j, 2000, j, fill="#1A2940")

bg_canvas_right = tk.Canvas(right_frame, bg="#0B1020", highlightthickness=0)
bg_canvas_right.place(relwidth=1, relheight=1)

for i in range(0, 1000, 40):
    bg_canvas_right.create_line(i, 0, i, 1000, fill="#1A2940")
for j in range(0, 1000, 40):
    bg_canvas_right.create_line(0, j, 1000, j, fill="#1A2940")

# -----------------------------
# TOOLTIP CLASS
# -----------------------------
class ToolTip:
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tip = None
        widget.bind("<Enter>", self.show)
        widget.bind("<Leave>", self.hide)

    def show(self, event=None):
        if self.tip:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + 20
        self.tip = tk.Toplevel(self.widget)
        self.tip.wm_overrideredirect(True)
        self.tip.wm_geometry(f"+{x}+{y}")
        label = tk.Label(
            self.tip,
            text=self.text,
            bg="#1F2933",
            fg="white",
            relief="solid",
            borderwidth=1,
            font=("Consolas", 9)
        )
        label.pack(ipadx=4, ipady=2)

    def hide(self, event=None):
        if self.tip:
            self.tip.destroy()
            self.tip = None

# -----------------------------
# MINERAL FIND BUTTON
# -----------------------------
button = tk.Button(left_frame,text="Find Minerals",bg="#222",fg="white",font=("Arial", 14),command=toggle_mineral_find)
button.pack()

# -----------------------------
# CAMERA FEED LABEL
# -----------------------------
# Display size for camera feed (pixels)
DISPLAY_W = 900
DISPLAY_H = 610

camera_label = tk.Label(left_frame, bg="black", width=DISPLAY_W, height=DISPLAY_H)
camera_label.pack(padx=(25,50))

# -----------------------------
# MINI-MAP CANVAS
# -----------------------------
minimap_sizex = 340
# make minimap slightly shorter than camera so status bar remains visible
minimap_sizey = int(DISPLAY_H * 0.78)
minimap = tk.Canvas(
    left_frame,width=minimap_sizex,height=minimap_sizey, bg="#0B1020",highlightthickness=1,highlightbackground="#2B4A6A")
map_plot_frame = tk.Frame(left_frame, bg="#0B1020")
map_plot_frame.pack(pady=10)

minimap.pack(in_=map_plot_frame, side="left")
button.lift()

# -----------------------------
# TOOLTIP ATTACHMENTS
# -----------------------------
ToolTip(camera_label, "Live camera feed with ArUco marker overlays.")
ToolTip(minimap, "Mini-map showing robot, minerals, and crisis events.")
ToolTip(button, "Start/stop the autonomous mineral finding system.")

camera_label.lift()

# -----------------------------
# LIVE PLOT (angle & distance vs time)
# -----------------------------
# Use a single axis with a twin y-axis; set background to match minimap
plot_fig = Figure(figsize=(4,4.5), dpi=100, facecolor="#0B1020")
plot_ax = plot_fig.add_subplot(111)
dist_ax = plot_ax.twinx()
plot_ax.set_facecolor("#0B1020")
dist_ax.set_facecolor("none")
plot_canvas = None
plot_times = []
plot_angles = []
plot_distances = []
plot_start_time = None
plotting_enabled = False
MAX_PLOT_POINTS = 400
# Plot redraw throttle (seconds)
PLOT_UPDATE_INTERVAL = 0.1
plot_last_draw_time = 0.0
# Minimap redraw throttle (seconds)
MINIMAP_UPDATE_INTERVAL = 0.1
minimap_last_draw_time = 0.0

def _create_plot_canvas(parent):
    global plot_canvas
    canvas = FigureCanvasTkAgg(plot_fig, master=parent)
    widget = canvas.get_tk_widget()
    widget.pack(side="left", padx=(10,0))
    try:
        widget.config(height=minimap_sizey)
    except Exception:
        pass
    plot_canvas = canvas
    return canvas

# create the plot canvas inside the map_plot_frame next to minimap
try:
    _create_plot_canvas(map_plot_frame)
except Exception:
    plot_canvas = None


# -----------------------------
# STATUS BAR (BOTTOM)
# -----------------------------
# Status panel moved to right frame (vertical column of labels)
status_frame = tk.Frame(right_frame, bg="#101828")
status_frame.pack(padx=10, pady=10, anchor="n")

status_mode_label = tk.Label(status_frame,text="Mode: ---",anchor="w",bg="#101828",fg="#E0E6F0",font=("Consolas", 16))
status_mode_label.pack(fill='x', pady=(2,2))

status_target_label = tk.Label(status_frame,text="Target: ---",anchor="w",bg="#101828",fg="#E0E6F0",font=("Consolas", 16))
status_target_label.pack(fill='x', pady=(2,2))

status_robot_label = tk.Label(status_frame,text="Robot: (---, ---)",anchor="w",bg="#101828",fg="#E0E6F0",font=("Consolas", 16))
status_robot_label.pack(fill='x', pady=(2,2))

status_mineral_label = tk.Label(status_frame,text="Minerals: 0",anchor="w",bg="#101828",fg="#E0E6F0",font=("Consolas", 16))
status_mineral_label.pack(fill='x', pady=(2,2))

# extra single-line status (used for weather/alerts)
status_extra_label = tk.Label(status_frame,text="",anchor="w",bg="#101828",fg="#E0E6F0",font=("Consolas", 14))
status_extra_label.pack(fill='x', pady=(6,2))

# =========================================================
# THREADING
# =========================================================
frame_queue = queue.Queue(maxsize=1)
stop_event = threading.Event()

# =========================================================
# CAMERA THREAD (AUTONOMOUS NAVIGATION)
# =========================================================
def camera_thread():
    global last_udp_time, current_target_id, last_print_time
    global waiting_for_next_target, target_reached_time
    global collected_mineral_id, detecting_minerals
    global system_active, base_added_to_path
    global weather_pos, prev_weather_pos, detour_waypoint
    global override_state  # to suppress autonomous UDP when manual override is on
    global orange_blob, red_blob, green_blob, orange_visible
    global robot_pos, robot_yaw, robot_green_yaw, robot_pixel_size
    global robot_last_seen_time
    global pending_return_to_base
    global green_investigate, green_investigate_start, green_target_pos, green_resume_target, green_last_seen
    global green_last_update_time
    global green_investigate_hold, green_investigate_hold_start
    global ema_udp_distance, ema_udp_cos, ema_udp_sin
    while not stop_event.is_set():

        marker_poses = {}   # id → (rvec, tvec)

        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        # Keep original frame for detection, but apply UI filters immediately
        orig = frame
        try:
            vis_frame = apply_filters_frame(orig)
            if vis_frame is None:
                vis_frame = orig.copy()
            if len(vis_frame.shape) == 2:
                vis_frame = cv2.cvtColor(vis_frame, cv2.COLOR_GRAY2BGR)
        except Exception:
            vis_frame = orig.copy()
        # =========================================================
        # COLOUR-BASED CRISIS DETECTION
        # =========================================================
        # Use raw colours for blob detection so colour filters don't affect masks
        hsv = cv2.cvtColor(orig, cv2.COLOR_BGR2HSV)

        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        orange_blob = find_blob_center(mask_orange)
        red_blob = find_blob_center(mask_red) # green
        green_blob = find_blob_center(mask_green)
        orange_visible = orange_blob is not None

        # Convert pixel coords to "fake" mm coords (same as ArUco tvecs)
        # You can refine this later with calibration if needed
        if orange_blob is not None:
            prev_weather_pos = weather_pos
            # store crisis centre (pixel coords used consistently with other markers in this code)
            prev_weather_pos = weather_pos
            weather_pos = np.array([orange_blob[0], orange_blob[1], 0.0])
            # print("Orange blob detected at", orange_blob)
        else:
            # Clear weather position when orange crisis is no longer visible
            weather_pos = None

        if red_blob is not None:
            # print("Red blob detected at", red_blob)
            # severe weather → return to base
            if system_active:
                critical_event_active = True
                critical_event_colour = (0, 0, 255)  # Red
                play_critical_event_sound()
                # print("\n[SYSTEM] Severe weather (RED BLOB) detected! Returning to base.")
                # treat red crisis similar to orange for avoidance
                prev_weather_pos = weather_pos
                weather_pos = np.array([red_blob[0], red_blob[1], 0.0])
                if BASE_ID in mineral_map:
                    current_target_id = BASE_ID
                    waiting_for_next_target = False
                    detour_waypoint = None
                else:
                    pending_return_to_base = True
                    waiting_for_next_target = False
                    detour_waypoint = None

        # Green crisis handling (enter investigation only when close and not in cooldown)
        try:
            if not hasattr(camera_thread, "_green_cooldown_active"):
                camera_thread._green_cooldown_active = False
                camera_thread._green_absent_since = None
        except Exception:
            pass

        green_visible = green_blob is not None

        # Cooldown reset: require the blob to disappear for >= 1s
        try:
            if green_visible:
                camera_thread._green_absent_since = None
            else:
                if camera_thread._green_absent_since is None:
                    camera_thread._green_absent_since = time.time()
                if camera_thread._green_cooldown_active and (time.time() - camera_thread._green_absent_since) >= 1.0:
                    camera_thread._green_cooldown_active = False
        except Exception:
            pass

        # Compute green target world position when visible (used for entry and tracking)
        if green_visible:
            if system_active:
                critical_event_active = True
                critical_event_colour = (0, 255, 0)  # Green
                play_critical_event_sound()
            try:
                if robot_pos is not None and robot_pixel_size is not None:
                    mm_per_pixel = float(marker_sizes.get(Robot_ID, 67)) / float(robot_pixel_size)
                    h, w = orig.shape[:2]
                    cx, cy = w / 2.0, h / 2.0
                    dx_px = float(green_blob[0]) - cx
                    dy_px = float(green_blob[1]) - cy
                    dx_mm = dx_px * mm_per_pixel
                    dy_mm = dy_px * mm_per_pixel
                    green_target_pos = np.array([robot_pos[0] + dx_mm, robot_pos[1] + dy_mm, 0.0])
                    mineral_map[GREEN_TEMP_ID] = green_target_pos
                    green_last_seen = time.time()
            except Exception:
                pass

        # Enter green-investigation mode immediately when visible (unless in cooldown)
        try:
            if (
                system_active
                and green_visible
                and (not green_investigate)
                and (not getattr(camera_thread, "_green_cooldown_active", False))
            ):
                green_investigate = True
                green_investigate_start = time.time()
                # pause mineral collection and save resume target
                try:
                    if current_target_id != GREEN_TEMP_ID:
                        green_resume_target = current_target_id
                except Exception:
                    green_resume_target = None
                current_target_id = GREEN_TEMP_ID
                waiting_for_next_target = True
                # reset hold timer on entry
                green_investigate_hold = False
                green_investigate_hold_start = None
        except Exception:
            pass

        if not ret:
            continue

        gray = cv2.cvtColor(vis_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            # Per-marker pose estimation using correct physical size
            for i, mid in enumerate(ids.flatten()):
                size_mm = marker_sizes.get(int(mid), 67)  

                rvecs_single, tvecs_single, _ = aruco.estimatePoseSingleMarkers(
                    [corners[i]], size_mm, CM, dist_coef
                )

                rvec = rvecs_single[0][0]  # (3,) vector
                tvec = tvecs_single[0][0]  # (3,) in mm
                
                mid_int = int(mid)
                marker_poses[mid_int] = (rvec, tvec)

                R, _ = cv2.Rodrigues(rvec)

                # Robot detection
                if mid == Robot_ID:
                    robot_pos = tvec
                    # X-axis yaw (unused for UDP angle) kept for compatibility
                    robot_yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
                    # Green axis is marker Y axis; compute its angle in camera XY plane
                    try:
                        robot_green_yaw = math.degrees(math.atan2(R[1, 1], R[0, 1]))
                    except Exception:
                        robot_green_yaw = robot_yaw
                    # Update last-seen timestamp for robot marker
                    try:
                        robot_last_seen_time = time.time()
                    except Exception:
                        pass
                    # Compute approx pixel side length for robot marker (used for mm/pixel mapping)
                    try:
                        c = corners[i][0]
                        side1 = np.linalg.norm(c[0] - c[1])
                        side2 = np.linalg.norm(c[1] - c[2])
                        side3 = np.linalg.norm(c[2] - c[3])
                        side4 = np.linalg.norm(c[3] - c[0])
                        robot_pixel_size = float((side1 + side2 + side3 + side4) / 4.0)
                    except Exception:
                        pass

                # Base detection (always active)
                if mid == BASE_ID:
                    mineral_map[BASE_ID] = tvec
                    # If a return-to-base was requested earlier because base wasn't visible,
                    # set the current target now that we have the base position.
                    if pending_return_to_base:
                        current_target_id = BASE_ID
                        pending_return_to_base = False
                        waiting_for_next_target = False
                        detour_waypoint = None

                # Mineral detection (only during scan)
                if detecting_minerals and mid in MINERAL_IDS:
                    mineral_map.setdefault(mid, tvec)

                # Draw axes for this marker onto the filtered visualization frame
                vis_frame = cv2.drawFrameAxes(vis_frame, CM, dist_coef, rvecs_single[0], tvecs_single[0], 100)

            vis_frame = aruco.drawDetectedMarkers(vis_frame, corners, ids)
            # debug overlay: show detected marker ids
            if DEBUG_ANGLES:
                try:
                    ids_list = sorted(int(k) for k in marker_poses.keys())
                    ids_text = "Detected IDs: " + ",".join(str(x) for x in ids_list)
                    cv2.putText(vis_frame, ids_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,0), 2)
                except Exception:
                    pass

        # Finish detection phase
        if detecting_minerals and time.time() - detection_start_time >= DETECTION_TIME_SEC:
            detecting_minerals = False
            remaining_minerals[:] = [mid for mid in mineral_map.keys() if mid in MINERAL_IDS]
            # print("Mineral map", mineral_map)

            # Use robot pose as start if available; otherwise fall back to base position
            if robot_pos is not None:
                start_pos = robot_pos
            elif BASE_ID in mineral_map:
                start_pos = mineral_map[BASE_ID]
            else:
                start_pos = None

            if start_pos is not None:
                path_order[:] = compute_shortest_path(start_pos, [(mid, mineral_map[mid]) for mid in remaining_minerals])

                # If compute_shortest_path for any reason returned fewer items than
                # we detected (race or numeric types), ensure all detected minerals
                # are present in path_order. Preserve the computed ordering first,
                # then append any missing minerals.
                try:
                    missing = [mid for mid in remaining_minerals if mid not in path_order]
                    if missing:
                        path_order.extend(missing)
                except Exception:
                    pass

                if DEBUG_ANGLES:
                    try:
                        print(f"[PATH DEBUG] minerals={sorted(remaining_minerals)} path_order={path_order}")
                    except Exception:
                        pass

        # Refresh synthetic green target position at ~1Hz while investigating
        try:
            if green_investigate:
                now_g = time.time()
                if (now_g - green_last_update_time) >= GREEN_REFRESH_SEC:
                    # Update last seen if visible
                    if green_blob is not None:
                        green_last_seen = now_g

                    # Only update target position if we still have recent visibility
                    if (green_blob is not None) or ((now_g - green_last_seen) <= GREEN_INVESTIGATE_TIMEOUT):
                        try:
                            if green_blob is not None and robot_pos is not None and robot_pixel_size is not None:
                                mm_per_pixel = float(marker_sizes.get(Robot_ID, 67)) / float(robot_pixel_size)
                                h, w = orig.shape[:2]
                                cx, cy = w / 2.0, h / 2.0
                                dx_px = float(green_blob[0]) - cx
                                dy_px = float(green_blob[1]) - cy
                                dx_mm = dx_px * mm_per_pixel
                                dy_mm = dy_px * mm_per_pixel
                                mineral_map[GREEN_TEMP_ID] = np.array([robot_pos[0] + dx_mm, robot_pos[1] + dy_mm, 0.0])
                                green_target_pos = mineral_map[GREEN_TEMP_ID]
                            else:
                                # keep last synthetic position if any
                                green_target_pos = mineral_map.get(GREEN_TEMP_ID)
                        except Exception:
                            pass
                    green_last_update_time = now_g
        except Exception:
            pass

        # Once within green radius, follow for 5s then resume prior target
        try:
            if green_investigate and (robot_pos is not None) and (green_target_pos is not None):
                dist_to_green = math.hypot(robot_pos[0] - green_target_pos[0], robot_pos[1] - green_target_pos[1])
                if dist_to_green <= GREEN_INVESTIGATE_RADIUS_MM and not green_investigate_hold:
                    green_investigate_hold = True
                    green_investigate_hold_start = time.time()

                if green_investigate_hold and (time.time() - (green_investigate_hold_start or 0.0)) >= 5.0:
                    green_investigate = False
                    current_target_id = green_resume_target
                    green_resume_target = None
                    waiting_for_next_target = False
                    green_investigate_hold = False
                    green_investigate_hold_start = None
                    try:
                        camera_thread._green_cooldown_active = True
                    except Exception:
                        pass
        except Exception:
            pass

        # Select next target
        if system_active and not detecting_minerals and current_target_id is None and not waiting_for_next_target:
            if path_order:
                # If the next item is the base but we still have minerals remaining,
                # keep the base until the end by moving it to the back of the list.
                try:
                    if path_order[0] == BASE_ID and remaining_minerals:
                        path_order[:] = [p for p in path_order if p != BASE_ID] + ([BASE_ID] if BASE_ID in path_order else [])
                except Exception:
                    pass

                current_target_id = path_order[0]
                detour_waypoint = None  # reset detour when new target chosen
                # print("[NAV] Target:", current_target_id)

        # Collection logic
        if system_active and robot_pos is not None and current_target_id and not waiting_for_next_target:
            tx, ty, tz = mineral_map[current_target_id]
            if math.hypot(robot_pos[0] - tx, robot_pos[1] - ty) < COLLECTION_RADIUS_MM:
                collected_mineral_id = current_target_id

                # If base reached
                if current_target_id == BASE_ID:
                    # print("\n[SYSTEM] Return to base completed. Mission ended.")
                    # Ensure we send a zero command when we've returned to base
                    try:
                        send_udp_packet(0.0, 0.0)
                    except Exception:
                        pass
                    # reset EMA smoothing state after mission end
                    try:
                        ema_udp_distance = None
                        ema_udp_cos = None
                        ema_udp_sin = None
                    except Exception:
                        pass
                    last_udp_time = time.time()
                    system_active = False
                    button.config(text="Find Minerals")
                    current_target_id = None
                    waiting_for_next_target = False
                    detour_waypoint = None
                    continue

                # Normal mineral collection
                if current_target_id in remaining_minerals:
                    remaining_minerals.remove(current_target_id)
                if path_order and path_order[0] == current_target_id:
                    path_order.pop(0)

                # After last mineral, append base if visible
                if not path_order and BASE_ID in mineral_map and not base_added_to_path:
                    # print("\n[SYSTEM] All minerals collected. Returning to base.")
                    path_order.append(BASE_ID)
                    base_added_to_path = True

                current_target_id = None
                detour_waypoint = None
                waiting_for_next_target = True
                target_reached_time = time.time()
                # print("[COLLECT] Mineral reached")

        # Pause between targets
        if waiting_for_next_target and time.time() - target_reached_time >= TARGET_PAUSE_SEC:
            waiting_for_next_target = False
            collected_mineral_id = None

        # =========================================================
        # CRISIS DETOUR UPDATE (ID 10 TANGENT AVOIDANCE)
        # =========================================================
        if CRISIS_EVENTS_ENABLED and system_active and robot_pos is not None and current_target_id and not waiting_for_next_target:
            if weather_pos is not None:
                rx, ry, rz = robot_pos
                tx, ty, tz = mineral_map[current_target_id]
                detour_waypoint = compute_tangent_waypoint((rx, ry),(tx, ty),(weather_pos[0], weather_pos[1]),
                    WEATHER_AVOID_RADIUS_MM,
                    prev_c=(prev_weather_pos[0], prev_weather_pos[1]) if prev_weather_pos is not None else None)
            else:
                detour_waypoint = None

        # Autonomous UDP sending (disabled when manual override is active)
        # Also halt completely while orange crisis is visible on screen.
        can_send_auto = (system_active and robot_pos is not None and robot_yaw is not None and current_target_id is not None and override_state == 0)  # ONLY send autonomous commands in autonomous mode
        if can_send_auto:
            now = time.time()
            if now - last_udp_time >= 1.0 / UDP_RATE_HZ:
                # Hard stop while orange crisis is visible on screen
                if orange_visible:
                    try:
                        send_udp_packet(0.0, 0.0)
                    except Exception:
                        pass
                    # reset EMA state while stopped so it doesn't smear across resume
                    try:
                        ema_udp_distance = None
                        ema_udp_cos = None
                        ema_udp_sin = None
                    except Exception:
                        pass
                    last_udp_time = now
                    continue
                rx, ry, rz = robot_pos

                # Fail-safe: if robot marker hasn't been seen for >2s, send zeros
                try:
                    if robot_last_seen_time is None or (time.time() - robot_last_seen_time) > 2.0:
                        try:
                            send_udp_packet(0.0, 0.0)
                        except Exception:
                            pass
                        # reset EMA state on fail-safe zero sends to avoid stale smoothing
                        try:
                            ema_udp_distance = None
                            ema_udp_cos = None
                            ema_udp_sin = None
                        except Exception:
                            pass
                        last_udp_time = now
                        # skip the normal navigation send for this cycle
                        continue
                except Exception:
                    # if any error checking timestamp, fall back to normal behaviour
                    pass

                # Navigation target selection
                # If we're investigating a green blob, prioritise that location for 5s
                if green_investigate:
                    # Follow the live green target while it has been seen recently
                    try:
                        if green_target_pos is not None and (time.time() - green_last_seen) < GREEN_INVESTIGATE_TIMEOUT:
                            target_x, target_y = float(green_target_pos[0]), float(green_target_pos[1])
                        else:
                            # Green target lost/timeout: stop investigating and resume mission
                            green_investigate = False
                            if green_resume_target is not None:
                                current_target_id = green_resume_target
                                green_resume_target = None
                    except Exception:
                        green_investigate = False

                # If not investigating green (or after resuming), use normal target selection
                if not green_investigate:
                    # Navigation target: detour waypoint if enabled & active
                    if CRISIS_EVENTS_ENABLED and detour_waypoint is not None:
                        wx, wy = detour_waypoint
                        # If close enough to waypoint, clear it and go straight to marker next
                        if math.hypot(rx - wx, ry - wy) < WEATHER_AVOID_RADIUS_MM * 0.6:
                            detour_waypoint = None
                            tx, ty, tz = mineral_map.get(current_target_id, (0.0, 0.0, 0.0))
                            target_x, target_y = tx, ty
                        else:
                            target_x, target_y = wx, wy
                    else:
                        tx, ty, tz = mineral_map.get(current_target_id, (0.0, 0.0, 0.0))
                        target_x, target_y = tx, ty

                # Prevent selecting the base as the active navigation target while
                # there are still minerals to collect. Orange crises should only
                # create detour waypoints; they must not force an immediate return.
                try:
                    if current_target_id == BASE_ID and remaining_minerals and not pending_return_to_base and critical_event_colour != (0, 0, 255):
                        # pick the first non-base target in the planned path if available
                        for pid in path_order:
                            if pid != BASE_ID:
                                current_target_id = pid
                                tx, ty, tz = mineral_map.get(current_target_id, (0.0, 0.0, 0.0))
                                target_x, target_y = tx, ty
                                break
                except Exception:
                    pass

                dx = target_x - rx
                dy = target_y - ry

                # If the robot's ArUco marker is not seen in this frame, send fail-safe zeros
                if Robot_ID not in marker_poses:
                    try:
                        send_udp_packet(0.0, 0.0)
                    except Exception:
                        pass
                    try:
                        ema_udp_distance = None
                        ema_udp_cos = None
                        ema_udp_sin = None
                    except Exception:
                        pass
                    last_udp_time = now
                    continue

                # Compute signed relative angle using atan2(cross,dot) when possible
                relative_distance = math.hypot(dx, dy)
                relative_angle = None

                try:
                    # Prefer using live marker poses (gives most accurate signed angle)
                    if (Robot_ID in marker_poses) and (current_target_id in marker_poses):
                        rvec106, tvec106 = marker_poses[Robot_ID]
                        rvecX, tvecX = marker_poses[current_target_id]
                        # use helper which uses atan2(n·(a×b), a·b)
                        dist_mm, ang_deg = compute_signed_angle_and_distance(rvec106, tvec106, rvecX, tvecX)
                        relative_distance = dist_mm
                        relative_angle = float(ang_deg)

                        now = time.time()
                        if now - last_print_time >= 1.0:
                            print(f"Signed angle between marker {Robot_ID} Y-axis and marker {current_target_id}: {relative_angle:.2f}°")
                            last_print_time = now

                        if DEBUG_ANGLES:
                            try:
                                cv2.putText(vis_frame, f"ang{Robot_ID}->{current_target_id}={relative_angle:.1f}", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                            except Exception:
                                pass
                    # If only robot marker pose available, compute signed angle against target vector
                    elif Robot_ID in marker_poses:
                        rvec106, tvec106 = marker_poses[Robot_ID]
                        R106, _ = cv2.Rodrigues(rvec106)
                        a = R106[:, 1]
                        n = R106[:, 2]
                        b = np.array([dx, dy, 0.0], dtype=float)
                        b_norm = np.linalg.norm(b)
                        if b_norm > 1e-6:
                            b /= b_norm
                            a_unit = a / np.linalg.norm(a)
                            ig_angle_rad = np.arctan2(np.dot(n, np.cross(a_unit, b)), np.dot(a_unit, b))
                            relative_angle = np.degrees(ig_angle_rad)
                            # signed distance along robot forward axis
                            relative_distance = float(np.dot(np.array([dx, dy, 0.0], dtype=float), a_unit))
                            #relative_angle = float(ig_angle_deg_signed)
                            if DEBUG_ANGLES:
                                try:
                                    cv2.putText(vis_frame, f"ang{Robot_ID}->target={relative_angle:.1f}", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                                except Exception:
                                    pass
                        else:
                            # degenerate vector -> zero angle
                            relative_angle = 0.0
                    else:
                        # Fallback: use yaw-based atan2 difference (still uses atan2 to preserve quadrant)
                        print("no fix")
                except Exception:
                    # hard fallback to zero on any unexpected error
                    try:
                        target_angle = math.degrees(math.atan2(dy, dx))
                        base_yaw = robot_green_yaw if robot_green_yaw is not None else (robot_yaw if robot_yaw is not None else 0.0)
                        relative_angle = (target_angle - base_yaw + 180) % 360 - 180
                    except Exception:
                        relative_angle = 0.0

                # Skip sending if we couldn't compute a valid angle
                if relative_angle is None:
                    # update timestamp to avoid immediate retry flood
                    last_udp_time = now
                else:
                    # default distance value
                    send_distance = relative_distance

                    # Apply exponential moving average smoothing for autonomous UDP sends
                    try:
                        if UDP_SMOOTHING_ENABLED:
                            alpha = UDP_SMOOTHING_ALPHA
                        else:
                            alpha = 1.0

                        # distance EMA
                        if ema_udp_distance is None:
                            ema_udp_distance = float(send_distance)
                        else:
                            ema_udp_distance = float(alpha * float(send_distance) + (1.0 - alpha) * float(ema_udp_distance))

                        # angle EMA via vector (cos,sin) to handle wraparound
                        rads = math.radians(float(relative_angle))
                        c = math.cos(rads)
                        s = math.sin(rads)
                        if ema_udp_cos is None or ema_udp_sin is None:
                            ema_udp_cos = c
                            ema_udp_sin = s
                        else:
                            ema_udp_cos = alpha * c + (1.0 - alpha) * ema_udp_cos
                            ema_udp_sin = alpha * s + (1.0 - alpha) * ema_udp_sin

                        smoothed_angle = math.degrees(math.atan2(ema_udp_sin, ema_udp_cos))
                        smoothed_distance = float(ema_udp_distance)
                    except Exception:
                        # on any error fall back to raw values
                        smoothed_distance = float(send_distance)
                        smoothed_angle = float(relative_angle)

                    try:
                        send_udp_packet(smoothed_distance, smoothed_angle)
                    except Exception:
                        # suppressed UDP AUTO send error print
                        pass

                    last_udp_time = now
        else:
            # periodic debug (every 2s) to show why autonomous UDP isn't sent
            if not hasattr(camera_thread, '_udp_debug_last'):
                camera_thread._udp_debug_last = 0
            if time.time() - camera_thread._udp_debug_last > 2.0:
                reasons = []
                if not system_active:
                    reasons.append('system_inactive')
                if robot_pos is None:
                    reasons.append('robot_pos_none')
                if robot_yaw is None:
                    reasons.append('robot_yaw_none')
                if current_target_id is None:
                    reasons.append('no_target')
                if override_state != 0:
                    reasons.append(f'override_state={override_state}')
                # print(f"[UDP AUTO DEBUG] not sending: {', '.join(reasons) or 'unknown'}")
                camera_thread._udp_debug_last = time.time()

        if frame_queue.full():
            try:
                frame_queue.get_nowait()
            except Exception:
                pass
        try:
            frame_queue.put_nowait((vis_frame, robot_pos))
        except Exception:
            pass

# =========================================================
# JOYSTICK UDP THREAD (MANUAL MODE)
# =========================================================
def joystick_udp_thread():
    global left_cmd, right_cmd, joystick_udp_enabled

    interval = 1.0 / UDP_RATE_HZ

    while not stop_event.is_set():
        if joystick_udp_enabled:
            # Send joystick motor commands as percentage values (-100 to 100)
            try:
                left_pct = max(-1, min(1, float(left_cmd)))
                right_pct = max(-1, min(1, float(right_cmd)))
            except Exception:
                left_pct = 0.0
                right_pct = 0.0
            # Pack two float32 values into 8 bytes
            try:
                msg = struct.pack("ff", left_pct, right_pct)
                udp_sock.sendto(msg, (UDP_IP, UDP_PORT))
                print(f"[UDP MANUAL] left={left_pct:.3f}, right={right_pct:.3f}")
            except Exception:
                pass

        time.sleep(interval)

# =========================================================
# MQTT HEARTBEAT THREAD (OVERRIDE MODE HEARTBEAT)
# =========================================================
def mqtt_heartbeat_thread():
    global override_state

    # Publish heartbeat messages. When manual override is active (override_state==1)
    # publish a steady '1' at `active_interval` seconds so receivers see a sustained
    # signal instead of a single spike. When not in manual override, publish less
    # frequently (long_interval) if the system is active.
    long_interval = 10.0
    active_interval = 1.0

    while not stop_event.is_set():
        try:
            if override_state == 1:
                mqtt_client.publish(MainTopic, payload="1", qos=0, retain=False)
            elif system_active:
                mqtt_client.publish(MainTopic, payload=str(override_state), qos=0, retain=False)
        except Exception:
            pass

        # Sleep with responsiveness to override state changes
        if override_state == 1:
            time.sleep(active_interval)
        else:
            # Sleep in short increments so the thread can react quickly if override is toggled
            slept = 0.0
            step = 0.5
            while slept < long_interval and not stop_event.is_set() and override_state != 1:
                time.sleep(step)
                slept += step

# =========================================================
# JOYSTICK UI CLASS (RIGHT SIDE)
# =========================================================
class JoystickUI:
    def __init__(self, parent):
        global override_state, joystick_udp_enabled

        self.parent = parent
        self.is_on = False
        self.canvas_size = 200
        self.radius = 80
        self.center = self.canvas_size // 2

        self.toggle_btn = tk.Button(parent,text="Manual OFF",width=12,command=self.toggle,bg="#444",fg="white",font=("Arial", 12))
        self.canvas = tk.Canvas(parent,width=self.canvas_size,height=self.canvas_size,bg="#0B1020",highlightthickness=1,highlightbackground="#2B4A6A")

        self.toggle_btn.pack(padx=20, pady=(80, 10), anchor="n")
        self.canvas.pack(padx=150, pady=(80, 10), anchor="n")

        # -----------------------------
        # FILTER SLIDERS (copied/adapted from filter tests)
        # -----------------------------
        # Expose filter controls as module globals so update_ui can access them
        global filter_brightness, filter_contrast, filter_gamma, filter_blur
        global filter_clahe_clip, filter_clahe_grid, filter_mode_var

        filter_frame = tk.Frame(parent, bg="#0B1020")
        filter_frame.pack(padx=20, pady=(10,40), anchor="n")

        def add_slider(parent, text, from_, to_, row, default):
            lbl = tk.Label(parent, text=text, bg="#0B1020", fg="#E0E6F0")
            lbl.grid(row=row, column=0, sticky="w")
            s = tk.Scale(parent, from_=from_, to=to_, orient="horizontal", bg="#0B1020", fg="#E0E6F0", highlightthickness=0)
            s.set(default)
            s.grid(row=row, column=1)
            return s

        filter_brightness = add_slider(filter_frame, "Brightness", -100, 100, 0, 0)
        filter_contrast = add_slider(filter_frame, "Contrast", 0, 3, 1, 1)
        filter_gamma = add_slider(filter_frame, "Gamma", 1, 300, 2, 100)
        filter_blur = add_slider(filter_frame, "Blur (odd only)", 1, 21, 3, 1)
        filter_clahe_clip = add_slider(filter_frame, "CLAHE Clip Limit", 1, 10, 4, 2)
        filter_clahe_grid = add_slider(filter_frame, "CLAHE Grid Size", 2, 16, 5, 8)

        filter_mode_var = tk.StringVar()
        filter_mode_var.set("None")
        filter_modes = ["None", "Grayscale", "Adaptive Threshold", "Canny Edges", "CLAHE"]
        dropdown = ttk.Combobox(filter_frame, textvariable=filter_mode_var, values=filter_modes, width=18)
        dropdown.grid(row=6, column=1)
        tk.Label(filter_frame, text="Filter Mode", bg="#0B1020", fg="#E0E6F0").grid(row=6, column=0)

        # draw arrow triangles inside d-pad circle
        c = self.canvas_size // 2
        arrow_height = 20
        arrow_width = 12
        margin = 15

        # Up arrow
        tip_y_up = c - self.radius + margin
        base_y_up = tip_y_up + arrow_height
        up_points = (c, tip_y_up, c - arrow_width, base_y_up, c + arrow_width, base_y_up)
        self.canvas.create_polygon(up_points, fill="#E0E6F0", outline="#2B4A6A")

        # Down arrow
        tip_y_down = c + self.radius - margin
        base_y_down = tip_y_down - arrow_height
        down_points = (c, tip_y_down, c - arrow_width, base_y_down, c + arrow_width, base_y_down)
        self.canvas.create_polygon(down_points, fill="#E0E6F0", outline="#2B4A6A")

        # Left arrow
        tip_x_left = c - self.radius + margin
        base_x_left = tip_x_left + arrow_height
        left_points = (tip_x_left, c, base_x_left, c - arrow_width, base_x_left, c + arrow_width)
        self.canvas.create_polygon(left_points, fill="#E0E6F0", outline="#2B4A6A")

        # Right arrow
        tip_x_right = c + self.radius - margin
        base_x_right = tip_x_right - arrow_height
        right_points = (tip_x_right, c, base_x_right, c - arrow_width, base_x_right, c + arrow_width)
        self.canvas.create_polygon(right_points, fill="#E0E6F0", outline="#2B4A6A")

        # Circular boundary
        self.canvas.create_oval(
            self.center - self.radius,
            self.center - self.radius,
            self.center + self.radius,
            self.center + self.radius,
            outline="black"
        )

        self.canvas.bind("<B1-Motion>", self.move)
        self.canvas.bind("<ButtonRelease-1>", self.release)

    def toggle(self):
        global joystick_udp_enabled, left_cmd, right_cmd, override_state

        self.is_on = not self.is_on

        if self.is_on:
            # Enable joystick control and manual override
            joystick_udp_enabled = True
            override_state = 1
            self.toggle_btn.config(text="Manual ON", bg="#800")
            # print("[JOYSTICK] Manual override ENABLED")

            # Send immediate MQTT signal (1)
            try:
                mqtt_client.publish(MainTopic, payload="1", qos=0, retain=False)
                # print("[MQTT] Immediate override=1")
            except Exception:
                # suppressed immediate MQTT publish error print
                pass

        else:
            # Disable joystick control, reset commands
            joystick_udp_enabled = False
            left_cmd = 0.0
            right_cmd = 0.0
            override_state = 0
            self.toggle_btn.config(text="Manual OFF", bg="#444")
            # print("[JOYSTICK] Manual override DISABLED")

            # Send one final stop UDP packet
            try:
                msg = struct.pack("ff", 0.0, 0.0)
                udp_sock.sendto(msg, (UDP_IP, UDP_PORT))
            except Exception:
                # suppressed UDP joystick stop error print
                pass

            # Send immediate MQTT signal (0)
            try:
                mqtt_client.publish(MainTopic, payload="0", qos=0, retain=False)
                # print("[MQTT] Immediate override=0")
            except Exception:
                # suppressed immediate MQTT publish error print
                pass

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

        self.toggle_btn.lift()
        self.canvas.tkraise()



# =========================================================
# UI LOOP
# =========================================================
def apply_filters_frame(frame):
    """Apply UI filter sliders to the provided BGR `frame` (numpy array) and
    return the filtered frame (BGR or single-channel depending on mode)."""
    try:
        # Ensure globals exist (they are created when JoystickUI is instantiated)
        global filter_brightness, filter_contrast, filter_gamma, filter_blur
        global filter_clahe_clip, filter_clahe_grid, filter_mode_var

        # Work on a copy of the frame
        img = frame.astype(np.float32)

        # Brightness & contrast
        img = img * (filter_contrast.get() if filter_contrast else 1.0) + (filter_brightness.get() if filter_brightness else 0)
        img = np.clip(img, 0, 255).astype(np.uint8)

        # Gamma correction
        g = (filter_gamma.get() if filter_gamma else 100) / 100.0
        if g <= 0:
            g = 0.01
        img = np.power(img / 255.0, 1.0 / g)
        img = (img * 255).astype(np.uint8)

        # Blur
        k = filter_blur.get() if filter_blur else 1
        if k % 2 == 0:
            k += 1
        try:
            img = cv2.GaussianBlur(img, (k, k), 0)
        except Exception:
            pass

        mode = filter_mode_var.get() if filter_mode_var else "None"

        if mode == "Grayscale":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        elif mode == "Adaptive Threshold":
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

        elif mode == "Canny Edges":
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img = cv2.Canny(gray, 80, 150)

        elif mode == "CLAHE":
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=(filter_clahe_clip.get() if filter_clahe_clip else 2),
                                   tileGridSize=(filter_clahe_grid.get() if filter_clahe_grid else 8,
                                                 filter_clahe_grid.get() if filter_clahe_grid else 8))
            img = clahe.apply(gray)

        return img
    except Exception:
        return frame
def update_ui():
    global robot_pos
    try:
        if not frame_queue.empty():
            frame, robot_pos = frame_queue.get()

            # Draw logical path (ID sequence) in BLUE
            if robot_pos is not None and path_order:
                pts = [robot_pos] + [mineral_map[mid] for mid in path_order if mid in mineral_map]
                for i in range(len(pts) - 1):
                    cv2.line(
                        frame,
                        (int(pts[i][0]), int(pts[i][1])),
                        (int(pts[i+1][0]), int(pts[i+1][1])),
                        (255, 0, 0),
                        2
                    )   # BLUE

            # Draw planned path to current target in GREEN:
            # robot -> detour waypoint (if any) -> current target
            if current_target_id == BASE_ID and BASE_ID not in mineral_map:
                # print("[UI] Warning: Base position unknown, cannot draw path to base.")
                pass

            if robot_pos is not None and current_target_id and (current_target_id in mineral_map):
                rx, ry, rz = robot_pos
                tx, ty, tz = mineral_map[current_target_id]

                if CRISIS_EVENTS_ENABLED and detour_waypoint is not None:
                    wx, wy = detour_waypoint
                    # robot -> waypoint
                    cv2.line(
                        frame,
                        (int(rx), int(ry)),
                        (int(wx), int(wy)),
                        (0, 255, 0),
                        3
                    )
                    # waypoint -> target
                    cv2.line(
                        frame,
                        (int(wx), int(wy)),
                        (int(tx), int(ty)),
                        (0, 255, 0),
                        3
                    )
                else:
                    # straight line robot -> target
                    cv2.line(
                        frame,
                        (int(rx), int(ry)),
                        (int(tx), int(ty)),
                        (0, 255, 0),
                        3
                    )

            # Draw orange crisis radius if visible and enabled
            if CRISIS_EVENTS_ENABLED and weather_pos is not None:
                wx, wy, wz = weather_pos
                cv2.circle(
                    frame,
                    (int(wx), int(wy)),
                    int(WEATHER_AVOID_RADIUS_MM),
                    (0, 165, 255),
                    2
                )  # orange-ish circle
                cv2.putText(
                    frame,
                    "Localised weather",
                    (int(wx) + 10, int(wy) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 165, 255),
                    1
                )
                try:
                    status_extra_label.config(text="Localised weather detected")
                except Exception:
                    pass
            # =========================================================
            # DRAW COLOUR BLOB DETECTIONS
            # =========================================================
            if orange_blob is not None:
                cv2.circle(frame, orange_blob, 10, (0,165,255), -1)
                cv2.putText(frame, "Orange Crisis", (orange_blob[0]+10, orange_blob[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,165,255), 1)

            if red_blob is not None:
                cv2.circle(frame, red_blob, 10, (0,0,255), -1)
                cv2.putText(frame, "Red Crisis", (red_blob[0]+10, red_blob[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

            if green_blob is not None:
                cv2.circle(frame, green_blob, 10, (0,255,0), -1)
                cv2.putText(frame, "Green Crisis", (green_blob[0]+10, green_blob[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

            # Countdown
            if waiting_for_next_target:
                remaining = max(0, int(TARGET_PAUSE_SEC - (time.time() - target_reached_time)))
                cv2.putText(
                    frame,
                    f"Next target in {remaining}s",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 255),
                    2
                )
            # flashing warning triangle for critical event
            if critical_event_active and critical_event_colour is not None:
                now = time.time()
                if now - critical_event_last_toggle >= 0.5:
                    critical_event_visible = not critical_event_visible
                    critical_event_last_toggle = now

                if critical_event_visible:
                    h, w, _ = frame.shape
                    pts = np.array([[w - 80, 40], [w - 40, 100], [w - 120, 100]], np.int32)
                    cv2.fillConvexPoly(frame, [pts], critical_event_colour)
                    cv2.putText(frame, "!", (w-82, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)


            # Frames are already filtered in the camera thread; overlays drawn on `vis_frame`.

            # Convert and resize using OpenCV (faster than PIL resize on full image)
            try:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                resized = cv2.resize(frame_rgb, (DISPLAY_W, DISPLAY_H), interpolation=cv2.INTER_AREA)
                img = ImageTk.PhotoImage(Image.fromarray(resized))
                camera_label.imgtk = img
                camera_label.config(image=img)
            except Exception:
                try:
                    # fallback to original method if any error
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = ImageTk.PhotoImage(Image.fromarray(frame).resize((DISPLAY_W, DISPLAY_H)))
                    camera_label.imgtk = img
                    camera_label.config(image=img)
                except Exception:
                    pass

        # Update status bar
        mode_str = ("Manual" if override_state == 1 else ("Autonomous" if system_active else "Idle"))
        # If we're actively investigating a live green target, show that in the status
        try:
            if green_investigate and green_target_pos is not None and (time.time() - green_last_seen) < GREEN_INVESTIGATE_TIMEOUT:
                target_str = "Green"
            else:
                target_str = (str(current_target_id) if current_target_id is not None else "---")
        except Exception:
            target_str = (str(current_target_id) if current_target_id is not None else "---")
        if robot_pos is not None:
            rx, ry, rz = robot_pos
            robot_str = f"({int(rx)}, {int(ry)})"
        else:
            robot_str = "(---, ---)"

        try:
            status_mode_label.config(text=f"Mode: {mode_str}")
            status_target_label.config(text=f"Target: {target_str}")
            status_robot_label.config(text=f"Robot: {robot_str}")
        except Exception:
            pass

        # ========================================================
        # minimap drawing
        # ========================================================
        try:
            global minimap_last_draw_time
            now_m = time.time()
            if (now_m - minimap_last_draw_time) >= MINIMAP_UPDATE_INTERVAL:
                minimap_last_draw_time = now_m
                minimap.delete("all")
                minimap.create_rectangle(0,0,minimap_sizex,minimap_sizey, outline="#2B4A6A")

                def map_to_minimap(x,y, scale=0.2):
                    cx = minimap_sizex / 2 + x * scale
                    cy = (minimap_sizey / 2 + y * scale)
                    return cx, cy

                # Draw minerals
                for mid, pos in mineral_map.items():
                    if mid in MINERAL_IDS:
                        mx, my, mz = pos
                        px, py = map_to_minimap(mx, my)
                        minimap.create_oval(px-3, py-3, px+3, py+3, fill="#4FD1C5", outline="")  # teal for minerals

                # Draw base if known
                if BASE_ID in mineral_map:
                    bx, by, bz = mineral_map[BASE_ID]
                    px, py = map_to_minimap(bx, by)
                    minimap.create_rectangle(px-3, py-3, px+3, py+3, outline="#F6E05E", width=2)  # yellow for base

                # Draw crisis events
                if weather_pos is not None:
                    wx, wy, wz = weather_pos
                    px, py = map_to_minimap(wx, wy)
                    minimap.create_oval(px-6, py-6, px+6, py+6, outline="#F6AD55", width=2)  # orange for crisis

                # Draw red crisis blob (severe)
                if red_blob is not None:
                    rbx, rby = red_blob
                    px, py = map_to_minimap(rbx, rby)
                    minimap.create_oval(px-6, py-6, px+6, py+6, outline="#F56565", width=2)  # red for severe

                # Draw green crisis blob (moving object)
                if green_blob is not None:
                    gbx, gby = green_blob
                    px, py = map_to_minimap(gbx, gby)
                    minimap.create_oval(px-6, py-6, px+6, py+6, outline="#34D399", width=2)  # green for object

                # Draw robot
                if robot_pos is not None:
                    rx, ry, rz = robot_pos
                    px, py = map_to_minimap(rx, ry)
                    minimap.create_polygon(px, py - 6,px - 5, py + 6,    px + 5, py + 6,  fill="#BA0F7E")  # blue triangle for robot
        except Exception:
            pass

        # Update live plot if enabled (throttled)
        try:
            global plotting_enabled, plot_times, plot_angles, plot_distances, plot_canvas, plot_last_draw_time
            now = time.time()
            if plot_canvas is not None:
                # Only redraw at most every PLOT_UPDATE_INTERVAL seconds
                if plotting_enabled and plot_times and (now - plot_last_draw_time) >= PLOT_UPDATE_INTERVAL:
                    plot_last_draw_time = now
                    plot_ax.clear()
                    dist_ax.clear()

                    plot_fig.patch.set_facecolor('#0B1020')
                    plot_ax.set_facecolor('#0B1020')

                    last_t = plot_times[-1] if plot_times else 0.0
                    window_start = math.floor(last_t / 10.0) * 10.0
                    window_end = window_start + 10.0

                    plot_ax.plot(plot_times, plot_angles, color='tab:blue')
                    dist_ax.plot(plot_times, plot_distances, color='#F6E05E')

                    plot_ax.set_ylim(-180, 180)
                    dist_ax.set_ylim(0, 2000)
                    plot_ax.set_ylabel('Angle (deg)', color='tab:blue')
                    dist_ax.set_ylabel('Distance (mm)', color='#F6E05E')
                    plot_ax.set_xlabel('Time (s)')

                    plot_ax.tick_params(axis='y', colors='tab:blue')
                    dist_ax.tick_params(axis='y', colors='#F6E05E')
                    try:
                        plot_ax.spines['left'].set_color('tab:blue')
                        dist_ax.spines['right'].set_color('#F6E05E')
                    except Exception:
                        pass

                    plot_ax.set_xlim(window_start, window_end)
                    plot_ax.grid(False)
                    plot_canvas.draw_idle()
                elif not plotting_enabled and plot_canvas is not None and (now - plot_last_draw_time) >= PLOT_UPDATE_INTERVAL:
                    # clear when not plotting (throttled)
                    plot_last_draw_time = now
                    plot_ax.clear()
                    dist_ax.clear()
                    plot_canvas.draw_idle()
        except Exception:
            pass
    except Exception:
        pass
    finally:
        root.after(50, update_ui)
# =========================================================
# CLEANUP
# =========================================================
def on_close():
    stop_event.set()
    cap.release()
    try:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
    except Exception:
        pass
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# =========================================================
# START THREADS & UI
# =========================================================
threading.Thread(target=camera_thread, daemon=True).start()
threading.Thread(target=joystick_udp_thread, daemon=True).start()
threading.Thread(target=mqtt_heartbeat_thread, daemon=True).start()

# Create joystick UI in the right frame
joystick_ui = JoystickUI(right_frame)

update_ui()
root.mainloop()

