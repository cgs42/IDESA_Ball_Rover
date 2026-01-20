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
import heapq

# =========================================================
# CAMERA & ARUCO SETUP
# =========================================================
camera_calibration = np.load('workdir/Calibration.npz')
CM = camera_calibration['CM']
dist_coef = camera_calibration['dist_coef']

marker_size = 67 # aruco codes measure 67mm x 67mm
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

cap = cv2.VideoCapture(1)

# =========================================================
# UDP SETUP
# =========================================================
UDP_IP = "172.26.156.13"  # amy's IP = "172.26.198.126", Caitlin's IP = "172.26.156.13"
UDP_PORT = 25000
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

UDP_RATE_HZ = 1.0
last_udp_time = 0.0

# =========================================================
# MINERAL STATE
# =========================================================
MINERAL_IDS = [1, 2, 3, 4, 5]  # aruco code IDs representing minerals
COLLECTION_RADIUS_PX = 30      # distance from target position to consider mineral 'collected' in pixels
TARGET_PAUSE_SEC = 5.0         # time to wait at each target before moving to the next in seconds
DETECTION_TIME_SEC = 5.0       # time allowed for mineral detection phase to accomodate for detection flickering in seconds
BASE_ID = 6                    # aruco code ID representing home base
Robot_ID = 145                 # aruco code ID representing the robot
CRISIS_WEATHER_ID = 10         # aruco code ID representing localised weather event

WEATHER_AVOID_RADIUS_MM = 50   # radius around crisis event to avoid in mm

# set vectors and variables
mineral_map = {}               # id: (x, y, z) position in mm
remaining_minerals = []
unsafe_minerals = set()        # minerals currently too close to weather event
path_order = []                # logical order of mineral/base IDs (used for UI, not strict)

current_target_id = None       # current mineral/base ID we are logically heading for
collected_mineral_id = None

system_active = False
waiting_for_next_target = False
target_reached_time = 0.0

detecting_minerals = False
detection_start_time = 0.0

base_added_to_path = False     # ensures base is added only once
weather_pos = None             # (x, y, z) position of crisis weather event in mm

# =========================================================
# PATH PLANNING (A* ON GRID)
# =========================================================
# Arena and grid configuration
ARENA_WIDTH_MM = 2000
ARENA_HEIGHT_MM = 1200
GRID_RES_MM = 10  # 10mm per cell (G2 choice)

GRID_WIDTH = ARENA_WIDTH_MM // GRID_RES_MM   # 200
GRID_HEIGHT = ARENA_HEIGHT_MM // GRID_RES_MM # 120

def clamp_mm_to_arena(x_mm, y_mm):
    """Clamp mm coordinates to within arena bounds."""
    x_mm_clamped = max(0, min(ARENA_WIDTH_MM - 1, x_mm))
    y_mm_clamped = max(0, min(ARENA_HEIGHT_MM - 1, y_mm))
    return x_mm_clamped, y_mm_clamped

def mm_to_grid(x_mm, y_mm):
    """Convert mm coordinates to grid indices."""
    x_mm, y_mm = clamp_mm_to_arena(x_mm, y_mm)
    gx = int(x_mm / GRID_RES_MM)
    gy = int(y_mm / GRID_RES_MM)
    # Ensure within grid bounds
    gx = max(0, min(GRID_WIDTH - 1, gx))
    gy = max(0, min(GRID_HEIGHT - 1, gy))
    return gx, gy

def grid_to_mm(gx, gy):
    """Convert grid indices back to mm coordinates (center of cell)."""
    x_mm = (gx + 0.5) * GRID_RES_MM
    y_mm = (gy + 0.5) * GRID_RES_MM
    return x_mm, y_mm

def build_occupancy_grid(weather_pos_local):
    """
    Build occupancy grid for A*.
    Cells inside weather avoidance radius are blocked.
    """
    grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
    if weather_pos_local is not None:
        wx, wy, wz = weather_pos_local
        # Convert hazard radius to grid cells
        radius_cells = int(WEATHER_AVOID_RADIUS_MM / GRID_RES_MM)
        wgx, wgy = mm_to_grid(wx, wy)

        for gy in range(max(0, wgy - radius_cells - 2), min(GRID_HEIGHT, wgy + radius_cells + 3)):
            for gx in range(max(0, wgx - radius_cells - 2), min(GRID_WIDTH, wgx + radius_cells + 3)):
                cx_mm, cy_mm = grid_to_mm(gx, gy)
                if math.hypot(cx_mm - wx, cy_mm - wy) <= WEATHER_AVOID_RADIUS_MM:
                    grid[gy][gx] = 1  # blocked cell
    return grid

def a_star(start_g, goal_g, grid):
    """
    Standard A* pathfinding on 2D grid.
    start_g, goal_g: (gx, gy)
    grid: 0 = free, 1 = blocked
    Returns list of grid cells [(gx, gy), ...] including start and goal, or None if no path.
    """
    sx, sy = start_g
    gx, gy = goal_g
    if grid[sy][sx] == 1 or grid[gy][gx] == 1:
        return None

    open_set = []
    heapq.heappush(open_set, (0, (sx, sy)))
    came_from = {}
    g_score = { (sx, sy): 0 }

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    neighbors = [(-1, -1), (0, -1), (1, -1),
                 (-1,  0),          (1,  0),
                 (-1,  1), (0,  1), (1,  1)]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == (gx, gy):
            # reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dx, dy in neighbors:
            nx = current[0] + dx
            ny = current[1] + dy
            if nx < 0 or nx >= GRID_WIDTH or ny < 0 or ny >= GRID_HEIGHT:
                continue
            if grid[ny][nx] == 1:
                continue
            tentative_g = g_score[current] + math.hypot(dx, dy)
            if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                g_score[(nx, ny)] = tentative_g
                f = tentative_g + heuristic((nx, ny), (gx, gy))
                heapq.heappush(open_set, (f, (nx, ny)))
                came_from[(nx, ny)] = current

    return None

def plan_path_mm(start_pos, goal_pos, weather_pos_local):
    """
    Plan a path in mm coordinates from start_pos (robot) to goal_pos (mineral/base),
    avoiding the circular weather event region using A* on a grid.
    Returns list of (x_mm, y_mm) waypoints, including start and goal.
    Falls back to straight-line if no path is found.
    """
    sx, sy, sz = start_pos
    gx, gy, gz = goal_pos

    # Clamp to arena and convert to grid
    start_g = mm_to_grid(sx, sy)
    goal_g = mm_to_grid(gx, gy)

    grid = build_occupancy_grid(weather_pos_local)
    path_grid = a_star(start_g, goal_g, grid)

    if path_grid is None:
        # No path found – fall back to straight-line (still useful)
        return [(sx, sy), (gx, gy)]

    path_mm = [grid_to_mm(gx, gy) for (gx, gy) in path_grid]

    # Ensure exact goal included
    if math.hypot(path_mm[-1][0] - gx, path_mm[-1][1] - gy) > GRID_RES_MM:
        path_mm.append((gx, gy))

    # Also make sure we start exactly at robot position
    if math.hypot(path_mm[0][0] - sx, path_mm[0][1] - sy) > GRID_RES_MM:
        path_mm.insert(0, (sx, sy))

    return path_mm

# Waypoint state for following planned paths
current_waypoints = []     # list of (x_mm, y_mm)
current_waypoint_index = 0 # index into current_waypoints

# =========================================================
# PATH PLANNING (MINERAL ORDER)
# =========================================================
def compute_shortest_path(start_pos, mineral_items):
    """
    Original nearest-neighbor heuristic over minerals (no hazards).
    Now used to define logical order (path_order) only, not geometric path.
    """
    path = []
    current = start_pos
    items = mineral_items.copy()

    while items:
        mid, pos = min(
            items,
            key=lambda x: math.hypot(current[0] - x[1][0], current[1] - x[1][1]))
        path.append(mid)
        current = pos
        items = [i for i in items if i[0] != mid]

    return path

def is_mineral_safe(mid, weather_pos_local):
    """
    Check if a mineral is currently safe (far enough from the crisis event).
    """
    if weather_pos_local is None:
        return True
    if mid not in mineral_map:
        return True
    mx, my, mz = mineral_map[mid]
    wx, wy, wz = weather_pos_local
    return math.hypot(mx - wx, my - wy) > WEATHER_AVOID_RADIUS_MM

def update_safe_unsafe_minerals():
    """
    Move minerals between remaining_minerals and unsafe_minerals depending on weather proximity.
    """
    global unsafe_minerals

    if weather_pos is None:
        # All remaining minerals are safe if there's no weather event
        unsafe_minerals.clear()
        return

    # Mark too-close minerals as unsafe
    newly_unsafe = []
    for mid in remaining_minerals:
        if not is_mineral_safe(mid, weather_pos):
            newly_unsafe.append(mid)
    for mid in newly_unsafe:
        remaining_minerals.remove(mid)
        unsafe_minerals.add(mid)

    # Check if any unsafe minerals have become safe again
    newly_safe = []
    for mid in list(unsafe_minerals):
        if is_mineral_safe(mid, weather_pos):
            newly_safe.append(mid)
    for mid in newly_safe:
        unsafe_minerals.remove(mid)
        remaining_minerals.append(mid)

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
    global detecting_minerals, detection_start_time, base_added_to_path
    global current_waypoints, current_waypoint_index, unsafe_minerals, weather_pos

    system_active = not system_active

    if system_active:
        print("\n[SYSTEM] MINERAL FIND STARTED")
        mineral_map.clear()
        remaining_minerals.clear()
        unsafe_minerals.clear()
        path_order.clear()
        current_target_id = None
        waiting_for_next_target = False
        detecting_minerals = True
        detection_start_time = time.time()
        base_added_to_path = False
        current_waypoints = []
        current_waypoint_index = 0
        weather_pos = None
        button.config(text="End Mineral Find")
    else:
        print("\n[SYSTEM] MINERAL FIND STOPPED")
        button.config(text="Find Minerals")
        current_target_id = None
        waiting_for_next_target = False
        detecting_minerals = False
        current_waypoints = []
        unsafe_minerals.clear()

button = tk.Button(left_frame,text="Find Minerals",bg="#222",fg="white",font=("Arial", 14),command=toggle_mineral_find)
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
    global system_active, base_added_to_path, weather_pos
    global current_waypoints, current_waypoint_index

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        robot_pos = None
        robot_yaw = None
        robot_green_yaw = None
        robot_green_vec = None

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)

            for i, mid in enumerate(ids.flatten()):
                tvec = tvecs[i][0]  # in mm
                R, _ = cv2.Rodrigues(rvecs[i])

                # Robot detection
                if mid == Robot_ID:
                    robot_pos = tvec
                    robot_yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
                    try:
                        robot_green_yaw = math.degrees(math.atan2(R[1, 1], R[0, 1]))
                        robot_green_vec = np.array([R[0, 1], R[1, 1]], dtype=float)
                    except Exception:
                        robot_green_yaw = robot_yaw
                        robot_green_vec = None

                # Base detection (always active)
                if mid == BASE_ID:
                    mineral_map[BASE_ID] = tvec

                # Crisis weather event detection (always active)
                if mid == CRISIS_WEATHER_ID:
                    weather_pos = tvec

                # Mineral detection (only during scan)
                if detecting_minerals and mid in MINERAL_IDS:
                    mineral_map.setdefault(mid, tvec)

                frame = cv2.drawFrameAxes(frame, CM, dist_coef, rvecs[i], tvecs[i], 100)

            frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # Finish detection phase
        if detecting_minerals and time.time() - detection_start_time >= DETECTION_TIME_SEC:
            detecting_minerals = False
            remaining_minerals[:] = [mid for mid in mineral_map.keys() if mid in MINERAL_IDS]
            print("Mineral map", mineral_map)

            if robot_pos is not None:
                # Initial logical order (no hazard consideration here; avoidance is handled by path planner and safe/unsafe)
                path_order[:] = compute_shortest_path(
                    robot_pos,
                    [(mid, mineral_map[mid]) for mid in remaining_minerals]
                )
                print("[PATH] Initial mineral order:", path_order)

        # Update safe/unsafe minerals based on current weather position
        if system_active and not detecting_minerals:
            update_safe_unsafe_minerals()

        # Select next target ID (mineral or base)
        if system_active and not detecting_minerals and current_target_id is None and not waiting_for_next_target:
            # If there are safe minerals, choose the nearest safe one
            if remaining_minerals and robot_pos is not None:
                safe_minerals = [mid for mid in remaining_minerals if is_mineral_safe(mid, weather_pos)]
                if safe_minerals:
                    # Choose nearest safe mineral to current robot position
                    current_target_id = min(safe_minerals,key=lambda mid: math.hypot(robot_pos[0] - mineral_map[mid][0],robot_pos[1] - mineral_map[mid][1]))
                else:
                    # No safe minerals for now – wait
                    current_target_id = None

            # After all minerals collected, go to base (once it has been added)
            if not remaining_minerals and not unsafe_minerals and BASE_ID in mineral_map and not base_added_to_path:
                print("\n[SYSTEM] All minerals collected. Returning to base.")
                current_target_id = BASE_ID
                base_added_to_path = True

            if current_target_id is not None and robot_pos is not None:
                print("[NAV] Logical target ID:", current_target_id)
                # Plan a geometric path to this target using A*
                target_pos = mineral_map[current_target_id]
                current_waypoints = plan_path_mm(robot_pos, target_pos, weather_pos)
                current_waypoint_index = 0
                print(f"[PATH] Planned {len(current_waypoints)} waypoints to target {current_target_id}")

        # Collection logic
        if system_active and robot_pos is not None and current_target_id and not waiting_for_next_target:
            # Collection is still based on proximity to the actual marker position, not waypoints
            tx, ty, tz = mineral_map[current_target_id]
            if math.hypot(robot_pos[0] - tx, robot_pos[1] - ty) < COLLECTION_RADIUS_PX:
                collected_mineral_id = current_target_id

                # If base reached
                if current_target_id == BASE_ID:
                    print("\n[SYSTEM] Return to base completed. Mission ended.")
                    system_active = False
                    button.config(text="Find Minerals")
                    current_target_id = None
                    waiting_for_next_target = False
                    current_waypoints = []
                    continue

                # Normal mineral collection
                if current_target_id in remaining_minerals:
                    remaining_minerals.remove(current_target_id)
                if current_target_id in unsafe_minerals:
                    unsafe_minerals.discard(current_target_id)
                if path_order and path_order[0] == current_target_id:
                    path_order.pop(0)

                current_target_id = None
                current_waypoints = []
                waiting_for_next_target = True
                target_reached_time = time.time()
                print("[COLLECT] Mineral reached")

        # Pause between targets
        if waiting_for_next_target and time.time() - target_reached_time >= TARGET_PAUSE_SEC:
            waiting_for_next_target = False
            collected_mineral_id = None

        # Dynamic replanning in presence of moving weather event
        # (replan path to current target if weather_pos exists and robot_pos exists)
        if system_active and current_target_id and robot_pos is not None and not waiting_for_next_target:
            if weather_pos is not None:
                target_pos = mineral_map[current_target_id]
                current_waypoints = plan_path_mm(robot_pos, target_pos, weather_pos)
                current_waypoint_index = 0

        # UDP sending (now towards current waypoint if available, else towards marker)
        if system_active and robot_pos is not None and robot_yaw is not None and current_target_id:
            now = time.time()
            if now - last_udp_time >= 1.0 / UDP_RATE_HZ:
                # Choose navigation target: current waypoint if available
                if current_waypoints and current_waypoint_index < len(current_waypoints):
                    wx_mm, wy_mm = current_waypoints[current_waypoint_index]
                    tx, ty, tz = mineral_map[current_target_id]
                    # Advance waypoint when close enough
                    if math.hypot(robot_pos[0] - wx_mm, robot_pos[1] - wy_mm) < GRID_RES_MM * 1.5:
                        current_waypoint_index += 1
                        if current_waypoint_index < len(current_waypoints):
                            wx_mm, wy_mm = current_waypoints[current_waypoint_index]
                        else:
                            wx_mm, wy_mm = tx, ty
                    target_x = wx_mm
                    target_y = wy_mm
                else:
                    # Fallback: go straight to marker
                    tx, ty, tz = mineral_map[current_target_id]
                    target_x = tx
                    target_y = ty

                rx, ry, rz = robot_pos

                dx = target_x - rx
                dy = target_y - ry

                # Compute signed relative angle between robot green-axis and path using dot/cross
                if robot_green_vec is not None:
                    forward = robot_green_vec.astype(float)
                    f_norm = np.linalg.norm(forward)
                    if f_norm > 1e-6:
                        forward /= f_norm
                    else:
                        forward = np.array([0.0, 1.0], dtype=float)
                else:
                    if robot_green_yaw is not None:
                        ang = math.radians(robot_green_yaw)
                        forward = np.array([math.cos(ang), math.sin(ang)], dtype=float)
                    else:
                        forward = np.array([0.0, 1.0], dtype=float)

                path = np.array([dx, dy], dtype=float)
                p_norm = np.linalg.norm(path)
                if p_norm > 1e-6:
                    path_unit = path / p_norm
                else:
                    path_unit = np.array([0.0, 0.0], dtype=float)

                dot = float(forward[0]*path_unit[0] + forward[1]*path_unit[1])
                cross = float(forward[0]*path_unit[1] - forward[1]*path_unit[0])
                angle_rad = math.atan2(cross, dot)
                relative_angle = math.degrees(angle_rad)
                relative_distance = math.hypot(dx, dy)

                udp_sock.sendto(
                    struct.pack('<ff', relative_distance, relative_angle),
                    (UDP_IP, UDP_PORT)
                )
                print(f"[UDP] distance = {relative_distance:.1f}mm angle = {relative_angle:.1f}° (ID {current_target_id})")
                last_udp_time = now

        if not frame_queue.full():
            frame_queue.put((frame, robot_pos))

# =========================================================
# UI LOOP
# =========================================================
def update_ui():
    if not frame_queue.empty():
        frame, robot_pos = frame_queue.get()

        # Draw full planned geometric path (A* waypoints) in GREEN
        if robot_pos is not None and current_waypoints:
            # Start from robot position
            prev = (int(robot_pos[0]), int(robot_pos[1]))

            for (wx_mm, wy_mm) in current_waypoints:
                curr = (int(wx_mm), int(wy_mm))
                cv2.line(frame,
                         prev,
                         curr,
                         (0, 255, 0), 3)   # GREEN path, thicker line
                prev = curr

        # Draw logical sequence of remaining IDs (for reference) in BLUE
        if robot_pos is not None and path_order:
            pts = [robot_pos] + [mineral_map[mid] for mid in path_order if mid in mineral_map]
            for i in range(len(pts) - 1):
                cv2.line(frame,
                         (int(pts[i][0]), int(pts[i][1])),
                         (int(pts[i+1][0]), int(pts[i+1][1])),
                         (255, 0, 0), 1)   # BLUE thin line

        # Draw weather event radius if visible
        if weather_pos is not None:
            wx, wy, wz = weather_pos
            cv2.circle(frame,
                       (int(wx), int(wy)),
                       int(WEATHER_AVOID_RADIUS_MM),
                       (0, 0, 255), 2)
            cv2.putText(frame, "Weather event",
                        (int(wx) + 10, int(wy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255), 1)

        # Countdown
        if waiting_for_next_target:
            remaining = max(0, int(TARGET_PAUSE_SEC - (time.time() - target_reached_time)))
            cv2.putText(frame, f"Next target in {remaining}s",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 255), 2)

        # Convert frame to Tkinter
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
