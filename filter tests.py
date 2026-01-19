import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2.aruco as aruco

# -----------------------------
# CAMERA SETUP
# -----------------------------
cap = cv2.VideoCapture(1)

# ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters()

# -----------------------------
# TKINTER WINDOW
# -----------------------------
root = tk.Tk()
root.title("ArUco Visibility & Glare Reduction Tuner")

# -----------------------------
# IMAGE PANELS
# -----------------------------
raw_panel = tk.Label(root)
raw_panel.grid(row=0, column=0)

filtered_panel = tk.Label(root)
filtered_panel.grid(row=0, column=1)

# -----------------------------
# SLIDER FRAME
# -----------------------------
controls = tk.Frame(root)
controls.grid(row=1, column=0, columnspan=2)

def add_slider(text, from_, to_, row, default):
    label = tk.Label(controls, text=text)
    label.grid(row=row, column=0, sticky="w")
    slider = tk.Scale(controls, from_=from_, to=to_, orient="horizontal")
    slider.set(default)
    slider.grid(row=row, column=1)
    return slider

# -----------------------------
# SLIDERS
# -----------------------------
brightness = add_slider("Brightness", -100, 100, 0, 0)
contrast = add_slider("Contrast", 0, 3, 1, 1)
gamma = add_slider("Gamma", 1, 300, 2, 100)
blur = add_slider("Blur (odd only)", 1, 21, 3, 1)

# CLAHE sliders
clahe_clip = add_slider("CLAHE Clip Limit", 1, 10, 4, 2)
clahe_grid = add_slider("CLAHE Grid Size", 2, 16, 5, 8)

# -----------------------------
# FILTER MODE DROPDOWN
# -----------------------------
filter_mode_var = tk.StringVar()
filter_mode_var.set("None")

filter_modes = ["None", "Grayscale", "Adaptive Threshold", "Canny Edges", "CLAHE"]

dropdown = ttk.Combobox(controls, textvariable=filter_mode_var, values=filter_modes)
dropdown.grid(row=6, column=1)
tk.Label(controls, text="Filter Mode").grid(row=6, column=0)

# -----------------------------
# PROCESSING FUNCTION
# -----------------------------
def apply_filters(frame):
    # Convert to float for brightness/contrast
    img = frame.astype(np.float32)

    # Apply brightness & contrast
    img = img * contrast.get() + brightness.get()
    img = np.clip(img, 0, 255).astype(np.uint8)

    # Gamma correction
    g = gamma.get() / 100
    img = np.power(img / 255.0, 1.0 / g)
    img = (img * 255).astype(np.uint8)

    # Blur
    k = blur.get()
    if k % 2 == 0:
        k += 1
    img = cv2.GaussianBlur(img, (k, k), 0)

    mode = filter_mode_var.get()

    if mode == "Grayscale":
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    elif mode == "Adaptive Threshold":
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            11, 2
        )

    elif mode == "Canny Edges":
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.Canny(gray, 80, 150)

    elif mode == "CLAHE":
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(
            clipLimit=clahe_clip.get(),
            tileGridSize=(clahe_grid.get(), clahe_grid.get())
        )
        img = clahe.apply(gray)

    return img

# -----------------------------
# UPDATE LOOP
# -----------------------------
def update():
    ret, frame = cap.read()
    if not ret:
        root.after(10, update)
        return

    # Resize for display
    frame_small = cv2.resize(frame, (480, 360))

    # Apply filters
    filtered = apply_filters(frame_small)

    # Detect ArUco on filtered image (if grayscale)
    if len(filtered.shape) == 2:
        corners, ids, _ = aruco.detectMarkers(filtered, aruco_dict, parameters=parameters)
        if ids is not None:
            filtered = cv2.cvtColor(filtered, cv2.COLOR_GRAY2BGR)
            filtered = aruco.drawDetectedMarkers(filtered, corners, ids)

    # Convert for Tkinter
    raw_img = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(frame_small, cv2.COLOR_BGR2RGB)))
    filtered_img = ImageTk.PhotoImage(image=Image.fromarray(filtered))

    raw_panel.config(image=raw_img)
    raw_panel.image = raw_img

    filtered_panel.config(image=filtered_img)
    filtered_panel.image = filtered_img

    root.after(10, update)

update()
root.mainloop()
