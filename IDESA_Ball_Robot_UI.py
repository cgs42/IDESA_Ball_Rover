# User interface to start the automated running of the IDESA ball robot. Also includes a manual override. 
import tkinter as tk

# --------------------
# Main window
# --------------------
root = tk.Tk()
root.title("IDESA Ball Rover UI")
root.geometry("900x500")
root.configure(bg="black")

# --------------------
# Layout frames
# --------------------
left_frame = tk.Frame(root, bg="black", width=450)
right_frame = tk.Frame(root, bg="black", width=450)

left_frame.pack(side="left", fill="both", expand=True)
right_frame.pack(side="right", fill="both", expand=True)

# --------------------
# LEFT SIDE
# --------------------
find_button = tk.Button(
    left_frame,
    text="Find Minerals",
    bg="#222",
    fg="white",
    font=("Arial", 14),
    width=15
)
find_button.pack(pady=20)

# Grid placeholder
grid_frame = tk.Frame(left_frame, bg="white")
grid_frame.pack()

ROWS = 5
COLS = 7

for r in range(ROWS):
    for c in range(COLS):
        cell = tk.Label(
            grid_frame,
            width=6,
            height=3,
            bg="black",
            borderwidth=1,
            relief="solid"
        )
        cell.grid(row=r, column=c, padx=1, pady=1)

# --------------------
# RIGHT SIDE
# --------------------
title = tk.Label(
    right_frame,
    text="Manual Override",
    bg="black",
    fg="white",
    font=("Arial", 16)
)
title.pack(pady=20)

manual_override = False

def toggle_manual():
    global manual_override
    manual_override = not manual_override
    toggle_button.config(text="ON" if manual_override else "OFF")
    update_dpad_state()

toggle_button = tk.Button(
    right_frame,
    text="OFF",
    bg="#222",
    fg="white",
    font=("Arial", 14),
    width=8,
    command=toggle_manual
)
toggle_button.pack(pady=10)

# --------------------
# D-Pad
# --------------------
dpad_frame = tk.Frame(right_frame, bg="black")
dpad_frame.pack(pady=30)

def dpad_press(direction):
    if manual_override:
        print(f"D-pad: {direction}")

def update_dpad_state():
    state = "normal" if manual_override else "disabled"
    for btn in dpad_buttons:
        btn.config(state=state)

up = tk.Button(dpad_frame, text="↑", width=5, height=2, command=lambda: dpad_press("UP"))
down = tk.Button(dpad_frame, text="↓", width=5, height=2, command=lambda: dpad_press("DOWN"))
left = tk.Button(dpad_frame, text="←", width=5, height=2, command=lambda: dpad_press("LEFT"))
right = tk.Button(dpad_frame, text="→", width=5, height=2, command=lambda: dpad_press("RIGHT"))

up.grid(row=0, column=1)
left.grid(row=1, column=0)
right.grid(row=1, column=2)
down.grid(row=2, column=1)

dpad_buttons = [up, down, left, right]
update_dpad_state()

# --------------------
# Start UI
# --------------------
root.mainloop()
