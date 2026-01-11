# # MQTT comms script.

# import tkinter as tk
# import math
# import threading
# import queue
# import time
# import paho.mqtt.client as mqtt


# #MQTT broker details
# BROKER = "fesv-mqtt.bath.ac.uk"
# PORT = 31415
# USERNAME = "student"
# PASSWORD = "HousekeepingGlintsStreetwise"

# TOPIC_DESIRED_POSITION = "CottonCandyGrapes/BallRobot/DesiredPosition"
# TOPIC_CURRENT_POSITION = "CottonCandyGrapes/BallRobot/CurrentPosition"
# TOPIC_CURRENT_ANGLE = "CottonCandyGrapes/BallRobot/CurrentAngle"

#UPD test
import socket
import struct
import time

# ---------------------------
# UDP configuration
# ---------------------------
UDP_IP = "172.26.198.126"   # Change if Simulink is on another machine
UDP_PORT = 25000       # Must match UDP Receive block

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Sending test UDP packets: (50.0, 700.0) every second")

while True:
    x = 50.0
    y = 700.0

    # Pack as two float32 values (little endian)
    packet = struct.pack('<ff', x, y)

    sock.sendto(packet, (UDP_IP, UDP_PORT))

    print(f"Sent: x={x}, y={y}")

    time.sleep(1.0)
