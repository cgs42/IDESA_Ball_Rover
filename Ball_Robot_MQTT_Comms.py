# MQTT comms script.

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

TOPIC_DESIRED_POSITION = "CottonCandyGrapes/BallRobot/DesiredPosition"
TOPIC_CURRENT_POSITION = "CottonCandyGrapes/BallRobot/CurrentPosition"
TOPIC_CURRENT_ANGLE = "CottonCandyGrapes/BallRobot/CurrentAngle"