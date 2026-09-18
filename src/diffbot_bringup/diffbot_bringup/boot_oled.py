#!/usr/bin/env python3
import os
import socket
import subprocess
import time

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import ImageFont

WIDTH = 128
HEIGHT = 64


# ----------------------------
# Connect OLED
# ----------------------------
def connect_display():
    for addr in (0x3C, 0x3D):
        try:
            serial = i2c(port=1, address=addr)
            return ssd1306(serial, width=WIDTH, height=HEIGHT)
        except:
            pass
    return None


device = connect_display()

if device is None:
    print("OLED not found")
    exit()


# ----------------------------
# Fonts
# ----------------------------
try:
    title_font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 10
    )

    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 8
    )

except:
    title_font = ImageFont.load_default()
    font = ImageFont.load_default()

# ----------------------------
# Boot Animation
# ----------------------------
# ----------------------------
# Professional Boot Animation
# ----------------------------
def boot_animation():

    boot_steps = [

        ("Initializing...", 20),
        ("Loading Drivers...", 45),
        ("Connecting WiFi...", 70),
        ("Starting ROS...", 90),
        ("Ready", 100)

    ]

    for message, percent in boot_steps:

        with canvas(device) as draw:

            # Border
            draw.rectangle((0, 0, 127, 63), outline="white")

            # Title
            draw.text(
                (24, 6),
                "Bumpy Gamma",
                font=title_font,
                fill="white"
            )

            draw.line((5, 18, 122, 18), fill="white")

            # Current stage
            draw.text(
                (10, 26),
                message,
                font=font,
                fill="white"
            )

            # Progress outline
            draw.rectangle(
                (12, 45, 116, 55),
                outline="white"
            )

            # Filled progress
            fill = int(percent)

            draw.rectangle(
                (14, 47, 14 + fill - 5, 53),
                fill="white"
            )

        time.sleep(0.8)
boot_animation()
# ----------------------------
# Network
# ----------------------------
def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "--.--.--.--"


def wifi_status(ip):
    if ip == "--.--.--.--":
        return "Not connected"
    return "Connected"


# ----------------------------
# ROS Status
# ----------------------------
def ros_status():
    try:
        result = subprocess.run(
            ["pgrep", "-f", "ros2"],
            capture_output=True,
            text=True
        )

        if result.stdout.strip():
            return "RUN"

        return "WAIT"

    except:
        return "WAIT"

def oled_node_running():
    return os.path.exists("/tmp/oled_busy")
# ----------------------------
# Boot Status
# ----------------------------
def boot_status(ip):
    if ip == "--.--.--.--":
        return "BOOT"
    return "READY"


last_screen = None

while True:

    # If robot OLED node is running, let it control the display
    if oled_node_running():
        time.sleep(1)
        continue

    ip = get_ip()

    screen = (
        ip,
        wifi_status(ip),
        ros_status(),
        boot_status(ip)
    )

    if screen != last_screen:
        print("Redrawing status...")
        device = connect_display()

        with canvas(device) as draw:

            draw.rectangle((0, 0, 127, 63), outline="white")

            draw.text(
                (25, 2),
                "Bumpy Gamma",
                font=title_font,
                fill="white"
            )

            draw.line((2, 14, 125, 14), fill="white")

            draw.text(
                (4, 18),
                f"IP   {screen[0]}",
                font=font,
                fill="white"
            )

            draw.text(
                (4, 30),
                f"WiFi {screen[1]}",
                font=font,
                fill="white"
            )

            draw.text(
                (4, 42),
                f"ROS  {screen[2]}",
                font=font,
                fill="white"
            )

            draw.text(
                (4, 54),
                f"Boot {screen[3]}",
                font=font,
                fill="white"
            )

        last_screen = screen


    time.sleep(1)



