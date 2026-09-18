#!/usr/bin/env python3
# ROS2 OLED Display Node for Bumpy Gamma
# Sequence: boot splash (0-5s) → system info (5-12s) → robot eyes (12s+)
# Dependencies: pip3 install luma.oled Pillow
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import subprocess
import threading
import socket
import getpass
import time
import math
import random

# ── Third-party OLED / PIL libraries ─────────────────────────────────────────
# Install with:  pip3 install luma.oled Pillow
try:
    from luma.core.interface.serial import i2c
    from luma.core.render import canvas
    from luma.oled.device import ssd1306, sh1106
    LUMA_AVAILABLE = True
except ImportError as e:
    LUMA_AVAILABLE = False
    print(f"[oled_display_node] ERROR: luma.oled not installed -> {e}")
    print("  Fix:  pip3 install luma.oled")

try:
    from PIL import ImageFont
    PIL_AVAILABLE = True
except ImportError as e:
    PIL_AVAILABLE = False
    print(f"[oled_display_node] ERROR: Pillow not installed -> {e}")
    print("  Fix:  pip3 install Pillow")


def _text_width(font, text):
    """Return pixel width of text, compatible with all Pillow versions."""
    try:
        bbox = font.getbbox(text)
        return bbox[2] - bbox[0]
    except AttributeError:
        try:
            return int(font.getlength(text))
        except AttributeError:
            return len(text) * 7     # fallback for ImageFont.load_default()


class OledDisplayNode(Node):

    W = 128
    H = 64

    def __init__(self):
        super().__init__('oled_display_node')

        # ── Library availability check ────────────────────────────────────────
        if not LUMA_AVAILABLE or not PIL_AVAILABLE:
            self.get_logger().error(
                'luma.oled or Pillow missing — display disabled. '
                'Run: pip3 install luma.oled Pillow')
            # Do NOT raise — let bringup continue without the display
            self._display_ok = False
            self._init_state()
            return

        # ── Hardware init (with retry logic) ──────────────────────────────────
        self._display_ok = False
        for attempt, addr in enumerate([0x3C, 0x3D]):
            try:
                serial = i2c(port=1, address=addr)
                self.device = ssd1306(serial, width=self.W, height=self.H)
                # For SH1106 replace the line above with:
                # self.device = sh1106(serial, width=self.W, height=self.H)
                self._display_ok = True
                self.get_logger().info(f'OLED found at I2C address 0x{addr:02X}')
                break
            except Exception as e:
                self.get_logger().warn(
                    f'OLED not found at 0x{addr:02X}: {e}')

        if not self._display_ok:
            self.get_logger().error(
                'OLED hardware not found on I2C port 1 (tried 0x3C and 0x3D). '
                'Check wiring and that I2C is enabled (raspi-config). '
                'Robot bringup continues without display.')

        # ── Fonts ─────────────────────────────────────────────────────────────
        if self._display_ok:
            FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
            FONT_REG  = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
            try:
                self.font_large = ImageFont.truetype(FONT_BOLD, 14)
                self.font_med   = ImageFont.truetype(FONT_REG,  11)
                self.font_small = ImageFont.truetype(FONT_REG,   9)
            except (IOError, OSError):
                self.get_logger().warn('DejaVu fonts not found — using default font.')
                self.font_large = ImageFont.load_default()
                self.font_med   = ImageFont.load_default()
                self.font_small = ImageFont.load_default()

        # ── State init ────────────────────────────────────────────────────────
        self._init_state()

        # ── ROS subscribers ───────────────────────────────────────────────────
        self.create_subscription(Twist,    'cmd_vel', self._cb_cmd_vel, 10)
        self.create_subscription(Odometry, 'odom',    self._cb_odom,    10)

        # ── 20 fps display timer ──────────────────────────────────────────────
        self.create_timer(0.05, self._display_tick)

        status = "display ACTIVE" if self._display_ok else "display OFFLINE (check hardware)"
        self.get_logger().info(f'OLED node started — {status}')

    def _init_state(self):
        """Initialise all animation/tracking state variables."""
        self.username      = getpass.getuser()
        self.ip_address    = self._get_ip()
        self.linear_speed  = 0.0
        self.angular_speed = 0.0

        self.start_time  = time.time()
        self.anim_phase  = 0.0
        self.blink_state = False
        self.blink_timer = 0.0

        self.eye_mode  = "normal"
        self.tgt_look  = 0.0
        self.cur_look  = 0.0
        self.tgt_run   = 0.0
        self.cur_run   = 0.0
        self.eye_squash = 0.0

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _get_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "No IP"

    # ─────────────────────────────────────────────────────────────────────────
    # Subscribers
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_cmd_vel(self, msg: Twist):
        self.linear_speed  = msg.linear.x
        self.angular_speed = msg.angular.z

        if abs(msg.linear.x) > 0.2:
            self.eye_mode = "running"
            self.tgt_run  = min(1.0, abs(msg.linear.x) / 2.0)
            self.tgt_look = 0.0
        elif abs(msg.angular.z) > 0.2:
            self.tgt_run = 0.0
            if msg.angular.z > 0:
                self.eye_mode = "look_left"
                self.tgt_look = min(1.0, abs(msg.angular.z))
            else:
                self.eye_mode = "look_right"
                self.tgt_look = -min(1.0, abs(msg.angular.z))
        else:
            self.eye_mode = "normal"
            self.tgt_look = 0.0
            self.tgt_run  = 0.0

    def _cb_odom(self, msg: Odometry):
        self.linear_speed  = msg.twist.twist.linear.x
        self.angular_speed = msg.twist.twist.angular.z

    # ─────────────────────────────────────────────────────────────────────────
    # Main display tick
    # ─────────────────────────────────────────────────────────────────────────

    def _display_tick(self):
        # Skip drawing if display hardware is not available
        if not self._display_ok:
            return

        elapsed = time.time() - self.start_time
        self.anim_phase += 0.1
        self._update_smooth()

        try:
            if elapsed < 5.0:
                self._draw_boot(elapsed)
            elif elapsed < 8.0:
                self._draw_info()
            elif elapsed < 10.0:
                self._draw_eye_opening(elapsed)
            else:
                self._draw_eyes()
        except Exception as e:
            self.get_logger().warn(f'Display render error: {e}', throttle_duration_sec=5.0)

        # Blink update
        self.blink_timer += 0.05
        if self.blink_timer > 3.0:
            self.blink_state = True
            if self.blink_timer > 3.15:
                self.blink_state = False
                self.blink_timer = 0.0

    # ─────────────────────────────────────────────────────────────────────────
    # Eye smooth transitions
    # ─────────────────────────────────────────────────────────────────────────

    def _update_smooth(self):
        d = self.tgt_look - self.cur_look
        self.cur_look = (self.cur_look + d * 0.1) if abs(d) > 0.01 else self.tgt_look

        r = self.tgt_run - self.cur_run
        self.cur_run = (self.cur_run + r * 0.1) if abs(r) > 0.01 else self.tgt_run

        if self.eye_mode == "running":
            self.eye_squash = math.sin(self.anim_phase * 1.5) * 0.2 * self.cur_run
        else:
            self.eye_squash *= 0.8

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 1 – Boot splash
    # ─────────────────────────────────────────────────────────────────────────

    def _draw_boot(self, elapsed):
        W, H = self.W, self.H
        with canvas(self.device) as draw:
            draw.rectangle([0, 0, W - 1, H - 1], outline="white", fill="black")

            title = "Bumpy Gamma"
            tw = _text_width(self.font_large, title)
            draw.text(((W - tw) // 2, 6), title, fill="white", font=self.font_large)

            sub = "ROS2 Robot"
            sw = _text_width(self.font_small, sub)
            draw.text(((W - sw) // 2, 24), sub, fill="white", font=self.font_small)

            # Animated loading bar
            bx, by, bw, bh = 10, 42, 108, 12
            progress = min(elapsed / 5.0, 1.0)
            fill_w   = int(bw * progress)

            draw.rectangle([bx, by, bx + bw, by + bh], outline="white", fill="black")
            if fill_w > 0:
                draw.rectangle([bx, by, bx + fill_w, by + bh], fill="white")

            pct = f"{int(progress * 100)}%"
            pw  = _text_width(self.font_small, pct)
            cx  = bx + (bw - pw) // 2
            col = "black" if fill_w > (bw // 2 + pw // 2) else "white"
            draw.text((cx, by + 2), pct, fill=col, font=self.font_small)

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 2 – System info
    # ─────────────────────────────────────────────────────────────────────────

    def _draw_info(self):
        W, H = self.W, self.H
        with canvas(self.device) as draw:
            draw.rectangle([0, 0, W - 1, H - 1], outline="white", fill="black")

            header = "[ Bumpy Gamma ]"
            hw = _text_width(self.font_med, header)
            draw.text(((W - hw) // 2, 3), header, fill="white", font=self.font_med)

            draw.line([(0, 18), (W - 1, 18)], fill="white")   # separator

            draw.text((4, 22), f"User: {self.username}",   fill="white", font=self.font_small)
            draw.text((4, 34), f"IP:   {self.ip_address}", fill="white", font=self.font_small)
            draw.text((4, 46),
                      f"v={self.linear_speed:+.2f} w={self.angular_speed:+.2f}",
                      fill="white", font=self.font_small)

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 3 – Robot eyes
    # ─────────────────────────────────────────────────────────────────────────
    def _draw_eye_opening(self, elapsed):
        """
        Smooth eye-opening animation.
        Runs from 8s to 10s.
        """

        progress = min((elapsed - 8.0) / 2.0, 1.0)

        W = self.W
        H = self.H

        eye_width = 30

        # Height grows from 3 pixels to 30 pixels
        eye_height = int(3 + progress * 27)

        left_x = 40
        right_x = 88
        center_y = 32

        with canvas(self.device) as draw:

            draw.rectangle(
                (0, 0, W - 1, H - 1),
                fill="black"
            )

            for cx in (left_x, right_x):

                x1 = cx - eye_width // 2
                x2 = cx + eye_width // 2

                y1 = center_y - eye_height // 2
                y2 = center_y + eye_height // 2

                draw.rounded_rectangle(
                    (x1, y1, x2, y2),
                    radius=8,
                    fill="white"
                )

                if progress > 0.6:

                    pupil = 8

                    draw.ellipse(
                        (
                            cx - pupil // 2,
                            center_y - pupil // 2,
                            cx + pupil // 2,
                            center_y + pupil // 2
                        ),
                        fill="black"
                    )
    def _draw_eyes(self):
        x_off = y_off = 0.0
        if self.cur_run > 0.05:
            x_off  = random.uniform(-3.0, 3.0) * self.cur_run
            y_off  = random.uniform(-2.0, 2.0) * self.cur_run
            y_off += math.sin(self.anim_phase * 1.5) * 3.0 * self.cur_run

        with canvas(self.device) as draw:
            self._draw_eye(draw, 40 + x_off, 32 + y_off, 30)
            self._draw_eye(draw, 88 + x_off, 32 + y_off, 30)

    def _rounded_rect(self, draw, x1, y1, x2, y2, r, fill=None, outline=None):
        """Filled rounded rectangle — safe for any r value."""
        r = max(0, min(int(r), (x2 - x1) // 2, (y2 - y1) // 2))
        if r == 0:
            draw.rectangle([x1, y1, x2, y2], fill=fill, outline=outline)
            return
        draw.rectangle([x1 + r, y1,     x2 - r, y2    ], fill=fill)
        draw.rectangle([x1,     y1 + r, x2,     y2 - r], fill=fill)
        draw.pieslice([x1,       y1,       x1+r*2, y1+r*2], 180, 270, fill=fill)
        draw.pieslice([x2-r*2,   y1,       x2,     y1+r*2], 270, 360, fill=fill)
        draw.pieslice([x1,       y2-r*2,   x1+r*2, y2    ],  90, 180, fill=fill)
        draw.pieslice([x2-r*2,   y2-r*2,   x2,     y2    ],   0,  90, fill=fill)
        if outline:
            draw.arc([x1,       y1,       x1+r*2, y1+r*2], 180, 270, fill=outline)
            draw.arc([x2-r*2,   y1,       x2,     y1+r*2], 270, 360, fill=outline)
            draw.arc([x1,       y2-r*2,   x1+r*2, y2    ],  90, 180, fill=outline)
            draw.arc([x2-r*2,   y2-r*2,   x2,     y2    ],   0,  90, fill=outline)
            draw.line([(x1+r, y1), (x2-r, y1)], fill=outline)
            draw.line([(x1+r, y2), (x2-r, y2)], fill=outline)
            draw.line([(x1, y1+r), (x1, y2-r)], fill=outline)
            draw.line([(x2, y1+r), (x2, y2-r)], fill=outline)

    def _draw_eye(self, draw, cx, cy, size):
        cx, cy = int(cx), int(cy)

        h_adj = int(size * self.eye_squash)
        w_adj = int(size * 0.1 * abs(self.eye_squash))

        ew = max(6, size + w_adj)
        eh = max(6, size - h_adj)

        x1, y1 = cx - ew // 2, cy - eh // 2
        x2, y2 = cx + ew // 2, cy + eh // 2
        cr = min(10, eh // 3)

        self._rounded_rect(draw, x1, y1, x2, y2, cr, fill="white", outline="white")

        t   = time.time() - self.start_time
        bx  = math.sin(t * 0.5) * 3
        by  = math.cos(t * 0.3) * 2
        lx  = self.cur_look * size * 0.3

        ps = max(4, size // 2)
        px = cx + int(bx + lx)
        py = cy + int(by)

        mx = max(0, ew // 2 - ps // 2 - 2)
        my = max(0, eh // 2 - ps // 2 - 2)
        px = min(max(px, cx - mx), cx + mx)
        py = min(max(py, cy - my), cy + my)

        if not self.blink_state:
            draw.ellipse(
                [px - ps // 2, py - ps // 2, px + ps // 2, py + ps // 2],
                fill="black", outline="black"
            )
        else:
            draw.line([(x1 + cr, cy), (x2 - cr, cy)], fill="black", width=4)
def show_status_screen(self):
    """
    Display the idle/status screen before shutting down.
    """
    if not self._display_ok:
        return

    draw = ImageDraw.Draw(self.image)

    draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

    draw.text((20, 2), "Bumpy Gamma", font=self.font_medium, fill=255)

    draw.text((2, 20), f"IP  : {self.get_ip_address()}", font=self.font_small, fill=255)

    draw.text((2, 35), "WiFi: Connected", font=self.font_small, fill=255)

    draw.text((2, 50), "ROS : Waiting", font=self.font_small, fill=255)

    self.device.display(self.image)

# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OledDisplayNode()
        # At startup
        open("/tmp/oled_busy", "w").close()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[oled_display_node] Fatal: {e}")
    finally:

        # In finally:
        if os.path.exists("/tmp/oled_busy"):
            os.remove("/tmp/oled_busy")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
