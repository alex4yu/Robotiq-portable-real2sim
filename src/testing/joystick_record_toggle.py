#!/usr/bin/env python3
"""
Toggle RealSense recording with joystick button presses.

- First press  -> START recording to ~/robotiq_ws/recordings/YYYY-MM-DD-HH.SS.bag
- Second press -> STOP recording
- Holding the button does NOT stop; only the next press toggles it off.
- LED shows RED when not recording, GREEN when recording (BGR format)

Deps:
  pip install pyrealsense2 smbus2
"""

import os
import time
import signal
import sys
from datetime import datetime

from smbus2 import SMBus, i2c_msg
import pyrealsense2 as rs

# ---------------- Joystick (M5) config ----------------
BUS = 1
ADDR = 0x63
BTN_REG = 0x20          # 1 = not pressed, 0 = pressed (per your docs)

def reg_read(reg, n):
    with SMBus(BUS) as bus:
        w = i2c_msg.write(ADDR, [reg])
        r = i2c_msg.read(ADDR, n)
        bus.i2c_rdwr(w, r)
        return list(r)

def read_button_pressed() -> bool:
    try:
        val = reg_read(BTN_REG, 1)[0]
        return (val == 0)
    except Exception as e:
        # If I2C hiccups, treat as not pressed, but print once
        print(f"[WARN] I2C read failed: {e}")
        time.sleep(0.1)
        return False

def reg_write(reg, data):
    """Write data to I2C register"""
    try:
        with SMBus(BUS) as bus:
            bus.write_byte_data(ADDR, reg, data)
        return True
    except Exception as e:
        print(f"[WARN] I2C write failed: {e}")
        return False

def set_led_color(r, g, b):
    """Set LED color using BGR format (Blue-Green-Red)"""
    try:
        # Use BGR format for M5Stack JS2 joystick
        reg_write(0x30, b)  # Blue component
        reg_write(0x31, g)  # Green component  
        reg_write(0x32, r)  # Red component
        return True
    except Exception as e:
        print(f"[WARN] LED control failed: {e}")
        return False

def set_led_recording_state(is_recording):
    """Set LED based on recording state"""
    if is_recording:
        # Recording - show GREEN
        set_led_color(0, 255, 0)  # Green (BGR format)
    else:
        # Not recording - show RED
        set_led_color(255, 0, 0)  # Red (BGR format)

# ---------------- Recording paths ----------------
HOME = os.path.expanduser("~")
RECORD_DIR = os.path.join(HOME, "robotiq_ws", "recordings")

def ensure_record_dir():
    os.makedirs(RECORD_DIR, exist_ok=True)

def next_bag_path():
    # NOTE: user asked for YYYY-MM-DD-HH.SS.bag (hours.seconds)
    # If you intended HH.MM.SS, change strftime to "%Y-%m-%d-%H.%M.%S"
    fname = datetime.now().strftime("%Y-%m-%d-%H.%M.%S") + ".bag"
    return os.path.join(RECORD_DIR, fname)

# ---------------- RealSense helpers ----------------
class Recorder:
    def __init__(self):
        self.pipe = None
        self.cfg = None
        self.is_recording = False
        self.current_path = None

    def start(self, bag_path: str,
              depth=(640, 480, 15),
              color=(640, 480, 15),
              serial: str = None):
        if self.is_recording:
            print("[INFO] Already recording.")
            return

        self.pipe = rs.pipeline()
        self.cfg = rs.config()

        if serial:
            self.cfg.enable_device(serial)

        # Enable streams
        self.cfg.enable_stream(rs.stream.depth, depth[0], depth[1], rs.format.z16, depth[2])
        self.cfg.enable_stream(rs.stream.color, color[0], color[1], rs.format.rgb8, color[2])

        # Direct SDK recording to file
        self.cfg.enable_record_to_file(bag_path)

        print(f"[INFO] Starting recording to: {bag_path}")
        self.pipe.start(self.cfg)
        self.current_path = bag_path
        self.is_recording = True
        
        # Set LED to GREEN (recording)
        set_led_recording_state(True)

    def stop(self):
        if not self.is_recording:
            print("[INFO] Not recording.")
            return

        print("[INFO] Stopping recording…")
        try:
            self.pipe.stop()
        except Exception as e:
            print(f"[WARN] pipeline stop error: {e}")
        finally:
            self.pipe = None
            self.cfg = None
            self.is_recording = False
            print(f"[OK] Saved: {self.current_path}")
            self.current_path = None
            
            # Set LED to RED (not recording)
            set_led_recording_state(False)

    def spin_once(self, timeout_s=0.05):
        """Keep pipeline serviced while recording; not strictly required,
        but helps the SDK push frames promptly."""
        if not self.is_recording:
            time.sleep(timeout_s)
            return
        try:
            # Non-blocking-ish wait
            self.pipe.poll_for_frames()
        except Exception:
            pass
        time.sleep(timeout_s)

# ---------------- Main loop ----------------
def main():
    ensure_record_dir()
    rec = Recorder()

    # Clean shutdown on Ctrl+C
    def handle_sigint(sig, frame):
        print("\n[INFO] Ctrl+C received.")
        if rec.is_recording:
            rec.stop()
        # Turn off LED on shutdown
        set_led_color(0, 0, 0)  # Turn off LED
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    print("[READY] Press joystick button to START/STOP recording.")
    print(f"[PATH ] Recordings -> {RECORD_DIR}")
    print("[HINT ] If you have multiple cameras, plug in one to start. "
          "We can add per-serial control later.")
    print("[LED  ] RED = Not recording, GREEN = Recording")
    
    # Initialize LED to RED (not recording)
    set_led_recording_state(False)

    prev_pressed = False
    debounce_ms = 150
    last_edge_time = 0.0

    while True:
        pressed = read_button_pressed()
        now = time.time()

        # Rising edge detection with simple debounce
        if pressed and not prev_pressed and (now - last_edge_time) * 1000.0 > debounce_ms:
            last_edge_time = now
            if not rec.is_recording:
                path = next_bag_path()
                rec.start(path)
            else:
                rec.stop()

        prev_pressed = pressed
        rec.spin_once(timeout_s=0.03)  # service pipeline / keep loop responsive

if __name__ == "__main__":
    main()

