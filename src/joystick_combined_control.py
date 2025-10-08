#!/usr/bin/env python3
"""
Combined joystick controller for Robotiq gripper and RealSense recording.

Controls:
- Joystick Y-axis: Pull DOWN -> gripper CLOSE, release -> OPEN
- Joystick button: Press to START/STOP recording
- LED shows RED when not recording, GREEN when recording (BGR format)

Dependencies:
  pip install pyrealsense2 smbus2 pymodbus
"""

import os
import time
import threading
import signal
import sys
from datetime import datetime

from smbus2 import SMBus, i2c_msg
import pyrealsense2 as rs

try:
    from pymodbus.client import ModbusSerialClient          # pymodbus >=3
except Exception:
    from pymodbus.client.sync import ModbusSerialClient      # pymodbus 2.x

# ========= RS-485 / Robotiq =========
PORT = "/dev/ttySC0"         # e.g. "/dev/ttyUSB0" if using a USB RS-485 dongle
BAUD = 115200
UNIT = 9
CTRL_ADDR = 0x03E8
STAT_ADDR = 0x07D0

# ========= Joystick (M5 Joystick2 @ 0x63) =========
I2C_BUS = 1
JOY_ADDR = 0x63
REG_XY_INT12 = 0x50          # returns 4 bytes: xL,xH,yL,yH as signed centered values (-4095..4095)
BTN_REG = 0x20               # 1 = not pressed, 0 = pressed
POLL_HZ = 100                 # joystick polling rate

# ========= Teleop behavior =========
# Joystick "down" usually makes Y NEGATIVE on many sticks. If yours is opposite, set INVERT_Y = True.
INVERT_Y = False

# Hysteresis thresholds (on INT12 scale). More negative = more "down".
TH_DOWN_ENTER = -900         # go to CLOSE when Y < this
TH_DOWN_EXIT  = -400         # leave CLOSE when Y > this  (prevents chatter near center)

CLOSE_SPEED = 0xFF           # 0..255 (0x00..0xFF) Robotiq speed
CLOSE_FORCE = 0xFF           # 0..255 force
OPEN_SPEED  = 0xFF
OPEN_FORCE  = 0xFF

# ========= Recording paths =========
HOME = os.path.expanduser("~")
RECORD_DIR = os.path.join(HOME, "robotiq_ws", "recordings")

# ========= Helpers =========
def ctrl_regs(b0,b1,b2,b3,b4,b5):
    return [(b0<<8)|b1, (b2<<8)|b3, (b4<<8)|b5]

class SafeModbusClient:
    def __init__(self, port, baud, unit):
        self.client = ModbusSerialClient(method="rtu", port=port, baudrate=baud,
                                         bytesize=8, parity="N", stopbits=1, timeout=0.3)
        self.unit = unit
        self.lock = threading.Lock()
    def connect(self):
        ok = self.client.connect()
        print("[MODBUS] connect ->", ok)
        return ok
    def write_ctrl(self, vals, label):
        with self.lock:
            print(f"[MODBUS] {label} -> {vals}")
            r = self.client.write_registers(CTRL_ADDR, vals, unit=self.unit)
        ok = (getattr(r, "isError", lambda: False)() is False)
        print(f"[MODBUS] {label} result -> {'OK' if ok else r}")
        return ok
    def read_status(self, count=2):
        with self.lock:
            r = self.client.read_holding_registers(STAT_ADDR, count, unit=self.unit)
        if getattr(r, "isError", lambda: False)():
            return None
        return r.registers
    def close(self):
        with self.lock:
            self.client.close()
            print("[MODBUS] close()")

def activate(safe):
    # Full activation with required calibration sequence
    print("[INFO] Deactivating gripper...")
    safe.write_ctrl(ctrl_regs(0,0, 0,0, 0,0), "DEACTIVATE"); time.sleep(0.2)
    
    print("[INFO] Activating gripper with calibration...")
    # Activate with rACT=1, rGTO=1, position=0 (open), speed/force for calibration
    safe.write_ctrl(ctrl_regs(0x09,0, 0,0x00, 0x50,0x50), "ACTIVATE_OPEN"); time.sleep(1.0)
    
    print("[INFO] Calibration close...")
    # Close for calibration
    safe.write_ctrl(ctrl_regs(0x09,0, 0,0xFF, 0x50,0x50), "CALIB_CLOSE"); time.sleep(1.0)
    
    print("[INFO] Calibration open...")
    # Open for calibration
    safe.write_ctrl(ctrl_regs(0x09,0, 0,0x00, 0x50,0x50), "CALIB_OPEN"); time.sleep(1.0)
    
    print("[INFO] Gripper activated and calibrated - ready for joystick control")

def cmd_close(safe, speed=CLOSE_SPEED, force=CLOSE_FORCE):
    return safe.write_ctrl(ctrl_regs(0x09,0, 0,0xFF, speed, force), "CLOSE")

def cmd_open(safe, speed=OPEN_SPEED, force=OPEN_FORCE):
    return safe.write_ctrl(ctrl_regs(0x09,0, 0,0x00, speed, force), "OPEN")

def keepalive(safe, stop_evt):
    while not stop_evt.is_set():
        safe.read_status(2)
        time.sleep(0.1)

# ========= Recording helpers =========
def ensure_record_dir():
    os.makedirs(RECORD_DIR, exist_ok=True)

def next_bag_path():
    # NOTE: user asked for YYYY-MM-DD-HH.SS.bag (hours.seconds)
    # If you intended HH.MM.SS, change strftime to "%Y-%m-%d-%H.%M.%S"
    fname = datetime.now().strftime("%Y-%m-%d-%H.%M.%S") + ".bag"
    return os.path.join(RECORD_DIR, fname)

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

# ---- Joystick read (M5 Joystick2) ----
def reg_read(addr, reg, n):
    with SMBus(I2C_BUS) as bus:
        w = i2c_msg.write(addr, [reg])
        r = i2c_msg.read(addr, n)
        bus.i2c_rdwr(w, r)
        return list(r)

def s16_le(lo, hi):
    v = lo | (hi << 8)
    return v - 65536 if v & 0x8000 else v

def read_xy_int12():
    # -4095..4095 centered values
    xL,xH,yL,yH = reg_read(JOY_ADDR, REG_XY_INT12, 4)
    x = s16_le(xL, xH)
    y = s16_le(yL, yH)
    if INVERT_Y:
        y = -y
    return x, y

def read_button_pressed() -> bool:
    try:
        val = reg_read(JOY_ADDR, BTN_REG, 1)[0]
        return (val == 0)
    except Exception as e:
        # If I2C hiccups, treat as not pressed, but print once
        print(f"[WARN] I2C read failed: {e}")
        time.sleep(0.1)
        return False

def reg_write(addr, reg, data):
    """Write data to I2C register"""
    try:
        with SMBus(I2C_BUS) as bus:
            bus.write_byte_data(addr, reg, data)
        return True
    except Exception as e:
        print(f"[WARN] I2C write failed: {e}")
        return False

def set_led_color(r, g, b):
    """Set LED color using BGR format (Blue-Green-Red)"""
    try:
        # Use BGR format for M5Stack JS2 joystick
        reg_write(JOY_ADDR, 0x30, b)  # Blue component
        reg_write(JOY_ADDR, 0x31, g)  # Green component  
        reg_write(JOY_ADDR, 0x32, r)  # Red component
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

def main():
    # Initialize recording directory
    ensure_record_dir()
    
    # Initialize Modbus client
    safe = SafeModbusClient(PORT, BAUD, UNIT)
    if not safe.connect():
        print("❌ Could not open Modbus port")
        return

    # Initialize recorder
    rec = Recorder()

    # Clean shutdown on Ctrl+C
    def handle_sigint(sig, frame):
        print("\n[INFO] Ctrl+C received.")
        if rec.is_recording:
            rec.stop()
        cmd_open(safe)  # Open gripper on exit
        # Turn off LED on shutdown
        set_led_color(0, 0, 0)  # Turn off LED
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    # Start keepalive thread
    stop_evt = threading.Event()
    threading.Thread(target=keepalive, args=(safe, stop_evt), daemon=True).start()

    # Init gripper with required calibration
    activate(safe)

    # Teleop state machine
    state = "OPEN"   # "OPEN" or "CLOSE"
    print("[READY] Combined joystick control:")
    print("        - Pull joystick DOWN to CLOSE gripper, release to OPEN")
    print("        - Press joystick button to START/STOP recording")
    print("        - Ctrl-C to exit")
    print(f"[INFO ] Hysteresis: enter_close={TH_DOWN_ENTER}, exit_close={TH_DOWN_EXIT}, invert_y={INVERT_Y}")
    print(f"[PATH ] Recordings -> {RECORD_DIR}")
    print("[LED  ] RED = Not recording, GREEN = Recording")
    
    # Initialize LED to RED (not recording)
    set_led_recording_state(False)

    # Button debouncing
    prev_pressed = False
    debounce_ms = 150
    last_edge_time = 0.0

    try:
        period = 1.0 / POLL_HZ
        t_next = time.time()
        while True:
            x, y = read_xy_int12()
            pressed = read_button_pressed()
            now = time.time()

            # Gripper control with hysteresis logic:
            if state == "OPEN":
                if y < TH_DOWN_ENTER:
                    state = "CLOSE"
                    cmd_close(safe)
                    print(f"[JOY ] y={y:5d} -> CLOSE")
            else:  # state == "CLOSE"
                if y > TH_DOWN_EXIT:
                    state = "OPEN"
                    cmd_open(safe)
                    print(f"[JOY ] y={y:5d} -> OPEN")

            # Recording toggle with debouncing
            if pressed and not prev_pressed and (now - last_edge_time) * 1000.0 > debounce_ms:
                last_edge_time = now
                if not rec.is_recording:
                    path = next_bag_path()
                    rec.start(path)
                else:
                    rec.stop()

            prev_pressed = pressed

            # Service recording pipeline
            rec.spin_once(timeout_s=0.01)

            # Optional debug print (comment out if noisy)
            # print(f"[DBG ] x={x:5d} y={y:5d} state={state} btn={pressed} rec={rec.is_recording}")

            # pace loop
            t_next += period
            dt = t_next - time.time()
            if dt > 0:
                time.sleep(dt)
            else:
                t_next = time.time()
    except KeyboardInterrupt:
        print("\n[EXIT] Opening gripper then quitting...")
        cmd_open(safe)
    finally:
        if rec.is_recording:
            rec.stop()
        stop_evt.set()
        time.sleep(0.2)
        # Turn off LED on exit
        set_led_color(0, 0, 0)  # Turn off LED
        safe.close()

if __name__ == "__main__":
    main()
