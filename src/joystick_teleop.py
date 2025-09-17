#!/usr/bin/env python3
# joystick_robotiq_teleop.py
# Pull joystick DOWN -> gripper CLOSE; otherwise -> OPEN.

import time, threading
from smbus2 import SMBus, i2c_msg
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
    # Typical activate sequence
    safe.write_ctrl(ctrl_regs(0,0, 0,0, 0,0), "DEACTIVATE"); time.sleep(0.2)
    safe.write_ctrl(ctrl_regs(0x01,0, 0,0, 0x50,0x50), "ACTIVATE"); time.sleep(0.5)

def cmd_close(safe, speed=CLOSE_SPEED, force=CLOSE_FORCE):
    return safe.write_ctrl(ctrl_regs(0x09,0, 0,0xFF, speed, force), "CLOSE")

def cmd_open(safe, speed=OPEN_SPEED, force=OPEN_FORCE):
    return safe.write_ctrl(ctrl_regs(0x09,0, 0,0x00, speed, force), "OPEN")

def keepalive(safe, stop_evt):
    while not stop_evt.is_set():
        safe.read_status(2)
        time.sleep(0.1)

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

def main():
    safe = SafeModbusClient(PORT, BAUD, UNIT)
    if not safe.connect():
        print("❌ Could not open Modbus port")
        return

    stop_evt = threading.Event()
    threading.Thread(target=keepalive, args=(safe, stop_evt), daemon=True).start()

    # Init gripper
    activate(safe)
    cmd_open(safe)

    # Teleop state machine
    state = "OPEN"   # "OPEN" or "CLOSE"
    print("[READY] Pull joystick DOWN to CLOSE, release to OPEN. Ctrl-C to exit.")
    print(f"[INFO ] Hysteresis: enter_close={TH_DOWN_ENTER}, exit_close={TH_DOWN_EXIT}, invert_y={INVERT_Y}")

    try:
        period = 1.0 / POLL_HZ
        t_next = time.time()
        while True:
            x, y = read_xy_int12()

            # Hysteresis logic:
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

            # Optional debug print (comment out if noisy)
            # print(f"[DBG ] x={x:5d} y={y:5d} state={state}")

            # pace loop
            t_next += period
            dt = t_next - time.time()
            if dt > 0:
                time.sleep(dt)
            else:
                t_next = time.time()
    except KeyboardInterrupt:
        print("\n[EXIT] Opening then quitting...")
        cmd_open(safe)
    finally:
        stop_evt.set()
        time.sleep(0.2)
        safe.close()

if __name__ == "__main__":
    main()

