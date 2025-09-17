#!/usr/bin/env python3
# robotiq_space_teleop_pynput.py
# Hold SPACE to close; release SPACE to open. ESC = open then quit.
import time
import threading
from pymodbus.client.sync import ModbusSerialClient
from pynput import keyboard

# ======== Serial / Modbus settings (match your working test) ========
#PORT = "/dev/ttyUSB0"   # if using USB adapter
PORT = "/dev/ttySC0"      # RS-485 HAT on Raspberry Pi
BAUD = 115200
UNIT = 9                  # Slave ID
CTRL_ADDR = 0x03E8        # 1000: Robot Output (write) block (3 regs)
STAT_ADDR = 0x07D0        # 2000: Robot Input (read) block
# ====================================================================

# Pack 6 control bytes into 3 registers (exactly your order)
def ctrl_regs(b0, b1, b2, b3, b4, b5):
    # Reg1000: (b0<<8)|b1  -> [ACTION, 0x00]
    # Reg1001: (b2<<8)|b3  -> [0x00, POSITION]
    # Reg1002: (b4<<8)|b5  -> [SPEED, FORCE]
    return [(b0 << 8) | b1, (b2 << 8) | b3, (b4 << 8) | b5]

class SafeModbusClient:
    """Serialize Modbus access so reads/writes never collide."""
    def __init__(self, port, baud, unit):
        self.client = ModbusSerialClient(
            method="rtu", port=port, baudrate=baud,
            bytesize=8, parity="N", stopbits=1, timeout=0.3
        )
        self.unit = unit
        self.lock = threading.Lock()

    def connect(self):
        return self.client.connect()

    def write_ctrl(self, vals, label="WRITE"):
        with self.lock:
            r = self.client.write_registers(CTRL_ADDR, vals, unit=self.unit)
        ok = (not r.isError())
        print(f"{label}: {'OK' if ok else r}")
        return ok

    def read_status(self, count=2):
        with self.lock:
            r = self.client.read_holding_registers(STAT_ADDR, count, unit=self.unit)
        if r.isError():
            return None
        return r.registers

    def close(self):
        with self.lock:
            self.client.close()

def keepalive_loop(safe, stop_event):
    """Optional: 10 Hz read to keep link alive; safe due to lock."""
    while not stop_event.is_set():
        safe.read_status(count=2)
        time.sleep(0.1)

def activate_sequence(safe):
    # 1) DEACTIVATE (clear)
    safe.write_ctrl(ctrl_regs(0x00,0x00, 0x00,0x00, 0x00,0x00), "DEACTIVATE")
    time.sleep(0.2)
    # 2) ACTIVATE with rACT=1 **ONLY** (NO rGTO here)
    #    -> ACTION = 0x01; speed/force preset is fine (won't move yet)
    safe.write_ctrl(ctrl_regs(0x01,0x00, 0x00,0x00, 0x50,0x50), "ACTIVATE")
    time.sleep(0.5)

def cmd_close(safe, speed=0xFF, force=0xFF):
    # ACTION = rACT|rGTO = 0x01|0x08 = 0x09, POSITION=0xFF
    return safe.write_ctrl(ctrl_regs(0x09,0x00, 0x00,0xFF, speed,force), "CLOSE")

def cmd_open(safe, speed=0xFF, force=0xFF):
    # ACTION = 0x09, POSITION=0x00
    return safe.write_ctrl(ctrl_regs(0x09,0x00, 0x00,0x00, speed,force), "OPEN")

def main():
    safe = SafeModbusClient(PORT, BAUD, UNIT)
    if not safe.connect():
        print(f"❌ Could not open {PORT}")
        return

    # Optional: keepalive reader (can omit if not needed)
    stop_evt = threading.Event()
    t = threading.Thread(target=keepalive_loop, args=(safe, stop_evt), daemon=True)
    t.start()

    activate_sequence(safe)
    # Start from open for safety
    cmd_open(safe)

    state = {"space_down": False}

    print("[READY] Hold SPACE to CLOSE; release SPACE to OPEN. Press ESC to exit.")

    def on_press(key):
        print(f"[KEYPRESS] {key}")  # debug
        try:
            if key == keyboard.Key.space and not state["space_down"]:
                print("[DEBUG] SPACE down detected")
                state["space_down"] = True
                cmd_close(safe)
        except Exception as e:
            print(f"[ERROR] on_press: {e}")

    def on_release(key):
        print(f"[KEYRELEASE] {key}")  # debug
        try:
            if key == keyboard.Key.space and state["space_down"]:
                print("[DEBUG] SPACE released")
                state["space_down"] = False
                cmd_open(safe)
            elif key == keyboard.Key.esc:
                print("[DEBUG] ESC released")
                # Open before exit so you don't leave it clamped
                cmd_open(safe)
                print("[EXIT] ESC -> OPEN then quit.")
                return False
        except Exception as e:
            print(f"[ERROR] on_release: {e}")

    try:
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    finally:
        stop_evt.set()
        t.join(timeout=0.5)
        safe.close()

if __name__ == "__main__":
    main()

