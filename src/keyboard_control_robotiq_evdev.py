#!/usr/bin/env python3
import time, threading, select
from evdev import InputDevice, categorize, ecodes
try:
    from pymodbus.client import ModbusSerialClient          # pymodbus >=3
except Exception:
    from pymodbus.client.sync import ModbusSerialClient      # pymodbus 2.x

PORT = "/dev/ttySC0"    # change if needed (/dev/ttyUSB0 for USB RS485)
BAUD = 115200
UNIT = 9
CTRL_ADDR = 0x03E8
STAT_ADDR = 0x07D0
KBD = "/dev/input/by-id/usb-SONiX_USB_Keyboard-event-kbd"  # <-- set yours

def ctrl_regs(b0,b1,b2,b3,b4,b5):
    return [(b0<<8)|b1, (b2<<8)|b3, (b4<<8)|b5]

class SafeModbusClient:
    def __init__(self, port, baud, unit):
        self.client = ModbusSerialClient(method="rtu", port=port, baudrate=baud,
                                         bytesize=8, parity="N", stopbits=1, timeout=0.3)
        self.unit = unit
        self.lock = threading.Lock()
    def connect(self): 
        ok = self.client.connect(); print("[MODBUS] connect ->", ok); return ok
    def write_ctrl(self, vals, label):
        with self.lock:
            print(f"[MODBUS] {label} -> {vals}")
            r = self.client.write_registers(CTRL_ADDR, vals, unit=self.unit)
        ok = (getattr(r, "isError", lambda: False)() is False)
        print(f"[MODBUS] {label} result -> {'OK' if ok else r}"); return ok
    def read_status(self, count=2):
        with self.lock:
            r = self.client.read_holding_registers(STAT_ADDR, count, unit=self.unit)
        if getattr(r, "isError", lambda: False)(): return None
        return r.registers
    def close(self):
        with self.lock:
            self.client.close(); print("[MODBUS] close()")

def activate(safe):
    safe.write_ctrl(ctrl_regs(0,0, 0,0, 0,0), "DEACTIVATE"); time.sleep(0.2)
    safe.write_ctrl(ctrl_regs(0x01,0, 0,0, 0x50,0x50), "ACTIVATE"); time.sleep(0.5)

def cmd_close(safe, speed=0xFF, force=0xFF):
    return safe.write_ctrl(ctrl_regs(0x09,0, 0,0xFF, speed, force), "CLOSE")
def cmd_open(safe, speed=0xFF, force=0xFF):
    return safe.write_ctrl(ctrl_regs(0x09,0, 0,0x00, speed, force), "OPEN")

def keepalive(safe, stop_evt):
    while not stop_evt.is_set():
        safe.read_status(2); time.sleep(0.1)

def main():
    safe = SafeModbusClient(PORT, BAUD, UNIT)
    if not safe.connect(): print("❌ open port failed"); return
    stop_evt = threading.Event()
    threading.Thread(target=keepalive, args=(safe, stop_evt), daemon=True).start()
    activate(safe); cmd_open(safe)
    space_down = False
    dev = InputDevice(KBD)
    print("[READY] Hold SPACE to CLOSE; release SPACE to OPEN. ESC = quit.")
    try:
        while True:
            r,_,_ = select.select([dev.fd], [], [], 0.1)
            if not r: continue
            for ev in dev.read():
                if ev.type != ecodes.EV_KEY: continue
                ke = categorize(ev)
                # debug prints:
                print("[KEY]", ke.scancode, ke.keycode, "state", ke.keystate)
                if ke.scancode == ecodes.KEY_SPACE:
                    if ke.keystate == ke.key_down and not space_down:
                        space_down = True; cmd_close(safe)
                    elif ke.keystate == ke.key_up and space_down:
                        space_down = False; cmd_open(safe)
                elif ke.scancode == ecodes.KEY_ESC and ke.keystate == ke.key_down:
                    cmd_open(safe); print("[EXIT] ESC -> OPEN then quit."); return
    finally:
        stop_evt.set(); time.sleep(0.2); safe.close()

if __name__ == "__main__":
    main()

