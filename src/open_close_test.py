#!/usr/bin/env python3
# robotiq_keepalive_open_close.py
import time
import threading
from pymodbus.client.sync import ModbusSerialClient

#PORT = "/dev/ttyUSB0"  # if using USB
PORT = "/dev/ttySC0"    # if using rs485 hat
BAUD = 115200
UNIT = 9                # default slave ID

CTRL_ADDR = 0x03E8      # write 3 regs: control block
STAT_ADDR = 0x07D0      # read status block

# Helper to pack 6 control bytes into 3 registers
def ctrl_regs(b0, b1, b2, b3, b4, b5):
    return [(b0 << 8) | b1, (b2 << 8) | b3, (b4 << 8) | b5]

def write_ctrl(client, vals, label):
    r = client.write_registers(CTRL_ADDR, vals, unit=UNIT)
    ok = (not r.isError())
    print(f"{label}: {'OK' if ok else r}")
    return ok

def read_status(client, count=4):
    r = client.read_holding_registers(STAT_ADDR, count, unit=UNIT)
    if r.isError(): return None
    return r.registers

def keepalive_loop(client, stop_event):
    # 10 Hz keepalive by reading some status
    while not stop_event.is_set():
        read_status(client, count=2)  # any valid FC03 keeps it alive
        time.sleep(0.1)

def main():
    client = ModbusSerialClient(
        method="rtu", port=PORT, baudrate=BAUD,
        bytesize=8, parity="N", stopbits=1, timeout=0.3
    )
    if not client.connect():
        print(f"❌ Could not open {PORT}")
        return

    # Start keepalive thread
    stop_evt = threading.Event()
    t = threading.Thread(target=keepalive_loop, args=(client, stop_evt), daemon=True)
    t.start()

    try:
        # Optional: clear state then activate
        write_ctrl(client, ctrl_regs(0x00,0x00, 0x00,0x00, 0x00,0x00), "DEACTIVATE")
        time.sleep(0.2)

        # ACTIVATE (rACT=1, rGTO=0 initially is fine)
        # Many examples also set rGTO=1 immediately; both work.
        write_ctrl(client, ctrl_regs(0x01,0x00, 0x00,0x00, 0x50,0x50), "ACTIVATE")  # speed/force set to 0x50

        # Give it a moment to finish activation
        time.sleep(0.5)

        # CLOSE: rACT=1 & rGTO=1 (0x09), position=255, speed=255, force=255
        write_ctrl(client, ctrl_regs(0x09,0x00, 0x00,0xFF, 0xFF,0xFF), "CLOSE")
        # Keep polling while it moves
        time.sleep(2.0)

        # OPEN: rACT=1 & rGTO=1 (0x09), position=0, speed=255, force=255
        write_ctrl(client, ctrl_regs(0x09,0x00, 0x00,0x00, 0xFF,0xFF), "OPEN")
        time.sleep(2.0)

        # Read a few status regs so you can see them
        regs = read_status(client, count=6)
        if regs is not None:
            print("Status regs:", regs)

    finally:
        stop_evt.set()
        t.join(timeout=0.5)
        client.close()

if __name__ == "__main__":
    main()

