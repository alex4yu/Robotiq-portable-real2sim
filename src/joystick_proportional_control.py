#!/usr/bin/env python3
# joystick_proportional_control.py
# Proportional force and speed control for Robotiq gripper via joystick
# - Joystick UP (variable) -> OPEN with speed proportional to push distance
#   * Light push = slow opening (precise)
#   * Hard push = fast opening
# - Joystick DOWN (variable) -> CLOSE with force AND speed proportional to pull distance
#   * Light pull = slow and gentle (precise)
#   * Hard pull = fast and strong (quick grasp)
# - Centered -> HOLD current position

import time
import threading
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
POLL_HZ = 50                 # joystick polling rate (reduced from 100Hz to avoid overloading Modbus)

# ========= Control Configuration =========
# Joystick behavior - adjust if your joystick is inverted
INVERT_Y = False
INVERT_X = False

# Deadzone - no action when joystick is centered within this radius
DEADZONE_RADIUS = 200

# Y-axis thresholds for OPEN and CLOSE (up/down control)
Y_OPEN_THRESHOLD = 300       # Trigger open when Y > this (positive = UP)
Y_MAX_PUSH = 3000            # Maximum expected Y push for speed mapping (UP)
Y_CLOSE_THRESHOLD = -300     # Trigger close when Y < this (negative = DOWN)
Y_MAX_PULL = -3000           # Maximum expected Y pull for speed/force mapping (DOWN)

# Speed settings (proportional control)
MIN_OPEN_SPEED = 0x10        # Minimum speed when just crossing threshold (very slow, precise)
MAX_OPEN_SPEED = 0xFF        # Maximum speed when fully pushed up (fast)
MIN_CLOSE_SPEED = 0x10       # Minimum speed when just crossing threshold (very slow, precise)
MAX_CLOSE_SPEED = 0xFF       # Maximum speed when fully pulled down (fast)

# Force settings
OPEN_FORCE = 0xFF            # Fixed force for opening (use max for reliable opening)
MIN_CLOSE_FORCE = 0x10       # Minimum force when just crossing threshold (very gentle)
MAX_CLOSE_FORCE = 0xFF       # Maximum force when fully pulled down (strong)

# Command rate limiting
MIN_COMMAND_INTERVAL = 0.05  # Minimum time between commands (50ms = 20Hz max)

# ========= Helpers =========
def ctrl_regs(b0, b1, b2, b3, b4, b5):
    """Convert 6 bytes into 3 Modbus registers (16-bit each)"""
    return [(b0 << 8) | b1, (b2 << 8) | b3, (b4 << 8) | b5]


def map_range(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another, with clamping"""
    # Clamp input
    value = max(in_min, min(in_max, value))
    # Linear mapping
    result = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    # Clamp output
    return int(max(out_min, min(out_max, result)))


class SafeModbusClient:
    """Thread-safe Modbus client wrapper"""
    def __init__(self, port, baud, unit):
        self.client = ModbusSerialClient(
            method="rtu", port=port, baudrate=baud,
            bytesize=8, parity="N", stopbits=1, timeout=0.3
        )
        self.unit = unit
        self.lock = threading.Lock()
        
    def connect(self):
        ok = self.client.connect()
        print(f"[MODBUS] connect -> {ok}")
        return ok
    
    def write_ctrl(self, vals, label=""):
        with self.lock:
            if label:
                print(f"[MODBUS] {label} -> {vals}")
            r = self.client.write_registers(CTRL_ADDR, vals, unit=self.unit)
        ok = (getattr(r, "isError", lambda: False)() is False)
        if label:
            print(f"[MODBUS] {label} result -> {'OK' if ok else r}")
        return ok
    
    def read_status(self, count=3):
        """Read status registers (at least 3 for position feedback)"""
        with self.lock:
            r = self.client.read_holding_registers(STAT_ADDR, count, unit=self.unit)
        if getattr(r, "isError", lambda: False)():
            return None
        return r.registers
    
    def close(self):
        with self.lock:
            self.client.close()
            print("[MODBUS] close()")


def activate_gripper(safe):
    """Activate the gripper - must be done before any motion"""
    print("[INIT] Activating gripper...")
    # Clear activation
    safe.write_ctrl(ctrl_regs(0x00, 0x00, 0x00, 0x00, 0x00, 0x00), "DEACTIVATE")
    time.sleep(0.2)
    
    # Set activation bit (rACT=1)
    safe.write_ctrl(ctrl_regs(0x01, 0x00, 0x00, 0x00, 0x00, 0x00), "ACTIVATE")
    time.sleep(0.5)
    
    # Wait for activation to complete (gSTA should become 0x03)
    print("[INIT] Waiting for activation to complete...")
    for _ in range(20):  # Wait up to 2 seconds
        status = safe.read_status(1)
        if status:
            gripper_status = status[0]
            gSTA = (gripper_status >> 4) & 0x03
            if gSTA == 0x03:
                print("[INIT] Gripper activated successfully (gSTA=0x03)")
                return True
        time.sleep(0.1)
    
    print("[WARN] Activation may not be complete, but proceeding...")
    return True


def cmd_grip(safe, position, speed, force, label=""):
    """
    Send a grip command with specific position, speed, and force
    
    Args:
        position: 0x00 (open) to 0xFF (closed)
        speed: 0x00 (slow) to 0xFF (fast)
        force: 0x00 (gentle) to 0xFF (strong)
    """
    # Byte 0: rACT=1, rGTO=1 (activate and go to position)
    byte0 = 0x09  # 0b00001001 = rACT | rGTO
    byte1 = 0x00  # Reserved
    byte2 = 0x00  # Reserved
    byte3 = position & 0xFF
    byte4 = speed & 0xFF
    byte5 = force & 0xFF
    
    return safe.write_ctrl(ctrl_regs(byte0, byte1, byte2, byte3, byte4, byte5), label)


def cmd_stop(safe, label=""):
    """
    Send a STOP command - gripper stops and holds current position
    Sets rACT=1 (stay activated) but rGTO=0 (stop motion)
    """
    # Byte 0: rACT=1, rGTO=0 (activated but not going to position = STOP)
    byte0 = 0x01  # 0b00000001 = rACT only
    byte1 = 0x00  # Reserved
    byte2 = 0x00  # Reserved
    byte3 = 0x00  # Position doesn't matter when rGTO=0
    byte4 = 0x00  # Speed doesn't matter when rGTO=0
    byte5 = 0x00  # Force doesn't matter when rGTO=0
    
    return safe.write_ctrl(ctrl_regs(byte0, byte1, byte2, byte3, byte4, byte5), label)


def keepalive(safe, stop_evt):
    """Background thread to keep connection alive and read status"""
    while not stop_evt.is_set():
        safe.read_status(3)
        time.sleep(0.1)


# ---- Joystick reading ----
def reg_read(addr, reg, n):
    """Read n bytes from I2C register"""
    with SMBus(I2C_BUS) as bus:
        w = i2c_msg.write(addr, [reg])
        r = i2c_msg.read(addr, n)
        bus.i2c_rdwr(w, r)
        return list(r)


def s16_le(lo, hi):
    """Convert little-endian bytes to signed 16-bit integer"""
    v = lo | (hi << 8)
    return v - 65536 if v & 0x8000 else v


def read_xy_int12():
    """Read joystick X,Y values in INT12 format (-4095 to 4095)"""
    xL, xH, yL, yH = reg_read(JOY_ADDR, REG_XY_INT12, 4)
    x = s16_le(xL, xH)
    y = s16_le(yL, yH)
    
    if INVERT_Y:
        y = -y
    if INVERT_X:
        x = -x
    
    return x, y


def main():
    print("=" * 60)
    print("Robotiq Gripper - Proportional Force & Speed Control")
    print("=" * 60)
    print("Controls:")
    print("  UP (hold):         Gripper opens while held")
    print("                     - Push further = faster opening")
    print("                     - Light push = slow (precise control)")
    print("  DOWN (hold):       Gripper closes while held")
    print("                     - Pull further = faster + stronger grip")
    print("                     - Light pull = slow + gentle (precise control)")
    print("  RELEASE (center):  Gripper STOPS and holds position")
    print("  Ctrl-C:            Exit and open gripper")
    print("=" * 60)
    
    # Connect to gripper
    safe = SafeModbusClient(PORT, BAUD, UNIT)
    if not safe.connect():
        print("❌ Could not open Modbus port")
        return
    
    # Start keepalive thread
    stop_evt = threading.Event()
    threading.Thread(target=keepalive, args=(safe, stop_evt), daemon=True).start()
    
    # Activate gripper
    if not activate_gripper(safe):
        print("❌ Gripper activation failed")
        return
    
    print("\n[READY] Joystick control active")
    print(f"[CONFIG] Deadzone: {DEADZONE_RADIUS}, Y_open: {Y_OPEN_THRESHOLD}, Y_close: {Y_CLOSE_THRESHOLD}")
    print(f"[CONFIG] Speed range (open):  {MIN_OPEN_SPEED:#04x} to {MAX_OPEN_SPEED:#04x}")
    print(f"[CONFIG] Speed range (close): {MIN_CLOSE_SPEED:#04x} to {MAX_CLOSE_SPEED:#04x}")
    print(f"[CONFIG] Force range (close): {MIN_CLOSE_FORCE:#04x} to {MAX_CLOSE_FORCE:#04x}")
    print(f"[CONFIG] Invert - Y: {INVERT_Y}, X: {INVERT_X}\n")
    
    # Control state - track what command was last sent
    last_action = None  # "OPEN", "CLOSE", "STOP"
    last_force = None
    last_speed = None
    last_cmd_time = 0
    
    try:
        period = 1.0 / POLL_HZ
        t_next = time.time()
        
        while True:
            x, y = read_xy_int12()
            now = time.time()
            
            # Determine what action to take based on joystick position
            current_action = None
            target_force = None
            target_speed = None
            
            # Priority 1: Check for OPEN command (UP)
            if y > Y_OPEN_THRESHOLD:
                current_action = "OPEN"
                target_force = OPEN_FORCE
                
                # Proportional speed based on how far joystick is pushed up
                # Map Y from threshold to max_push -> speed from min to max
                target_speed = map_range(
                    y,
                    Y_OPEN_THRESHOLD,
                    Y_MAX_PUSH,
                    MIN_OPEN_SPEED,
                    MAX_OPEN_SPEED
                )
            
            # Priority 2: Check for CLOSE command (DOWN)
            elif y < Y_CLOSE_THRESHOLD:
                current_action = "CLOSE"
                
                # Proportional force AND speed based on how far joystick is pulled down
                # Map Y from threshold to max_pull -> force/speed from min to max
                target_force = map_range(
                    abs(y), 
                    abs(Y_CLOSE_THRESHOLD), 
                    abs(Y_MAX_PULL),
                    MIN_CLOSE_FORCE, 
                    MAX_CLOSE_FORCE
                )
                target_speed = map_range(
                    abs(y), 
                    abs(Y_CLOSE_THRESHOLD), 
                    abs(Y_MAX_PULL),
                    MIN_CLOSE_SPEED, 
                    MAX_CLOSE_SPEED
                )
            
            # Priority 3: In deadzone - STOP
            else:
                current_action = "STOP"
            
            # Decide if we need to send a command
            send_command = False
            reason = ""
            
            # Always send command if action changed
            if current_action != last_action:
                send_command = True
                reason = f"{last_action} -> {current_action}"
            
            # If opening with same action, check if speed changed significantly
            elif current_action == "OPEN" and target_speed is not None and last_speed is not None:
                if abs(target_speed - last_speed) > 5:
                    send_command = True
                    reason = f"speed: {last_speed:#04x} -> {target_speed:#04x}"
            
            # If closing with same action, check if force or speed changed significantly
            elif current_action == "CLOSE" and target_force is not None and last_force is not None:
                force_changed = abs(target_force - last_force) > 5
                speed_changed = last_speed is not None and abs(target_speed - last_speed) > 5
                if force_changed or speed_changed:
                    send_command = True
                    reason = f"force: {last_force:#04x} -> {target_force:#04x}, speed: {last_speed:#04x} -> {target_speed:#04x}"
            
            # Send command if needed and rate limit allows
            if send_command and (now - last_cmd_time) >= MIN_COMMAND_INTERVAL:
                
                if current_action == "OPEN":
                    # Send OPEN command (position 0x00 = fully open) with variable speed
                    cmd_grip(safe, 0x00, target_speed, target_force, "")
                    print(f"[CTRL] OPEN (UP) | y={y:5d} | speed={target_speed:#04x} | {reason}")
                
                elif current_action == "CLOSE":
                    # Send CLOSE command (position 0xFF = fully closed) with variable force and speed
                    cmd_grip(safe, 0xFF, target_speed, target_force, "")
                    print(f"[CTRL] CLOSE (DOWN) | y={y:5d} | speed={target_speed:#04x}, force={target_force:#04x} | {reason}")
                
                elif current_action == "STOP":
                    # Send STOP command - gripper holds current position
                    cmd_stop(safe, "")
                    print(f"[CTRL] STOP | x={x:5d}, y={y:5d} | holding position | {reason}")
                
                last_action = current_action
                last_force = target_force
                last_speed = target_speed
                last_cmd_time = now
            
            # Optional: verbose debug output (uncomment to see all joystick values)
            # if True:  # Set to True to enable debug
            #     force_str = f"{target_force:#04x}" if target_force else "N/A"
            #     speed_str = f"{target_speed:#04x}" if target_speed else "N/A"
            #     print(f"[DBG] x={x:5d} y={y:5d} action={current_action:6s} speed={speed_str} force={force_str}")
            
            # Pace loop
            t_next += period
            dt = t_next - time.time()
            if dt > 0:
                time.sleep(dt)
            else:
                t_next = time.time()
    
    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl-C detected, opening gripper and shutting down...")
        cmd_grip(safe, 0x00, MAX_OPEN_SPEED, OPEN_FORCE, "FINAL OPEN")
        time.sleep(0.5)
    
    except Exception as e:
        print(f"\n[ERROR] Exception occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        stop_evt.set()
        time.sleep(0.2)
        safe.close()
        print("[EXIT] Shutdown complete")


if __name__ == "__main__":
    main()

