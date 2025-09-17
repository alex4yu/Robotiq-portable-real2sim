# joystick_m5_js2_min.py
from smbus2 import SMBus, i2c_msg
import time

BUS = 1
ADDR = 0x63

def reg_read(reg, n):
    with SMBus(BUS) as bus:
        w = i2c_msg.write(ADDR, [reg])
        r = i2c_msg.read(ADDR, n)
        bus.i2c_rdwr(w, r)
        return list(r)

def u16_le(lo, hi): return lo | (hi << 8)
def s16_le(lo, hi):
    v = u16_le(lo, hi)
    return v - 65536 if v & 0x8000 else v

while True:
    # Option A: centered Int12 values (recommended)
    xL,xH,yL,yH = reg_read(0x50, 4)         # -4095..4095
    x = s16_le(xL, xH)
    y = s16_le(yL, yH)

    # Option B (fallback): raw 0..65535
    # xL,xH,yL,yH = reg_read(0x00, 4)       # 0..65535
    # x = u16_le(xL, xH) - 32768
    # y = u16_le(yL, yH) - 32768

    btn = reg_read(0x20, 1)[0]              # 1 = no press, 0 = press (per docs)
    pressed = (btn == 0)

    # Direction (simple threshold)
    th = 600     # tweak to taste if using Int12; try ~500–1000
    if abs(x) < th and abs(y) < th:
        direction = "CENTER"
    elif abs(x) >= abs(y):
        direction = "RIGHT" if x > 0 else "LEFT"
    else:
        direction = "UP" if y > 0 else "DOWN"

    print(f"X={x:6d}  Y={y:6d}  Dir={direction:6s}  BTN={'PRESSED' if pressed else 'released'}")
    time.sleep(0.05)

