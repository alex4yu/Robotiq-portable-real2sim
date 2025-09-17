#!/usr/bin/env python3
# pip install minimalmodbus pyserial
import time, minimalmodbus, serial

# --- Register addresses from FT300-S manual ---
FX_REG = 180   # Fx..Mz are 180..185 (signed 16-bit)
ACC_X_REG = 190  # 190..192 for accel

N_FORCE_SCALE   = 100.0     # Fx/Fy/Fz in N = value / 100
NM_TORQUE_SCALE = 1000.0    # Mx/My/Mz in Nm = value / 1000
G_ACC_SCALE     = 1000.0    # Accel in g = value / 1000

# --- Hard-coded config ---
PORT  = "/dev/ttySC0"
SLAVE = 9

def make_instrument():
    inst = minimalmodbus.Instrument(PORT, SLAVE)
    inst.serial.baudrate = 19200
    inst.serial.bytesize = 8
    inst.serial.parity   = serial.PARITY_NONE
    inst.serial.stopbits = 1
    inst.serial.timeout  = 0.3
    inst.mode = minimalmodbus.MODE_RTU
    return inst

def read_wrench(inst):
    regs = inst.read_registers(FX_REG, 6, functioncode=3)
    regs = [r - 65536 if r > 32767 else r for r in regs]  # signed16
    fx, fy, fz, mx, my, mz = regs
    return (fx / N_FORCE_SCALE,
            fy / N_FORCE_SCALE,
            fz / N_FORCE_SCALE,
            mx / NM_TORQUE_SCALE,
            my / NM_TORQUE_SCALE,
            mz / NM_TORQUE_SCALE)

def read_acc(inst):
    regs = inst.read_registers(ACC_X_REG, 3, functioncode=3)
    regs = [r - 65536 if r > 32767 else r for r in regs]
    return tuple(r / G_ACC_SCALE for r in regs)

if __name__ == "__main__":
    inst = make_instrument()
    while True:
        try:
            fx, fy, fz, mx, my, mz = read_wrench(inst)
            ax, ay, az = read_acc(inst)
            print(f"Fx={fx:.3f}N  Fy={fy:.3f}N  Fz={fz:.3f}N  "
                  f"Mx={mx:.4f}Nm  My={my:.4f}Nm  Mz={mz:.4f}Nm  "
                  f"|  ax={ax:.4f}g ay={ay:.4f}g az={az:.4f}g")
        except Exception as e:
            print("Read error:", e)
        time.sleep(0.1)

