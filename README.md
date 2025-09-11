# Robotiq 2F-85 Gripper Control on Raspberry Pi

This repository contains Python scripts and setup notes for controlling a **Robotiq 2F-85** gripper over RS-485 using a Raspberry Pi.  
Two connection methods are supported:
- **USB–RS485 converter** (quick sanity check)
- **Waveshare RS485 CAN HAT (B)** (SPI → dual UART chip)

---

## 1. Hardware Needed

- **Robotiq 2F-85** gripper  
- **24 V DC power supply** for the gripper  
- **Raspberry Pi 5**   
- **RS-485 interface**  
  - *Option A:* USB–RS485 converter (e.g. CH340, FTDI, etc.)  
  - *Option B:* Waveshare RS485 CAN HAT (B)  
- **Cables**: 3-wire cable from gripper (current setup: White, Green, Bare = GND) + power leads for 24 V

---

## 2. Wiring

### Robotiq cable pinout
- **White = 485+ (non-inverting / D+) connect to terminal A on convertor/HAT**  
- **Green = 485– (inverting / D–) connect to terminal B on converto/HAT**  
- **Bare = RS-485 GND**  

The gripper requires separate 24 VDC power. The USB converter or HAT does not provide power.

### USB–RS485 Converter
- White → **A**  
- Green → **B**  
- Bare → **GND**  
- Also connect **24 V + and GND** to the gripper’s power pins.  

### RS485 CAN HAT (B) — RS485_0 terminal
- White → **A**  
- Green → **B**  
- Bare → **GND**  

---

## 3. HAT Setup (Raspberry Pi OS Bookworm)

The HAT uses an **SC16IS752 dual UART (via SPI)** for RS-485.

1. Edit `/boot/firmware/config.txt` add this to the bottom:

   dtparam=spi=on
   dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25,spimaxfrequency=1000000
   dtoverlay=sc16is752-spi1,int_pin=24
2. Reboot

Run in terminal:

    sudo reboot

---

3. Verify devices

After reboot, check for new UART devices:

    ls /dev/ttySC*

Expected output:

    /dev/ttySC0  /dev/ttySC1

Use /dev/ttySC0 for the RS485_0 terminal block.

Note: You do not need wiringPi, bcm2835, or lgpio for this workflow. The kernel driver exposes /dev/ttySC* directly.

---

4. Software Setup

a. Go to your workspace:

       cd robotiq_ws

b. Create and activate a Python virtual environment:

       python3 -m venv venv
       source venv/bin/activate

c. Install dependencies (listed in requirements.txt):

       pip install -r requirements.txt

requirements.txt should include:

    pyserial
    pymodbus

The requirements may need to be updated
---

5. Running the Test Script

A) USB–RS485

A1. Find the USB device:

       sudo dmesg | grep ttyUSB

For example, you might see /dev/ttyUSB0.

A2. Open src/robotiq_keepalive_open_close.py and set:

    PORT = "/dev/ttyUSB0"

A3. Run the script:

       python src/robotiq_keepalive_open_close.py

B) RS485 CAN HAT (recommended)

B1. Confirm /dev/ttySC0 exists.
B2. Open src/robotiq_keepalive_open_close.py and set:

    PORT = "/dev/ttySC0"

B3. Run the script:

       python src/robotiq_keepalive_open_close.py

---

6. Expected Behavior

- On power-up: gripper LED = red (waiting for communication)
- After activation: LED = blue
- The script keeps polling status (prevents timeout faults)
- Gripper should close then open

---

7. Troubleshooting

- LED turns blue then red again → keepalive missing; ensure script is running.
- No response → swap A/B wires, confirm 24 V supply, check grounds.
- Permission denied on /dev/tty* → add your user to the dialout group:

      sudo usermod -aG dialout $USER

- No /dev/ttySC* devices → double-check overlays in /boot/firmware/config.txt.

