#!/usr/bin/env python3
"""
M5Stack JS2 Joystick LED Test Script
====================================

This script tests the LED control functionality of the M5Stack JS2 joystick.
It tries multiple methods to control the LED based on common M5Stack I2C protocols.

NOTE: The JS2 joystick uses BGR (Blue-Green-Red) format instead of RGB!

Features:
- Tests different LED control methods (BGR format)
- Interactive LED color testing
- Button press LED control (Red=Not pressed, Green=Pressed)
- LED pattern demonstrations
"""

import time
from smbus2 import SMBus, i2c_msg

# I2C Configuration
I2C_BUS = 1  # Default I2C bus on Raspberry Pi
JS2_ADDR = 0x63  # M5Stack JS2 Joystick I2C address

class JS2LEDTester:
    def __init__(self, bus_num=1, addr=0x63):
        self.bus = SMBus(bus_num)
        self.addr = addr
        
    def read_reg(self, reg, length=1):
        """Read register(s) from I2C device"""
        try:
            with SMBus(I2C_BUS) as bus:
                w = i2c_msg.write(self.addr, [reg])
                r = i2c_msg.read(self.addr, length)
                bus.i2c_rdwr(w, r)
                return list(r)
        except Exception as e:
            print(f"I2C read error from 0x{self.addr:02X}: {e}")
            return None
    
    def write_reg(self, reg, data):
        """Write data to I2C register"""
        try:
            with SMBus(I2C_BUS) as bus:
                bus.write_byte_data(self.addr, reg, data)
            return True
        except Exception as e:
            print(f"I2C write error to 0x{self.addr:02X}: {e}")
            return False
    
    def method1_rgb_registers(self, r, g, b):
        """Method 1: Set RGB values to specific registers (0x30-0x32) - BGR format"""
        try:
            print(f"Method 1: Setting R={r}, G={g}, B={b} to registers 0x30-0x32 (BGR format)")
            success = True
            success &= self.write_reg(0x30, b)  # Blue component (BGR format)
            success &= self.write_reg(0x31, g)  # Green component  
            success &= self.write_reg(0x32, r)  # Red component (BGR format)
            return success
        except Exception as e:
            print(f"Method 1 error: {e}")
            return False
    
    
    
    def test_color_cycle(self):
        """Test LED with a color cycle"""
        print("\n=== Color Cycle Test ===")
        colors = [
            (255, 0, 0, "Red"),
            (0, 255, 0, "Green"),
            (0, 0, 255, "Blue"),
            (255, 255, 0, "Yellow"),
            (255, 0, 255, "Magenta"),
            (0, 255, 255, "Cyan"),
            (255, 255, 255, "White"),
            (0, 0, 0, "Off")
        ]
        
        for r, g, b, name in colors:
            print(f"Testing {name} (RGB: {r},{g},{b})")
            # Try the most likely successful method first
            self.method1_rgb_registers(r, g, b)
            time.sleep(1)
    
    
    def continuous_button_led(self):
        """Continuous button LED control - runs automatically"""
        print("\n=== Continuous Button LED Control ===")
        print("Starting continuous LED control...")
        print("RED = Button not pressed, GREEN = Button pressed")
        print("Press Ctrl+C to stop")
        
        try:
            while True:
                # Read button state
                btn = self.read_reg(0x20, 1)
                if btn is not None:
                    pressed = (btn[0] == 0)
                    
                    if pressed:
                        # Button pressed - show green
                        self.method1_rgb_registers(0, 255, 0)  # Green (BGR format)
                    else:
                        # Button not pressed - show red
                        self.method1_rgb_registers(255, 0, 0)  # Red (BGR format)
                else:
                    # Error reading button - show blue
                    self.method1_rgb_registers(0, 0, 255)  # Blue for error (BGR format)
                
                time.sleep(0.05)  # 20Hz update rate
                
        except KeyboardInterrupt:
            print("\nContinuous LED control stopped")
            # Turn off LED
            self.method1_rgb_registers(0, 0, 0)
    
    
    
    def scan_i2c_devices(self):
        """Scan for I2C devices"""
        print("Scanning I2C devices...")
        devices = []
        for addr in range(0x08, 0x78):
            try:
                with SMBus(I2C_BUS) as bus:
                    bus.read_byte(addr)
                devices.append(addr)
                print(f"Device found at 0x{addr:02X}")
            except:
                pass
        return devices

def main():
    """Main function"""
    print("M5Stack JS2 Joystick LED Test Script")
    print("====================================")
    print("This script tests LED control functionality of the M5Stack JS2 joystick.")
    print("It tries multiple methods to control the LED based on common I2C protocols.")
    print()
    
    # Create LED tester instance
    tester = JS2LEDTester()
    
    # Scan for I2C devices
    devices = tester.scan_i2c_devices()
    print(f"Found {len(devices)} I2C devices: {[hex(d) for d in devices]}")
    
    if JS2_ADDR not in devices:
        print(f"WARNING: JS2 joystick not found at 0x{JS2_ADDR:02X}")
        print("Please check I2C connections and address")
        return
    
    print(f"JS2 joystick found at 0x{JS2_ADDR:02X}")
    print()
    
    try:
        while True:
            print("\n=== LED Test Menu ===")
            print("1. Color cycle test")
            print("2. Continuous button LED control (Red=Not pressed, Green=Pressed)")
            print("0. Exit")
            
            choice = input("\nSelect option (0-2): ").strip()
            
            if choice == "0":
                break
            elif choice == "1":
                tester.test_color_cycle()
            elif choice == "2":
                tester.continuous_button_led()
            else:
                print("Invalid choice. Please select 0-2.")
                
    except KeyboardInterrupt:
        print("\n\nExiting...")
        # Turn off LED before exiting
        tester.method1_rgb_registers(0, 0, 0)
        print("LED turned off")

if __name__ == "__main__":
    main()
