#!/usr/bin/env python3
"""
Combined script for M5Unit IMU-Pro Mini and M5 JS2 Joystick
- BMI270: 6-axis IMU (accelerometer + gyroscope)
- BMM150: 3-axis magnetometer  
- BMP280: Barometric pressure sensor
- M5 JS2: Joystick with button

All sensors communicate via I2C interface.
"""

import time
import math
from smbus2 import SMBus, i2c_msg

# I2C Configuration
I2C_BUS = 1  # Default I2C bus on Raspberry Pi

# Sensor I2C addresses (default)
BMI270_ADDR = 0x68  # 6-axis IMU (can be changed with toggle switch)
BMM150_ADDR = 0x13  # 3-axis magnetometer (try 0x10, 0x13, 0x12)
BMP280_ADDR = 0x76  # Barometric pressure sensor
JS2_ADDR = 0x63     # M5 JS2 Joystick

class CombinedIMUJoystick:
    def __init__(self, bus_num=1):
        self.bus = SMBus(bus_num)
        self.bmi270_addr = BMI270_ADDR
        self.bmm150_addr = BMM150_ADDR
        self.bmp280_addr = BMP280_ADDR
        self.js2_addr = JS2_ADDR
        
    def read_reg(self, addr, reg, length=1):
        """Read register(s) from I2C device"""
        try:
            with SMBus(I2C_BUS) as bus:
                w = i2c_msg.write(addr, [reg])
                r = i2c_msg.read(addr, length)
                bus.i2c_rdwr(w, r)
                return list(r)
        except Exception as e:
            print(f"I2C read error from 0x{addr:02X}: {e}")
            return None
    
    def write_reg(self, addr, reg, data):
        """Write data to I2C register"""
        try:
            with SMBus(I2C_BUS) as bus:
                bus.write_byte_data(addr, reg, data)
            return True
        except Exception as e:
            print(f"I2C write error to 0x{addr:02X}: {e}")
            return False
    
    def init_bmi270(self):
        """Initialize BMI270 6-axis IMU"""
        print("Initializing BMI270...")
        
        # Check if BMI270 is present
        chip_id = self.read_reg(self.bmi270_addr, 0x00)
        if chip_id is None or chip_id[0] != 0x24:
            print(f"BMI270 not found at 0x{self.bmi270_addr:02X}")
            return False
        
        print(f"BMI270 found (Chip ID: 0x{chip_id[0]:02X})")
        
        # Soft reset
        self.write_reg(self.bmi270_addr, 0x7E, 0xB6)
        time.sleep(0.1)
        
        # Configure accelerometer
        self.write_reg(self.bmi270_addr, 0x40, 0x0A)  # ACC_CONF
        time.sleep(0.01)
        
        # Configure gyroscope
        self.write_reg(self.bmi270_addr, 0x42, 0x0A)  # GYR_CONF
        time.sleep(0.01)
        
        # Map data ready interrupt
        self.write_reg(self.bmi270_addr, 0x58, 0x80)  # Map DRDY_INT to INT1
        time.sleep(0.01)
        
        # Enable sensors
        self.write_reg(self.bmi270_addr, 0x7C, 0x03)  # Enable accel and gyro
        time.sleep(0.2)  # Wait for sensors to start
        
        print("BMI270 initialization complete")
        return True
    
    def read_bmi270_accel(self):
        """Read accelerometer data (m/s²)"""
        data = self.read_reg(self.bmi270_addr, 0x0C, 6)
        if data is None:
            return None
        
        # Convert raw data to acceleration
        x_raw = self.twos_comp(data[1] << 8 | data[0], 16)
        y_raw = self.twos_comp(data[3] << 8 | data[2], 16) 
        z_raw = self.twos_comp(data[5] << 8 | data[4], 16)
        
        # Convert to G, then to m/s²
        x = (x_raw / 8192.0) * 9.81
        y = (y_raw / 8192.0) * 9.81
        z = (z_raw / 8192.0) * 9.81
        
        return (x, y, z)
    
    def read_bmi270_gyro(self):
        """Read gyroscope data (rad/s)"""
        data = self.read_reg(self.bmi270_addr, 0x12, 6)
        if data is None:
            return None
        
        # Convert raw data to angular velocity
        x_raw = self.twos_comp(data[1] << 8 | data[0], 16)
        y_raw = self.twos_comp(data[3] << 8 | data[2], 16)
        z_raw = self.twos_comp(data[5] << 8 | data[4], 16)
        
        # Convert to dps, then to rad/s
        x = (x_raw / 16.384) * math.pi / 180.0
        y = (y_raw / 16.384) * math.pi / 180.0
        z = (z_raw / 16.384) * math.pi / 180.0
        
        return (x, y, z)
    
    def init_bmm150(self):
        """Initialize BMM150 magnetometer"""
        print("Initializing BMM150...")
        
        # Try different possible addresses for BMM150
        bmm150_addresses = [0x10, 0x11, 0x12, 0x13]
        
        for addr in bmm150_addresses:
            try:
                chip_id = self.read_reg(addr, 0x40)
                if chip_id is not None and chip_id[0] == 0x32:
                    self.bmm150_addr = addr
                    print(f"BMM150 found at 0x{addr:02X} (Chip ID: 0x{chip_id[0]:02X})")
                    
                    # Set power mode to normal
                    self.write_reg(addr, 0x4B, 0x01)
                    time.sleep(0.1)
                    
                    # Set data rate to 10Hz
                    self.write_reg(addr, 0x4C, 0x02)
                    time.sleep(0.1)
                    
                    return True
            except:
                continue
        
        print("BMM150 not found at any address")
        return False
    
    def read_bmm150_mag(self):
        """Read magnetometer data (μT)"""
        data = self.read_reg(self.bmm150_addr, 0x42, 8)
        if data is None:
            return None
        
        # Convert raw data to magnetic field (μT)
        x = self.twos_comp(data[1] << 8 | data[0], 16) / 16.0
        y = self.twos_comp(data[3] << 8 | data[2], 16) / 16.0
        z = self.twos_comp(data[5] << 8 | data[4], 16) / 16.0
        
        return (x, y, z)
    
    def init_bmp280(self):
        """Initialize BMP280 pressure sensor"""
        print("Initializing BMP280...")
        
        # Check if BMP280 is present
        chip_id = self.read_reg(self.bmp280_addr, 0xD0)
        if chip_id is None or chip_id[0] != 0x58:
            print(f"BMP280 not found at 0x{self.bmp280_addr:02X}")
            return False
        
        print(f"BMP280 found (Chip ID: 0x{chip_id[0]:02X})")
        
        # Read calibration data
        calib_data = self.read_reg(self.bmp280_addr, 0x88, 24)
        if calib_data is None:
            return False
        
        # Store calibration coefficients
        self.bmp280_calib = {
            'T1': self.uint16(calib_data[0], calib_data[1]),
            'T2': self.int16(calib_data[2], calib_data[3]),
            'T3': self.int16(calib_data[4], calib_data[5]),
            'P1': self.uint16(calib_data[6], calib_data[7]),
            'P2': self.int16(calib_data[8], calib_data[9]),
            'P3': self.int16(calib_data[10], calib_data[11]),
            'P4': self.int16(calib_data[12], calib_data[13]),
            'P5': self.int16(calib_data[14], calib_data[15]),
            'P6': self.int16(calib_data[16], calib_data[17]),
            'P7': self.int16(calib_data[18], calib_data[19]),
            'P8': self.int16(calib_data[20], calib_data[21]),
            'P9': self.int16(calib_data[22], calib_data[23])
        }
        
        # Set sampling and power mode
        self.write_reg(self.bmp280_addr, 0xF4, 0x27)  # Normal mode, 1x temp, 1x pressure
        time.sleep(0.1)
        
        return True
    
    def read_bmp280_temp_pressure(self):
        """Read temperature and pressure from BMP280"""
        # Read raw data
        data = self.read_reg(self.bmp280_addr, 0xF7, 6)
        if data is None:
            return None, None
        
        # Extract raw values
        adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        
        # Temperature calculation
        var1 = (adc_T / 16384.0 - self.bmp280_calib['T1'] / 1024.0) * self.bmp280_calib['T2']
        var2 = ((adc_T / 131072.0 - self.bmp280_calib['T1'] / 8192.0) * 
                (adc_T / 131072.0 - self.bmp280_calib['T1'] / 8192.0) * self.bmp280_calib['T3'])
        t_fine = var1 + var2
        temperature = t_fine / 5120.0
        
        # Pressure calculation
        var1 = t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.bmp280_calib['P6'] / 32768.0
        var2 = var2 + var1 * self.bmp280_calib['P5'] * 2.0
        var2 = var2 / 4.0 + self.bmp280_calib['P4'] * 65536.0
        var1 = (self.bmp280_calib['P3'] * var1 * var1 / 524288.0 + self.bmp280_calib['P2'] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.bmp280_calib['P1']
        pressure = 1048576.0 - adc_P
        pressure = (pressure - var2 / 4096.0) * 6250.0 / var1
        var1 = self.bmp280_calib['P9'] * pressure * pressure / 2147483648.0
        var2 = pressure * self.bmp280_calib['P8'] / 32768.0
        pressure = pressure + (var1 + var2 + self.bmp280_calib['P7']) / 16.0
        
        return temperature, pressure / 100.0  # Convert to hPa
    
    def read_joystick(self):
        """Read joystick position and button state"""
        try:
            # Read joystick position (centered Int12 values)
            xL, xH, yL, yH = self.read_reg(self.js2_addr, 0x50, 4)
            if xL is None:
                return None, None, None
            
            x = self.s16_le(xL, xH)
            y = self.s16_le(yL, yH)
            
            # Read button state
            btn = self.read_reg(self.js2_addr, 0x20, 1)
            if btn is None:
                return x, y, None
            
            pressed = (btn[0] == 0)
            
            return x, y, pressed
            
        except Exception as e:
            print(f"Joystick read error: {e}")
            return None, None, None
    
    def get_joystick_direction(self, x, y, threshold=600):
        """Get joystick direction based on position"""
        if x is None or y is None:
            return "ERROR"
        
        if abs(x) < threshold and abs(y) < threshold:
            return "CENTER"
        elif abs(x) >= abs(y):
            return "RIGHT" if x > 0 else "LEFT"
        else:
            return "UP" if y > 0 else "DOWN"
    
    def calculate_heading(self, mag_x, mag_y, mag_z):
        """Calculate magnetic heading from magnetometer data"""
        if mag_x is None or mag_y is None or mag_z is None:
            return None
        
        # Simple 2D heading calculation (ignoring Z component)
        heading = math.atan2(mag_y, mag_x) * 180.0 / math.pi
        if heading < 0:
            heading += 360.0
        return heading
    
    def twos_comp(self, val, bits):
        """Convert two's complement to signed integer"""
        if val & (1 << (bits - 1)):
            val -= 1 << bits
        return val
    
    def s16_le(self, lo, hi):
        """Convert two bytes to signed 16-bit integer (little endian)"""
        v = lo | (hi << 8)
        return v - 65536 if v & 0x8000 else v
    
    def uint16(self, lsb, msb):
        """Convert two bytes to unsigned 16-bit integer"""
        return (msb << 8) | lsb
    
    def int16(self, lsb, msb):
        """Convert two bytes to signed 16-bit integer"""
        val = (msb << 8) | lsb
        return self.twos_comp(val, 16)
    
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
    
    def test_all_sensors(self):
        """Test all sensors and display readings"""
        print("=== Combined IMU + Joystick Test ===")
        
        # First, scan for I2C devices
        devices = self.scan_i2c_devices()
        print(f"Found {len(devices)} I2C devices: {[hex(d) for d in devices]}")
        
        print("\nInitializing sensors...")
        
        # Initialize all sensors
        bmi270_ok = self.init_bmi270()
        bmm150_ok = self.init_bmm150()
        bmp280_ok = self.init_bmp280()
        
        # Test joystick
        print("Testing joystick...")
        js_x, js_y, js_btn = self.read_joystick()
        if js_x is not None:
            print(f"Joystick found at 0x{self.js2_addr:02X}")
            js_ok = True
        else:
            print(f"Joystick not found at 0x{self.js2_addr:02X}")
            js_ok = False
        
        if not any([bmi270_ok, bmm150_ok, bmp280_ok, js_ok]):
            print("No sensors found! Check I2C connections.")
            return
        
        print("\n=== Combined Sensor Readings ===")
        print("Press Ctrl+C to exit")
        print("="*80)
        
        try:
            while True:
                print("\n" + "="*80)
                
                # BMI270 readings
                if bmi270_ok:
                    accel = self.read_bmi270_accel()
                    gyro = self.read_bmi270_gyro()
                    
                    if accel is not None:
                        print(f"Accelerometer (m/s²): X={accel[0]:6.2f}, Y={accel[1]:6.2f}, Z={accel[2]:6.2f}")
                    else:
                        print("Accelerometer: Failed to read")
                    
                    if gyro is not None:
                        print(f"Gyroscope (rad/s):     X={gyro[0]:6.3f}, Y={gyro[1]:6.3f}, Z={gyro[2]:6.3f}")
                    else:
                        print("Gyroscope: Failed to read")
                
                # BMM150 readings
                if bmm150_ok:
                    mag = self.read_bmm150_mag()
                    if mag is not None:
                        heading = self.calculate_heading(mag[0], mag[1], mag[2])
                        print(f"Magnetometer (μT):     X={mag[0]:6.1f}, Y={mag[1]:6.1f}, Z={mag[2]:6.1f}")
                        if heading is not None:
                            print(f"Magnetic Heading:      {heading:6.1f}°")
                    else:
                        print("Magnetometer: Failed to read")
                
                # BMP280 readings
                if bmp280_ok:
                    temp, pressure = self.read_bmp280_temp_pressure()
                    if temp is not None and pressure is not None:
                        print(f"Temperature:           {temp:6.1f}°C")
                        print(f"Pressure:              {pressure:6.1f} hPa")
                    else:
                        print("BMP280: Failed to read")
                
                # Joystick readings
                if js_ok:
                    js_x, js_y, js_btn = self.read_joystick()
                    if js_x is not None:
                        direction = self.get_joystick_direction(js_x, js_y)
                        btn_status = "PRESSED" if js_btn else "released"
                        print(f"Joystick:              X={js_x:6d}, Y={js_y:6d}, Dir={direction:6s}, BTN={btn_status}")
                    else:
                        print("Joystick: Failed to read")
                
                time.sleep(0.1)  # 10Hz update rate
                
        except KeyboardInterrupt:
            print("\n\nTest completed!")

def main():
    """Main function"""
    print("Combined IMU + Joystick Test Script")
    print("===================================")
    print("This script tests the following sensors:")
    print("- BMI270: 6-axis IMU (accelerometer + gyroscope)")
    print("- BMM150: 3-axis magnetometer")
    print("- BMP280: Barometric pressure sensor")
    print("- M5 JS2: Joystick with button")
    print()
    
    # Create combined sensor instance
    sensors = CombinedIMUJoystick()
    
    # Run combined sensor test
    sensors.test_all_sensors()

if __name__ == "__main__":
    main()
