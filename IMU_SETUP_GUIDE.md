# BMI270 IMU ROS 2 Setup Guide

This guide walks through setting up a BMI270 IMU with ROS 2 Jazzy using a standalone Python script.

## Prerequisites

- ROS 2 Jazzy installed
- Python 3.10+ (system-wide, not virtual environment)
- I2C enabled on Raspberry Pi
- BMI270 IMU connected via I2C (typically at address 0x68)

## Step-by-Step Setup

### 1. **Enable I2C Interface**

```bash
# Enable I2C via raspi-config
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable

# Verify I2C is enabled
ls /dev/i2c-*

# Should show /dev/i2c-1 (or similar)
```

### 2. **Install Python Dependencies (System-Wide)**

Install the required packages system-wide (ROS 2 works better with system-wide packages):

```bash
# Install smbus2 for I2C communication
pip3 install smbus2 --break-system-packages

# Verify installation
python3 -c "import smbus2; print('smbus2 installed successfully')"
```

**Note:** We use system-wide packages instead of a virtual environment because ROS 2 commands (`ros2 run`, `ros2 launch`) use system Python by default.

### 3. **Verify I2C Connection**

Check that the IMU is detected on the I2C bus:

```bash
# Scan I2C bus for devices
i2cdetect -y 1
```

Expected output should show device at address `68` (or `0x68`):
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```

If the device is not found, check:
- I2C is enabled (`sudo raspi-config`)
- Wiring connections (SDA, SCL, VCC, GND)
- Device address matches (0x68 is default for BMI270)

### 4. **Set Up Port Permissions (If Needed)**

If you encounter permission errors:

```bash
# Add your user to the i2c group
sudo usermod -aG i2c $USER

# Logout and login again for changes to take effect
```

### 5. **Test the IMU Publisher**

The IMU publisher is a standalone Python script (`src/imu_ros_publisher.py`). To run it:

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Run the IMU publisher script
python3 src/imu_ros_publisher.py
```

Or run it via the main launcher:

```bash
python3 src/main_launcher.py
```

The script will automatically:
- Initialize the BMI270 sensor via I2C
- Start publishing to `/imu/data` and `/imu/temperature` topics
- Continue running until interrupted (Ctrl+C)

### 6. **Verify IMU Data is Published**

In a separate terminal:

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# List topics
ros2 topic list

# Should show:
# /imu/data
# /imu/temperature

# View IMU data
ros2 topic echo /imu/data

# View temperature
ros2 topic echo /imu/temperature
```

You should see continuous data streams from the IMU.



## Published Topics

- **`/imu/data`** (sensor_msgs/Imu): Linear acceleration, angular velocity, orientation quaternion
- **`/imu/temperature`** (sensor_msgs/Temperature): Temperature readings in Celsius

## Troubleshooting

### "ModuleNotFoundError: No module named 'smbus2'"
```bash
pip3 install smbus2 --break-system-packages
```

### "Permission denied" on `/dev/i2c-*`
```bash
sudo usermod -aG i2c $USER
# Logout and login again
```

### I2C device not found (no device at 0x68)
- Verify I2C is enabled: `sudo raspi-config` → Interface Options → I2C
- Check wiring: SDA, SCL, VCC (3.3V), GND
- Verify device address: `i2cdetect -y 1` should show device at 0x68
- Ensure IMU is powered on

### "ROS2 not available" error
```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Verify ROS 2 is installed
ros2 --help
```

### No data published
- Check that the script is running without errors
- Verify I2C connection: `i2cdetect -y 1`
- Check ROS 2 topics: `ros2 topic list`
- Verify subscribers: `ros2 topic info /imu/data`
