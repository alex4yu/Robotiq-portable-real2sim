# System-Wide Python Setup for Robotiq ROS 2 System

This document summarizes the system-wide Python package setup for your Robotiq gripper system with ROS 2.

## Why System-Wide Instead of Venv?

**ROS 2 works better with system-wide Python packages because:**
- ROS 2 commands (`ros2 run`, `ros2 launch`) use system Python by default
- No need to manage venv activation across different terminals
- Simpler integration with system services
- Avoids Python path conflicts

## Complete Setup Commands

### 1. Install System-Wide Python Packages

```bash
# Exit any virtual environment first
deactivate  # if in venv

# Install required packages system-wide
pip3 install smbus2 pymodbus==2.5.3 minimalmodbus evdev pynput --break-system-packages

# Verify installations
python3 -c "import smbus2; import pymodbus; import minimalmodbus; print('✓ All required packages installed')"

# Optional: RealSense camera support (skip if not needed)
# Note: pyrealsense2 may not be available via pip on all platforms
# See REALSENSE_SETUP_GUIDE.md for detailed build instructions
```

### 2. Build ROS 2 Workspace

```bash
cd ~/ros2_ws

# Clean previous builds (optional but recommended)
rm -rf build/ install/ log/

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build all packages
colcon build

# Or build specific package
colcon build --packages-select m5_imu_pro_ros2

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

### 3. Add to .bashrc (Optional but Recommended)

Add these lines to `~/.bashrc` for automatic sourcing:

```bash
# ROS 2 setup
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Optional: Set ROS_DOMAIN_ID to avoid interference
export ROS_DOMAIN_ID=42
```

Then reload:
```bash
source ~/.bashrc
```

## Testing Components

### Test IMU Publisher

```bash
ros2 run m5_imu_pro_ros2 imu_publisher
```

Should output IMU data without errors.

### Test FT Sensor

```bash
cd ~/robotiq_ws/src  # or wherever your scripts are
python3 test_ft_sensor.py
```

Should show force/torque readings.

### Test Complete System

```bash
python3 src/main_launcher.py
```

This will start:
- IMU ROS publisher
- Joystick teleop
- Recording control

## Package Locations

### System-Wide Packages
```bash
# Check where packages are installed
pip3 show smbus2
pip3 show pymodbus
pip3 show pyrealsense2
```

Typically in: `/usr/local/lib/python3.12/dist-packages/`

### ROS 2 Workspace
- Source: `~/ros2_ws/src/m5_imu_pro_ros2/`
- Install: `~/ros2_ws/install/m5_imu_pro_ros2/`

## Troubleshooting

### "ModuleNotFoundError: No module named 'smbus2'"

```bash
# Check if installed system-wide
python3 -c "import smbus2; print('OK')"

# If not, install it
pip3 install smbus2 --break-system-packages
```

### "No communication with the instrument" (FT Sensor)

```bash
# Check if port exists
ls -l /dev/ttySC*

# Check permissions
sudo chmod 666 /dev/ttySC1

# Or add yourself to dialout group permanently
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### "I2C device not found" (IMU)

```bash
# Check I2C is enabled
ls /dev/i2c-*

# Scan for I2C devices
i2cdetect -y 1

# Should show device at address 0x68 or 0x69

# If not enabled, enable it
sudo raspi-config
# Interface Options -> I2C -> Enable
```

### ROS 2 Commands Not Found

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Or add to ~/.bashrc permanently
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Complete Dependency List

### Python Packages (System-Wide)
- `smbus2` - I2C communication for IMU
- `pymodbus==2.5.3` - Modbus communication for Robotiq gripper (v2.x API)
- `minimalmodbus` - For FT sensor communication
- `evdev` - Joystick input handling
- `pynput` - Keyboard input
- `pyrealsense2` - **OPTIONAL** RealSense camera support (build from source if needed)

### System Packages
- `ros-jazzy-desktop` - ROS 2 Jazzy
- `python3-colcon-common-extensions` - Build tools
- `i2c-tools` - I2C debugging

### Install All At Once

```bash
# Python packages (required)
pip3 install smbus2 pymodbus==2.5.3 minimalmodbus evdev pynput --break-system-packages

# Optional: pyrealsense2 (only if you need RealSense cameras)
# This may not work on all platforms - build from source if needed
# pip3 install pyrealsense2 --break-system-packages

# System packages (if needed)
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions i2c-tools
```

## Quick Reference

| Component | Command | Port/Topic |
|-----------|---------|------------|
| IMU Publisher | `ros2 run m5_imu_pro_ros2 imu_publisher` | `/imu/data`, `/imu/temperature` |
| FT Sensor Test | `python3 test_ft_sensor.py` | `/dev/ttySC1` |
| Complete System | `python3 src/main_launcher.py` | All components |
| View IMU Topics | `ros2 topic list` | - |
| View IMU Data | `ros2 topic echo /imu/data` | - |

## Files Modified

The following files have been updated for system-wide operation:

1. `m5_imu_pro_ros2/setup.py` - Added `smbus2` dependency
2. `src/main_launcher.py` - Removed venv activation
3. `IMU_SETUP_GUIDE.md` - Updated for system-wide packages
4. `src/ft_sensor_weight_com.py` - Updated port to `/dev/ttySC1`

## Next Steps

1. ✓ Install system-wide packages
2. ✓ Build ROS 2 workspace
3. Test each component individually
4. Run complete system with `main_launcher.py`
5. Test weight/CoM measurement with FT sensor
6. Set up automatic startup (optional)

## Optional: Auto-Start on Boot

To start the system automatically on boot:

```bash
# Create systemd service
sudo nano /etc/systemd/system/robotiq-system.service
```

Add:
```ini
[Unit]
Description=Robotiq Gripper System
After=network.target

[Service]
Type=simple
User=geoduderp5
WorkingDirectory=/home/geoduderp5/robotiq_ws
ExecStart=/usr/bin/python3 /home/geoduderp5/robotiq_ws/src/main_launcher.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable robotiq-system
sudo systemctl start robotiq-system
```

---

**Questions?** See the detailed guides:
- `IMU_SETUP_GUIDE.md` - IMU setup details
- `FT_SENSOR_WEIGHT_COM_GUIDE.md` - FT sensor usage
- `REALSENSE_SETUP_GUIDE.md` - RealSense camera setup

