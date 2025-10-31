# Robotiq Portable Real2Sim System

A comprehensive system for controlling a **Robotiq 2F-85** gripper with integrated IMU, force/torque (FT) sensor, RealSense camera, and data recording capabilities. The system provides joystick-based teleoperation with proportional control and unified recording of all sensor data streams.

## Overview

This system integrates multiple hardware components into a unified ROS 2 ecosystem:

- **Robotiq 2F-85 Gripper** - Controlled via RS-485 Modbus communication
- **M5 IMU Pro (BMI270)** - Inertial measurement unit publishing accelerometer, gyroscope, and temperature data
- **FT Sensor** - Force/torque sensor (FT300-S) for measuring contact forces
- **RealSense Camera** - Multi-stream camera (IR1, IR2, Color, Depth) for vision data
- **M5 Joystick2** - I2C joystick for gripper control and recording toggle

All components publish to ROS 2 topics and can be recorded simultaneously to ROS bag files for simulation replay.

## Features

- ✅ **Proportional Gripper Control** - Joystick position maps to gripper speed and force
- ✅ **Unified ROS 2 Integration** - All sensors publish to standardized topics
- ✅ **Synchronized Recording** - Capture all data streams (IMU, FT, camera, gripper state) simultaneously
- ✅ **LED Status Indicators** - Visual feedback for recording state (RED = not recording, GREEN = recording)
- ✅ **Process Management** - Automatic startup, monitoring, and graceful shutdown of all components
- ✅ **Dependency Checking** - Validates all required dependencies before startup

## System Architecture

The system consists of three main ROS 2 publishers and one unified controller:

```
┌─────────────────────────────────────────────────────────────┐
│                    Main Launcher                            │
│                  (main_launcher.py)                         │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐ │
│  │ IMU Publisher│  │ FT Publisher │  │ Unified Controller│ │
│  │              │  │              │  │                   │ │
│  │ /imu/data    │  │ /ft_sensor/  │  │ - Gripper Control │ │
│  │ /imu/temp    │  │   wrench     │  │ - RealSense      │ │
│  │              │  │   accel      │  │ - Recording      │ │
│  └──────────────┘  └──────────────┘  └──────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### Components

1. **IMU Publisher** (`imu_ros_publisher.py`)
   - Reads BMI270 sensor via I2C
   - Publishes to `/imu/data` (sensor_msgs/Imu)
   - Publishes to `/imu/temperature` (sensor_msgs/Temperature)

2. **FT Sensor Publisher** (`ft_sensor_ros_publisher.py`)
   - Reads FT300-S sensor via RS-485 Modbus
   - Publishes to `/ft_sensor/wrench` (geometry_msgs/WrenchStamped)
   - Publishes to `/ft_sensor/acceleration` (geometry_msgs/Vector3Stamped)

3. **Unified System Controller** (`unified_system_controller.py`)
   - Gripper control via joystick (proportional)
   - RealSense streaming (IR1, IR2, Color, Depth)
   - Recording toggle via joystick button
   - LED status indicator

## Quick Start

### Prerequisites

- **Hardware:**
  - Raspberry Pi 5 (recommended) or compatible Linux system
  - Robotiq 2F-85 gripper with 24V power supply
  - M5 IMU Pro (BMI270)
  - FT300-S force/torque sensor
  - RealSense camera (D435/D435i recommended)
  - M5 Joystick2
  - RS-485 interface (USB converter or Waveshare RS485 CAN HAT)
  - I2C enabled on Raspberry Pi

- **Software:**
  - ROS 2 Jazzy (or compatible)
  - Python 3.10+
  - System-wide Python packages (see Installation)

### Installation

1. **Install ROS 2 Jazzy:**
   ```bash
   # Follow ROS 2 Jazzy installation guide for your platform
   # https://docs.ros.org/en/jazzy/Installation.html
   ```

2. **Install Python dependencies:**
   ```bash
   pip3 install smbus2 pymodbus==2.5.3 minimalmodbus pyserial pyrealsense2 --break-system-packages
   ```
   
   **Important:** You must use `pymodbus==2.5.3` (v2.x). Do not use pymodbus v3.x as it has breaking API changes that are incompatible with this system.

3. **Build ROS 2 workspace:**
   ```bash
   cd ~/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build
   source ~/ros2_ws/install/setup.bash
   ```

4. **Configure hardware:**
   - See `IMU_SETUP_GUIDE.md` for IMU setup
   - See `REALSENSE_SETUP_GUIDE.md` for RealSense setup
   - Ensure I2C is enabled: `sudo raspi-config` → Interface Options → I2C

## Hardware Setup

### RS-485 Connection Options

The system supports two RS-485 connection methods for the Robotiq gripper and FT sensor:

- **Option A:** USB-RS485 converter (e.g., CH340, FTDI) - Quick setup for testing
- **Option B:** Waveshare RS485 CAN HAT (B) - Recommended for permanent setup

### Port Configuration and Setup

#### USB-RS485 Converter Setup

1. **Find the USB device:**
   ```bash
   sudo dmesg | grep ttyUSB
   ```
   Typically shows `/dev/ttyUSB0` or similar

2. **Configure the port in `unified_system_controller.py`:**
   ```python
   PORT = "/dev/ttyUSB0"
   ```

#### RS485 CAN HAT (B) Setup

The Waveshare RS485 CAN HAT uses an **SC16IS752 dual UART chip (via SPI)** for RS-485 communication.

**1. Configure Raspberry Pi OS (Bookworm):**

Edit `/boot/firmware/config.txt` and add these lines at the bottom:

```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25,spimaxfrequency=1000000
dtoverlay=sc16is752-spi1,int_pin=24
```

**2. Reboot the Raspberry Pi:**
```bash
sudo reboot
```

**3. Verify UART devices:**

After reboot, check for new UART devices:
```bash
ls /dev/ttySC*
```

Expected output:
```
/dev/ttySC0  /dev/ttySC1
```

Use `/dev/ttySC0` for the gripper (RS485_0 terminal block).  
Use `/dev/ttySC1` for the FT sensor if connected to RS485_1.

**4. Set up port permissions:**

Add your user to the `dialout` group to access serial ports:
```bash
sudo usermod -aG dialout $USER
```
Logout and login again for changes to take effect.

**Important:** The gripper requires a separate **24V DC power supply**. The USB converter or HAT does not provide power to the gripper.

### Expected Gripper Behavior

When the system starts correctly:

- **Power-up (no communication):** Gripper LED = **RED** (waiting for activation)
- **After activation:** Gripper LED = **BLUE** (activated and ready)
- **During operation:** The keepalive thread continuously polls gripper status to prevent timeout faults
- **If LED turns blue then red:** Keepalive is missing - ensure the control script is running

### Running the System

**Start all components:**
```bash
python3 src/main_launcher.py
```

The launcher will:
1. Check all dependencies
2. Start IMU publisher
3. Start FT sensor publisher
4. Start unified system controller (gripper + camera + recording)

## Controls

### Joystick Operation

- **UP (Push)** - Open gripper
  - Push further = faster opening speed
  - Fixed force (max for reliable opening)

- **DOWN (Pull)** - Close gripper
  - Pull further = faster speed + stronger grip force
  - Proportional control for precise grasping

- **CENTER** - Stop and hold position
  - Gripper stops immediately and maintains current position

- **Button Press** - Toggle recording
  - First press: Start recording all streams to ROS bag
  - Second press: Stop recording and save bag file
  - LED indicates state: RED = not recording, GREEN = recording

### Recorded Topics

When recording is active, the following topics are saved to a ROS bag file:

- `/camera/infra1/image_raw` - Infrared stream 1
- `/camera/infra2/image_raw` - Infrared stream 2
- `/camera/color/image_raw` - RGB color stream
- `/camera/depth/image_raw` - Depth stream
- `/imu/data` - IMU accelerometer and gyroscope data
- `/imu/temperature` - IMU temperature
- `/ft_sensor/wrench` - Force and torque measurements
- `/ft_sensor/acceleration` - FT sensor accelerometer

Bag files are saved to: `~/robotiq_ws/recordings/` with timestamp format: `YYYY-MM-DD-HH.MM.SS.bag`

## Configuration

### Port Configuration

Edit the following files to match your hardware setup:

**Gripper (unified_system_controller.py):**
```python
PORT = "/dev/ttySC0"  # Change to "/dev/ttyUSB0" for USB converter
BAUD = 115200
UNIT = 9
```

**FT Sensor (ft_sensor_ros_publisher.py):**
```python
PORT = "/dev/ttySC1"  # Adjust for your setup
SLAVE = 9
```

### Joystick Calibration

Adjust thresholds and ranges in `unified_system_controller.py`:

```python
DEADZONE_RADIUS = 200        # Deadzone size
Y_OPEN_THRESHOLD = 300       # Threshold for open command
Y_CLOSE_THRESHOLD = -300     # Threshold for close command
MIN_OPEN_SPEED = 0x10        # Minimum opening speed
MAX_OPEN_SPEED = 0xFF        # Maximum opening speed
MIN_CLOSE_FORCE = 0x10       # Minimum closing force (gentle)
MAX_CLOSE_FORCE = 0xFF       # Maximum closing force (strong)
```

## Troubleshooting

### Common Issues

**"ModuleNotFoundError: No module named 'smbus2'"**
```bash
pip3 install smbus2 --break-system-packages
```

**"Could not connect to gripper"**
- Check RS-485 connection and port: `ls /dev/ttySC*` or `ls /dev/ttyUSB*`
- Verify 24V power supply is connected (separate from RS-485)
- **If using HAT:** Verify `/dev/ttySC*` devices exist after reboot
  - Check `/boot/firmware/config.txt` has correct overlays
  - Ensure SPI is enabled: `dtparam=spi=on`
- **If no response:** Verify RS-485 wiring connections and grounds
- **LED behavior troubleshooting:**
  - LED red (waiting): Normal on power-up, should turn blue after activation
  - LED blue then red: Keepalive missing - ensure control script is running continuously

**"I2C device not found" (IMU or Joystick)**
```bash
# Enable I2C
sudo raspi-config  # Interface Options → I2C → Enable

# Scan for devices
i2cdetect -y 1

# Should show:
# - 0x68 for BMI270 IMU
# - 0x63 for M5 Joystick2
```

**"Permission denied" on /dev/tty*** or `/dev/i2c-*`**
```bash
sudo usermod -aG dialout $USER
sudo usermod -aG i2c $USER
# Logout and login again
```

**No /dev/ttySC* devices (HAT not detected)**
- Double-check overlays in `/boot/firmware/config.txt`
- Verify SPI is enabled: `dtparam=spi=on`
- Ensure `dtoverlay=sc16is752-spi1,int_pin=24` is present
- Reboot after editing config.txt
- Check if HAT is properly seated on Raspberry Pi GPIO pins

**ROS 2 commands not found**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Or add to ~/.bashrc permanently
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**RealSense camera not detected**
- Check USB connection
- Install RealSense SDK: See `REALSENSE_SETUP_GUIDE.md`
- Verify with: `rs-enumerate-devices`

### Process Monitoring

The main launcher monitors all processes and will report if any component stops unexpectedly. Check the console output for error messages from individual components.

## Project Structure

```
.
├── README.md                      # This file
├── requirements.txt               # Python dependencies
├── SYSTEM_SETUP_SUMMARY.md        # Detailed setup instructions
├── IMU_SETUP_GUIDE.md            # IMU configuration guide
├── REALSENSE_SETUP_GUIDE.md      # RealSense setup guide
├── src/
│   ├── main_launcher.py          # Main system launcher
│   ├── unified_system_controller.py  # Gripper + Camera + Recording
│   ├── imu_ros_publisher.py      # IMU ROS 2 publisher
│   ├── ft_sensor_ros_publisher.py    # FT sensor ROS 2 publisher
│   └── testing/                  # Test and development scripts
└── m5_imu_pro_ros2/              # ROS 2 package for M5 IMU Pro
    ├── setup.py
    ├── package.xml
    └── m5_imu_pro_ros2/
        └── imu_pro_read_ros_publish.py
```

## Dependencies

### Python Packages (Required)
- `pymodbus==2.5.3` - Modbus communication for gripper and FT sensor (**IMPORTANT: Use v2.x only, v3.x has breaking changes**)
- `pyserial` - Serial port communication
- `smbus2` - I2C communication for IMU and joystick
- `minimalmodbus` - Modbus for FT sensor
- `pyrealsense2` - RealSense camera SDK (optional if not using camera)

### System Packages
- ROS 2 Jazzy desktop
- `python3-colcon-common-extensions` - Build tools
- `i2c-tools` - I2C debugging utilities

## Additional Resources

- **System Setup:** See `SYSTEM_SETUP_SUMMARY.md` for detailed installation
- **IMU Configuration:** See `IMU_SETUP_GUIDE.md`
- **RealSense Setup:** See `REALSENSE_SETUP_GUIDE.md`
- **Hardware Wiring:** Detailed wiring diagrams and RS-485 setup in this README

## ROS 2 Topics

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | IMU accelerometer and gyroscope data |
| `/imu/temperature` | `sensor_msgs/Temperature` | IMU temperature reading |
| `/ft_sensor/wrench` | `geometry_msgs/WrenchStamped` | Force and torque measurements |
| `/ft_sensor/acceleration` | `geometry_msgs/Vector3Stamped` | FT sensor accelerometer |
| `/camera/infra1/image_raw` | `sensor_msgs/Image` | RealSense IR stream 1 |
| `/camera/infra2/image_raw` | `sensor_msgs/Image` | RealSense IR stream 2 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RealSense RGB color stream |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | RealSense depth stream |

### Viewing Data

```bash
# List all topics
ros2 topic list

# Echo IMU data
ros2 topic echo /imu/data

# Echo FT sensor data
ros2 topic echo /ft_sensor/wrench

# View topic info
ros2 topic info /imu/data
```

## Stopping the System

Press `Ctrl+C` in the terminal running `main_launcher.py`. The launcher will:
1. Gracefully stop all processes
2. Stop any active recordings
3. Open the gripper before shutdown
4. Turn off LED indicators

## Auto-Start on Boot

To automatically start the system when the Raspberry Pi boots, set up a systemd service:

### 1. **Configure the Service File**

Edit `robotiq-system.service` and update the following:
- Replace `geoduderp5` with your actual Linux username (appears twice)
- Update the `WorkingDirectory` path to your actual project directory
- Update the paths in `ExecStart` to match your ROS 2 installation and project location

Example (if your username is `pi` and project is in `/home/pi/robotiq_ws`):
```ini
[Unit]
Description=Robotiq Gripper System Launcher
After=network.target
Wants=network-online.target

[Service]
Type=simple
User=pi
Group=pi
WorkingDirectory=/home/pi/robotiq_ws
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && python3 /home/pi/robotiq_ws/src/main_launcher.py'
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal
SyslogIdentifier=robotiq-system
Environment="PYTHONUNBUFFERED=1"

[Install]
WantedBy=multi-user.target
```

**Note:** The service directly sources ROS 2 and runs the launcher - no wrapper script needed.

### 2. **Install and Enable the Service**

```bash
# Copy service file to systemd directory
sudo cp robotiq-system.service /etc/systemd/system/

# Reload systemd to recognize the new service
sudo systemctl daemon-reload

# Enable the service to start on boot
sudo systemctl enable robotiq-system.service

# Start the service immediately (optional)
sudo systemctl start robotiq-system.service

# Check service status
sudo systemctl status robotiq-system.service
```

### 3. **Managing the Service**

```bash
# View service logs
sudo journalctl -u robotiq-system.service -f

# Stop the service
sudo systemctl stop robotiq-system.service

# Restart the service
sudo systemctl restart robotiq-system.service

# Disable auto-start (but keep service installed)
sudo systemctl disable robotiq-system.service

# Remove the service entirely
sudo systemctl disable robotiq-system.service
sudo rm /etc/systemd/system/robotiq-system.service
sudo systemctl daemon-reload
```

### 4. **Verify Auto-Start**

After enabling the service, reboot your Raspberry Pi:
```bash
sudo reboot
```

After reboot, check if the service is running:
```bash
sudo systemctl status robotiq-system.service
```

You should see the service is active and running. Check the logs if there are any issues:
```bash
sudo journalctl -u robotiq-system.service --since "10 minutes ago"
```

## License

This project is provided as-is for research and development purposes.

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review the setup guides in the project
3. Verify hardware connections and configurations
4. Check ROS 2 topic outputs: `ros2 topic list` and `ros2 topic echo <topic_name>`

