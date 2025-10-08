# BMI270 IMU ROS 2 Setup Guide

This guide walks through setting up a BMI270 IMU with ROS 2 Jazzy, including proper Python virtual environment configuration.

## Prerequisites

- ROS 2 Jazzy installed
- Python 3.12
- Virtual environment setup
- I2C enabled on Raspberry Pi
- BMI270 IMU connected via I2C

## Step-by-Step Setup

### 1. **Create and Activate Virtual Environment**

```bash
# Create virtual environment
python3 -m venv ~/venv

# Activate virtual environment
source ~/venv/bin/activate

# Verify activation (should show venv path)
which python3
```

### 2. **Install Python Dependencies**

```bash
# Install required packages in virtual environment
pip install smbus2 pymodbus

# Verify installation
python3 -c "import smbus2; print('smbus2 installed successfully')"
python3 -c "import pymodbus; print('pymodbus installed successfully')"
```

### 3. **Create ROS 2 Workspace**

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 4. **Set Up Package Structure**

Create the following directory structure:
```
m5_imu_pro_ros2/
├── package.xml
├── setup.py
├── setup.cfg
├── m5_imu_pro_ros2/                    # Python package directory
│   ├── __init__.py
│   └── imu_pro_read_ros_publish.py     # Main Python file
├── launch/
│   └── imu_launch.py
└── config/
    └── imu_config.yaml
```

### 5. **Copy Package Files**

```bash
# Copy the package to your workspace
cp -r /path/to/m5_imu_pro_ros2 ~/ros2_ws/src/
```

### 6. **Configure Python Environment for colcon**

This is the crucial step that ensures colcon uses your virtual environment:

```bash
# Activate virtual environment
source ~/venv/bin/activate

# Set Python path to include virtual environment packages
export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH

# Verify Python can find the packages
python3 -c "import smbus2; import pymodbus; print('All packages accessible')"
```

### 7. **Build the ROS Package**

```bash
# Build with virtual environment Python
colcon build --packages-select m5_imu_pro_ros2 --cmake-args -DPYTHON_EXECUTABLE=$(which python3)

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

### 8. **Test the IMU Publisher**

```bash
# Test direct execution
ros2 run m5_imu_pro_ros2 imu_publisher

# Test with launch file
ros2 launch m5_imu_pro_ros2 imu_launch.py
```



## Published Topics

- **`/imu/data`** (sensor_msgs/Imu): Linear acceleration, angular velocity, orientation
- **`/imu/temperature`** (sensor_msgs/Temperature): Temperature readings
