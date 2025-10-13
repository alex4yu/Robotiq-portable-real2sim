# Intel RealSense SDK Setup Guide

This guide walks through building and installing librealsense (Intel RealSense SDK) from source, including Python bindings.

## Overview

The Intel RealSense SDK (librealsense) provides access to RealSense cameras for depth sensing, RGB streams, and infrared imaging. Building from source ensures compatibility with your system and Python version.

## Prerequisites

- Ubuntu 20.04 or newer
- Python 3.8+ (tested with 3.12.3)
- Git
- CMake 3.8+
- USB 3.0 port
- Intel RealSense camera (D400 series, L500 series, etc.)

## Step-by-Step Installation

### 1. Install Build Dependencies

```bash
# Update package list
sudo apt-get update

# Install build tools
sudo apt-get install -y \
    git \
    cmake \
    build-essential \
    pkg-config \
    libssl-dev \
    libusb-1.0-0-dev \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev

# Verify installations
cmake --version
gcc --version
```

### 2. Clone the librealsense Repository

```bash
# Create workspace directory
mkdir -p ~/robotiq_ws
cd ~/robotiq_ws

# Clone the repository
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Optional: Checkout a specific stable version
# git checkout v2.54.2  # Replace with desired version
# Or use the latest:
git checkout master

# Check the current branch/tag
git describe --tags
```

### 3. Install udev Rules (Required for USB Access)

```bash
# Copy udev rules to system directory
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add yourself to video group (required for camera access)
sudo usermod -a -G video $USER

# Note: You may need to log out and log back in for group changes to take effect
```

### 4. Create Build Directory and Configure

```bash
cd ~/robotiq_ws/librealsense

# Create and enter build directory
mkdir build
cd build

# Configure with CMake
# IMPORTANT: Use system Python, not a virtual environment
cmake ../ \
    -DBUILD_PYTHON_BINDINGS=bool:true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true

# Verify configuration output shows:
#   -- Building Python bindings for Python 3.x
#   -- Python executable: /usr/bin/python3
```

**Common CMake Options:**

| Option | Description | Default |
|--------|-------------|---------|
| `BUILD_PYTHON_BINDINGS` | Build Python wrapper | OFF |
| `BUILD_EXAMPLES` | Build C++ examples | ON |
| `BUILD_GRAPHICAL_EXAMPLES` | Build GUI tools (realsense-viewer) | ON |
| `CMAKE_BUILD_TYPE` | Release or Debug | Release |
| `PYTHON_EXECUTABLE` | Path to Python interpreter | Auto-detected |

### 5. Build the SDK

```bash
# Build with 4 parallel jobs (adjust based on your CPU cores)
make -j4

# This will take 10-30 minutes depending on your system
# Watch for any errors during compilation

# Expected output at the end:
# [100%] Built target realsense2
# [100%] Built target realsense-viewer
# [100%] Built target pyrealsense2  (Python bindings)
```

**Build Progress Indicators:**
- `[  3%]` - Early build stages (utilities)
- `[ 25%]` - Core library compilation
- `[ 50%]` - Examples and tools
- `[ 75%]` - Python bindings
- `[100%]` - Complete!

### 6. Install the SDK

```bash
# Install to system directories (requires sudo)
sudo make install

# Update shared library cache
sudo ldconfig

# Verify library installation
ldconfig -p | grep realsense
# Should show: librealsense2.so.2.xx.x
```

### 7. Install Python Bindings

The Python bindings should be installed automatically with `make install`, but if needed:

```bash
# Method 1: System-wide installation
cd ~/robotiq_ws/librealsense/wrappers/python
sudo python3 setup.py install

# Method 2: User installation (no sudo required)
cd ~/robotiq_ws/librealsense/wrappers/python
python3 setup.py install --user

# Method 3: Using pip (if wheel was built)
pip3 install ~/robotiq_ws/librealsense/build/wrappers/python/dist/*.whl --break-system-packages
```

### 8. Verify Installation

```bash
# Test Python import
python3 -c "import pyrealsense2 as rs; print(f'pyrealsense2 version: {rs.__version__}')"

# Expected output:
# pyrealsense2 version: 2.xx.x

# Test with a connected camera (if available)
python3 << EOF
import pyrealsense2 as rs

# Create pipeline
pipeline = rs.pipeline()
config = rs.config()

# Try to start the pipeline
try:
    pipeline.start(config)
    print("✓ RealSense camera detected and working!")
    pipeline.stop()
except RuntimeError as e:
    print(f"⚠ Camera not detected or not connected: {e}")
EOF
```

### 9. Test GUI Viewer (Optional)

```bash
# Launch RealSense Viewer
realsense-viewer

# This will open a GUI application showing:
# - Connected cameras
# - Available streams (RGB, Depth, IR)
# - Camera controls
# - 3D visualization
```

**Viewer Controls:**
- Click camera serial number to expand options
- Toggle streams (RGB, Depth, Infrared)
- Adjust camera settings (exposure, gain, etc.)
- View 3D point cloud
- Record/playback bag files

