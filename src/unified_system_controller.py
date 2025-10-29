#!/usr/bin/env python3
"""
Unified System Controller for Robotiq Gripper with Recording

Combines:
- Joystick teleoperation of Robotiq gripper
- RealSense streaming (IR1, IR2, Color, Depth) 
- IMU data publishing
- FT sensor data publishing
- Unified ROS bag recording of all streams

Controls:
- Joystick Y-axis: Pull DOWN -> gripper CLOSE, release -> OPEN
- Joystick button: Press to START/STOP recording all data streams
- LED shows RED when not recording, GREEN when recording (BGR format)

Dependencies:
  pip install pyrealsense2 smbus2 pymodbus minimalmodbus pyserial
"""

import os
import time
import threading
import signal
import sys
import subprocess
from datetime import datetime
from pathlib import Path

import numpy as np
from smbus2 import SMBus, i2c_msg
import pyrealsense2 as rs

try:
    from pymodbus.client import ModbusSerialClient          # pymodbus >=3
except Exception:
    from pymodbus.client.sync import ModbusSerialClient      # pymodbus 2.x

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Imu, Temperature
    from geometry_msgs.msg import WrenchStamped, Vector3Stamped
    from std_msgs.msg import Header
    from cv_bridge import CvBridge
except ImportError:
    print("ROS2 not available. Please install ROS2 Jazzy and cv_bridge.")
    sys.exit(1)

# ========= RS-485 / Robotiq =========
PORT = "/dev/ttySC0"         # e.g. "/dev/ttyUSB0" if using a USB RS-485 dongle
BAUD = 115200
UNIT = 9
CTRL_ADDR = 0x03E8
STAT_ADDR = 0x07D0

# ========= Joystick (M5 Joystick2 @ 0x63) =========
I2C_BUS = 1
JOY_ADDR = 0x63
REG_XY_INT12 = 0x50          # returns 4 bytes: xL,xH,yL,yH as signed centered values (-4095..4095)
BTN_REG = 0x20               # 1 = not pressed, 0 = pressed
POLL_HZ = 100                 # joystick polling rate

# ========= Teleop behavior =========
INVERT_Y = False
TH_DOWN_ENTER = -900         # go to CLOSE when Y < this
TH_DOWN_EXIT  = -400         # leave CLOSE when Y > this  (prevents chatter near center)

CLOSE_SPEED = 0xFF           # 0..255 (0x00..0xFF) Robotiq speed
CLOSE_FORCE = 0xFF           # 0..255 force
OPEN_SPEED  = 0xFF
OPEN_FORCE  = 0xFF

# ========= Recording paths =========
HOME = os.path.expanduser("~")
RECORD_DIR = os.path.join(HOME, "robotiq_ws", "recordings")

# ========= FT Sensor Config =========
FT_PORT = "/dev/ttySC1"
FT_SLAVE = 9
FX_REG = 180   # Fx..Mz are 180..185 (signed 16-bit)
ACC_X_REG = 190  # 190..192 for accel
N_FORCE_SCALE = 100.0     # Fx/Fy/Fz in N = value / 100
NM_TORQUE_SCALE = 1000.0  # Mx/My/Mz in Nm = value / 1000
G_ACC_SCALE = 1000.0      # Accel in g = value / 1000

# ========= IMU Config =========
BMI270_I2C_ADDR = 0x68
BMI270_CHIP_ID = 0x00
BMI270_STATUS_REG = 0x03
BMI270_DATA_REG = 0x0C
BMI270_ACC_X = 0x0C
BMI270_ACC_Y = 0x0E
BMI270_ACC_Z = 0x10
BMI270_GYR_X = 0x12
BMI270_GYR_Y = 0x14
BMI270_GYR_Z = 0x16
BMI270_TEMP = 0x22

# ========= Helpers =========
def ctrl_regs(b0,b1,b2,b3,b4,b5):
    return [(b0<<8)|b1, (b2<<8)|b3, (b4<<8)|b5]

class SafeModbusClient:
    def __init__(self, port, baud, unit):
        self.client = ModbusSerialClient(method="rtu", port=port, baudrate=baud,
                                         bytesize=8, parity="N", stopbits=1, timeout=0.3)
        self.unit = unit
        self.lock = threading.Lock()
    def connect(self):
        ok = self.client.connect()
        print("[MODBUS] connect ->", ok)
        return ok
    def write_ctrl(self, vals, label):
        with self.lock:
            print(f"[MODBUS] {label} -> {vals}")
            r = self.client.write_registers(CTRL_ADDR, vals, unit=self.unit)
        ok = (getattr(r, "isError", lambda: False)() is False)
        print(f"[MODBUS] {label} result -> {'OK' if ok else r}")
        return ok
    def read_status(self, count=2):
        with self.lock:
            r = self.client.read_holding_registers(STAT_ADDR, count, unit=self.unit)
        if getattr(r, "isError", lambda: False)():
            return None
        return r.registers
    def close(self):
        with self.lock:
            self.client.close()
            print("[MODBUS] close()")

def activate(safe):
    """Activate and calibrate the Robotiq gripper"""
    print("[INFO] Deactivating gripper...")
    safe.write_ctrl(ctrl_regs(0,0, 0,0, 0,0), "DEACTIVATE"); time.sleep(0.2)
    
    print("[INFO] Activating gripper with calibration...")
    safe.write_ctrl(ctrl_regs(0x09,0, 0,0x00, 0x50,0x50), "ACTIVATE_OPEN"); time.sleep(1.0)
    
    print("[INFO] Calibration close...")
    safe.write_ctrl(ctrl_regs(0x09,0, 0,0xFF, 0x50,0x50), "CALIB_CLOSE"); time.sleep(1.0)
    
    print("[INFO] Calibration open...")
    safe.write_ctrl(ctrl_regs(0x09,0, 0,0x00, 0x50,0x50), "CALIB_OPEN"); time.sleep(1.0)
    
    print("[INFO] Gripper activated and calibrated - ready for joystick control")

def cmd_close(safe, speed=CLOSE_SPEED, force=CLOSE_FORCE):
    return safe.write_ctrl(ctrl_regs(0x09,0, 0,0xFF, speed, force), "CLOSE")

def cmd_open(safe, speed=OPEN_SPEED, force=OPEN_FORCE):
    return safe.write_ctrl(ctrl_regs(0x09,0, 0,0x00, speed, force), "OPEN")

def keepalive(safe, stop_evt):
    while not stop_evt.is_set():
        safe.read_status(2)
        time.sleep(0.1)

# ========= Recording helpers =========
def ensure_record_dir():
    os.makedirs(RECORD_DIR, exist_ok=True)

def next_bag_path():
    fname = datetime.now().strftime("%Y-%m-%d-%H.%M.%S") + ".bag"
    return os.path.join(RECORD_DIR, fname)

# ========= Joystick helpers =========
def reg_read(addr, reg, n):
    with SMBus(I2C_BUS) as bus:
        w = i2c_msg.write(addr, [reg])
        r = i2c_msg.read(addr, n)
        bus.i2c_rdwr(w, r)
        return list(r)

def s16_le(lo, hi):
    v = lo | (hi << 8)
    return v - 65536 if v & 0x8000 else v

def read_xy_int12():
    xL,xH,yL,yH = reg_read(JOY_ADDR, REG_XY_INT12, 4)
    x = s16_le(xL, xH)
    y = s16_le(yL, yH)
    if INVERT_Y:
        y = -y
    return x, y

def read_button_pressed() -> bool:
    try:
        val = reg_read(JOY_ADDR, BTN_REG, 1)[0]
        return (val == 0)
    except Exception as e:
        print(f"[WARN] I2C read failed: {e}")
        time.sleep(0.1)
        return False

def reg_write(addr, reg, data):
    """Write data to I2C register"""
    try:
        with SMBus(I2C_BUS) as bus:
            bus.write_byte_data(addr, reg, data)
        return True
    except Exception as e:
        print(f"[WARN] I2C write failed: {e}")
        return False

def set_led_color(r, g, b):
    """Set LED color using BGR format (Blue-Green-Red)"""
    try:
        reg_write(JOY_ADDR, 0x30, b)  # Blue component
        reg_write(JOY_ADDR, 0x31, g)  # Green component  
        reg_write(JOY_ADDR, 0x32, r)  # Red component
        return True
    except Exception as e:
        print(f"[WARN] LED control failed: {e}")
        return False

def set_led_recording_state(is_recording):
    """Set LED based on recording state"""
    if is_recording:
        set_led_color(0, 255, 0)  # Green (BGR format)
    else:
        set_led_color(255, 0, 0)  # Red (BGR format)

class UnifiedSystemController(Node):
    def __init__(self):
        super().__init__('unified_system_controller')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create publishers for all data streams
        self.infra1_pub = self.create_publisher(Image, '/camera/infra1/image_raw', 10)
        self.infra2_pub = self.create_publisher(Image, '/camera/infra2/image_raw', 10)
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, '/ft_sensor/wrench', 10)
        self.accel_pub = self.create_publisher(Vector3Stamped, '/ft_sensor/acceleration', 10)
        
        # Topics to record
        self.topics = [
            '/camera/infra1/image_raw',
            '/camera/infra2/image_raw', 
            '/camera/color/image_raw',
            '/camera/depth/image_raw',
            '/imu/data',
            '/ft_sensor/wrench',
            '/ft_sensor/acceleration'
        ]
        
        # Recording state
        self.is_recording = False
        self.bag_process = None
        self.current_bag_path = None
        
        # RealSense pipeline
        self.pipeline = None
        self.config = None
        self.is_streaming = False
        
        # Button state
        self.prev_pressed = False
        self.debounce_ms = 150
        self.last_edge_time = 0.0
        
        # Gripper state
        self.gripper_state = "OPEN"  # "OPEN" or "CLOSE"
        self.gripper_safe = None
        self.gripper_keepalive_thread = None
        self.gripper_stop_event = None
        
        # Initialize LED to RED (not recording)
        set_led_recording_state(False)
        
        print("[READY] Unified System Controller initialized")
        print("        - Joystick Y-axis: Pull DOWN to CLOSE gripper, release to OPEN")
        print("        - Joystick button: Press to START/STOP recording all streams")
        print(f"[PATH ] Recordings -> {RECORD_DIR}")
        print("[LED  ] RED = Not recording, GREEN = Recording")
        
        # Initialize gripper
        self.init_gripper()
        
        # Create timer for button checking and streaming
        self.timer = self.create_timer(0.01, self.update_loop)
    
    def init_gripper(self):
        """Initialize the Robotiq gripper"""
        try:
            self.gripper_safe = SafeModbusClient(PORT, BAUD, UNIT)
            if not self.gripper_safe.connect():
                print("[ERROR] Could not connect to gripper")
                return False
            
            # Start keepalive thread
            self.gripper_stop_event = threading.Event()
            self.gripper_keepalive_thread = threading.Thread(
                target=keepalive, 
                args=(self.gripper_safe, self.gripper_stop_event), 
                daemon=True
            )
            self.gripper_keepalive_thread.start()
            
            # Activate and calibrate gripper
            activate(self.gripper_safe)
            print("[INFO] Gripper initialized and ready")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to initialize gripper: {e}")
            return False
    
    def gripper_close(self, speed=CLOSE_SPEED, force=CLOSE_FORCE):
        """Close the gripper"""
        if self.gripper_safe:
            return cmd_close(self.gripper_safe, speed, force)
        return False
    
    def gripper_open(self, speed=OPEN_SPEED, force=OPEN_FORCE):
        """Open the gripper"""
        if self.gripper_safe:
            return cmd_open(self.gripper_safe, speed, force)
        return False
    
    def start_streaming(self):
        """Start RealSense streaming"""
        if self.is_streaming:
            return True
            
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Enable IR streams (left = index 1, right = index 2)
            self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 15)
            self.config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 15)
            
            # Enable Color stream
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)
            
            # Enable Depth stream
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
            
            # Start pipeline
            self.pipeline.start(self.config)
            self.is_streaming = True
            
            print("[INFO] RealSense streaming started")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to start RealSense streaming: {e}")
            return False
    
    def stop_streaming(self):
        """Stop RealSense streaming"""
        if not self.is_streaming:
            return
            
        try:
            if self.pipeline:
                self.pipeline.stop()
            self.pipeline = None
            self.config = None
            self.is_streaming = False
            print("[INFO] RealSense streaming stopped")
        except Exception as e:
            print(f"[WARN] Error stopping RealSense: {e}")
    
    def start_recording(self, bag_path):
        """Start ROS bag recording"""
        if self.is_recording:
            print("[INFO] Already recording.")
            return False
            
        try:
            # Start ros2 bag record process
            cmd = ['ros2', 'bag', 'record', '--output', bag_path] + self.topics
            
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            
            self.is_recording = True
            self.current_bag_path = bag_path
            
            print(f"[INFO] Started recording to: {bag_path}")
            set_led_recording_state(True)
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to start recording: {e}")
            return False
    
    def stop_recording(self):
        """Stop ROS bag recording"""
        if not self.is_recording:
            print("[INFO] Not recording.")
            return
            
        try:
            if self.bag_process:
                # Send SIGINT to ros2 bag record
                self.bag_process.send_signal(signal.SIGINT)
                
                # Wait for process to finish
                try:
                    self.bag_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print("[WARN] Recording process didn't stop gracefully, killing...")
                    self.bag_process.kill()
                    self.bag_process.wait()
                
                self.bag_process = None
            
            self.is_recording = False
            print(f"[OK] Saved: {self.current_bag_path}")
            self.current_bag_path = None
            
            set_led_recording_state(False)
            
        except Exception as e:
            print(f"[WARN] Error stopping recording: {e}")
    
    def publish_frames(self):
        """Publish RealSense frames to ROS topics"""
        if not self.is_streaming:
            return
            
        try:
            # Get frames
            frames = self.pipeline.poll_for_frames()
            if not frames:
                return
                
            # Get individual streams
            infra1_frame = frames.get_infrared_frame(1)
            infra2_frame = frames.get_infrared_frame(2)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            # Convert and publish
            if infra1_frame:
                frame_data = infra1_frame.get_data()
                frame_array = np.asanyarray(frame_data)
                img = self.bridge.cv2_to_imgmsg(frame_array, "mono8")
                img.header.stamp = self.get_clock().now().to_msg()
                img.header.frame_id = "camera_infra1_link"
                self.infra1_pub.publish(img)
            
            if infra2_frame:
                frame_data = infra2_frame.get_data()
                frame_array = np.asanyarray(frame_data)
                img = self.bridge.cv2_to_imgmsg(frame_array, "mono8")
                img.header.stamp = self.get_clock().now().to_msg()
                img.header.frame_id = "camera_infra2_link"
                self.infra2_pub.publish(img)
            
            if color_frame:
                frame_data = color_frame.get_data()
                frame_array = np.asanyarray(frame_data)
                img = self.bridge.cv2_to_imgmsg(frame_array, "rgb8")
                img.header.stamp = self.get_clock().now().to_msg()
                img.header.frame_id = "camera_color_link"
                self.color_pub.publish(img)
            
            if depth_frame:
                frame_data = depth_frame.get_data()
                frame_array = np.asanyarray(frame_data)
                img = self.bridge.cv2_to_imgmsg(frame_array, "16UC1")
                img.header.stamp = self.get_clock().now().to_msg()
                img.header.frame_id = "camera_depth_link"
                self.depth_pub.publish(img)
                
        except Exception as e:
            print(f"[WARN] Error publishing frames: {e}")
    
    def update_loop(self):
        """Main update loop for button checking, gripper control, and streaming"""
        # Read joystick state
        try:
            x, y = read_xy_int12()
            pressed = read_button_pressed()
        except Exception as e:
            print(f"[WARN] Joystick read error: {e}")
            return
        
        now = time.time()
        
        # Gripper control with hysteresis logic
        if self.gripper_safe:
            if self.gripper_state == "OPEN":
                if y < TH_DOWN_ENTER:
                    self.gripper_state = "CLOSE"
                    self.gripper_close()
                    print(f"[JOY ] y={y:5d} -> CLOSE")
            else:  # state == "CLOSE"
                if y > TH_DOWN_EXIT:
                    self.gripper_state = "OPEN"
                    self.gripper_open()
                    print(f"[JOY ] y={y:5d} -> OPEN")
        
        # Recording toggle with debouncing
        if pressed and not self.prev_pressed and (now - self.last_edge_time) * 1000.0 > self.debounce_ms:
            self.last_edge_time = now
            
            if not self.is_recording:
                # Start recording
                if not self.is_streaming:
                    if not self.start_streaming():
                        return
                
                bag_path = next_bag_path()
                if self.start_recording(bag_path):
                    print("[INFO] Recording started")
            else:
                # Stop recording
                self.stop_recording()
                print("[INFO] Recording stopped")
        
        self.prev_pressed = pressed
        
        # Publish frames if streaming
        if self.is_streaming:
            self.publish_frames()
    
    def shutdown(self):
        """Clean shutdown"""
        print("\n[INFO] Shutting down...")
        
        if self.is_recording:
            self.stop_recording()
        
        self.stop_streaming()
        
        # Cleanup gripper
        if self.gripper_safe:
            print("[INFO] Opening gripper before shutdown...")
            self.gripper_open()
            if self.gripper_stop_event:
                self.gripper_stop_event.set()
            if self.gripper_keepalive_thread:
                self.gripper_keepalive_thread.join(timeout=1.0)
            self.gripper_safe.close()
        
        # Turn off LED
        set_led_color(0, 0, 0)

def main():
    """Main entry point"""
    # Ensure recording directory exists
    ensure_record_dir()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run the controller
        controller = UnifiedSystemController()
        
        # Set up signal handler for graceful shutdown
        def signal_handler(sig, frame):
            print("\n[INFO] Ctrl+C received.")
            controller.shutdown()
            rclpy.shutdown()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Spin the node
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt received.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if 'controller' in locals():
            controller.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
