#!/usr/bin/env python3
"""
RealSense ROS Bag Recorder with Joystick Control

Records RealSense streams (IR1, IR2, Color) to ROS bag files when joystick button is pressed.
- First press  -> START recording to ~/robotiq_ws/recordings/YYYY-MM-DD-HH.MM.SS.bag
- Second press -> STOP recording
- LED shows RED when not recording, GREEN when recording (BGR format)

Dependencies:
  pip install pyrealsense2 smbus2 cv_bridge numpy
"""

import os
import time
import signal
import sys
import subprocess
import threading
from datetime import datetime
from pathlib import Path

import numpy as np
from smbus2 import SMBus, i2c_msg
import pyrealsense2 as rs

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header
    from cv_bridge import CvBridge
except ImportError:
    print("ROS2 not available. Please install ROS2 Jazzy and cv_bridge.")
    sys.exit(1)

# ---------------- Joystick (M5) config ----------------
BUS = 1
ADDR = 0x63
BTN_REG = 0x20          # 1 = not pressed, 0 = pressed (per device docs)

# ---------------- Recording paths ----------------
HOME = os.path.expanduser("~")
RECORD_DIR = os.path.join(HOME, "robotiq_ws", "recordings")

def reg_read(reg, n):
    with SMBus(BUS) as bus:
        w = i2c_msg.write(ADDR, [reg])
        r = i2c_msg.read(ADDR, n)
        bus.i2c_rdwr(w, r)
        return list(r)

def read_button_pressed() -> bool:
    try:
        val = reg_read(BTN_REG, 1)[0]
        return (val == 0)
    except Exception as e:
        print(f"[WARN] I2C read failed: {e}")
        time.sleep(0.1)
        return False

def reg_write(reg, data):
    """Write data to I2C register"""
    try:
        with SMBus(BUS) as bus:
            bus.write_byte_data(ADDR, reg, data)
        return True
    except Exception as e:
        print(f"[WARN] I2C write failed: {e}")
        return False

def set_led_color(r, g, b):
    """Set LED color using BGR format (Blue-Green-Red)"""
    try:
        # Use BGR format for M5Stack JS2 joystick
        reg_write(0x30, b)  # Blue component
        reg_write(0x31, g)  # Green component
        reg_write(0x32, r)  # Red component
        return True
    except Exception as e:
        print(f"[WARN] LED control failed: {e}")
        return False

def set_led_recording_state(is_recording):
    """Set LED based on recording state"""
    if is_recording:
        # Recording - show GREEN
        set_led_color(0, 255, 0)  # Green (BGR format)
    else:
        # Not recording - show RED
        set_led_color(255, 0, 0)  # Red (BGR format)

def ensure_record_dir():
    os.makedirs(RECORD_DIR, exist_ok=True)

def next_bag_path():
    # Filename format: YYYY-MM-DD-HH.MM.SS.bag
    fname = datetime.now().strftime("%Y-%m-%d-%H.%M.%S") + ".bag"
    return os.path.join(RECORD_DIR, fname)

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        
        # Create publishers
        self.infra1_pub = self.create_publisher(Image, '/camera/infra1/image_raw', 10)
        self.infra2_pub = self.create_publisher(Image, '/camera/infra2/image_raw', 10)
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        
        # Initialize RealSense
        self.pipeline = None
        self.config = None
        self.is_streaming = False
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Topics to record
        self.topics = [
            '/camera/infra1/image_raw',
            '/camera/infra2/image_raw', 
            '/camera/color/image_raw',
            '/camera/depth/image_raw'
        ]
        
        # Recording state
        self.is_recording = False
        self.bag_process = None
        self.current_bag_path = None
        
        # Button state
        self.prev_pressed = False
        self.debounce_ms = 150
        self.last_edge_time = 0.0
        
        # Initialize LED to RED (not recording)
        set_led_recording_state(False)
        
        print("[READY] Press joystick button to START/STOP recording (IR1, IR2, Color).")
        print(f"[PATH ] Recordings -> {RECORD_DIR}")
        print("[LED  ] RED = Not recording, GREEN = Recording")
        
        # Create timer for button checking and streaming
        self.timer = self.create_timer(0.01, self.update_loop)
    
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
                # Convert RealSense frame to numpy array
                frame_data = infra1_frame.get_data()
                frame_array = np.asanyarray(frame_data)
                img = self.bridge.cv2_to_imgmsg(frame_array, "mono8")
                img.header.stamp = self.get_clock().now().to_msg()
                img.header.frame_id = "camera_infra1_link"
                self.infra1_pub.publish(img)
            
            if infra2_frame:
                # Convert RealSense frame to numpy array
                frame_data = infra2_frame.get_data()
                frame_array = np.asanyarray(frame_data)
                img = self.bridge.cv2_to_imgmsg(frame_array, "mono8")
                img.header.stamp = self.get_clock().now().to_msg()
                img.header.frame_id = "camera_infra2_link"
                self.infra2_pub.publish(img)
            
            if color_frame:
                # Convert RealSense frame to numpy array
                frame_data = color_frame.get_data()
                frame_array = np.asanyarray(frame_data)
                img = self.bridge.cv2_to_imgmsg(frame_array, "rgb8")
                img.header.stamp = self.get_clock().now().to_msg()
                img.header.frame_id = "camera_color_link"
                self.color_pub.publish(img)
            
            if depth_frame:
                # Convert RealSense frame to numpy array
                frame_data = depth_frame.get_data()
                frame_array = np.asanyarray(frame_data)
                img = self.bridge.cv2_to_imgmsg(frame_array, "16UC1")
                img.header.stamp = self.get_clock().now().to_msg()
                img.header.frame_id = "camera_depth_link"
                self.depth_pub.publish(img)
                
        except Exception as e:
            print(f"[WARN] Error publishing frames: {e}")
    
    def update_loop(self):
        """Main update loop for button checking and streaming"""
        # Check button state
        pressed = read_button_pressed()
        now = time.time()
        
        # Rising edge detection with debounce
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
        
        # Turn off LED
        set_led_color(0, 0, 0)

def main():
    """Main entry point"""
    # Ensure recording directory exists
    ensure_record_dir()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run the publisher
        publisher = RealSensePublisher()
        
        # Set up signal handler for graceful shutdown
        def signal_handler(sig, frame):
            print("\n[INFO] Ctrl+C received.")
            publisher.shutdown()
            rclpy.shutdown()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Spin the node
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt received.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if 'publisher' in locals():
            publisher.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
