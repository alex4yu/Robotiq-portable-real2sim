#!/usr/bin/env python3
"""
Main launcher for Robotiq gripper system with IMU and recording
Starts all components: joystick teleop, IMU publisher, and recording control
"""

import os
import sys
import time
import subprocess
import signal
import threading
from pathlib import Path

class SystemLauncher:
    def __init__(self):
        self.processes = []
        self.running = True
        
        # Get the directory of this script
        self.script_dir = Path(__file__).parent.absolute()
        
        # Set up signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\nReceived signal {signum}, shutting down...")
        self.running = False
        self.stop_all_processes()
        sys.exit(0)
    
    def start_process(self, cmd, name, cwd=None):
        """Start a subprocess and track it"""
        try:
            print(f"Starting {name}...")
            process = subprocess.Popen(
                cmd,
                shell=True,
                executable='/bin/bash',
                cwd=cwd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            self.processes.append({
                'process': process,
                'name': name,
                'cmd': cmd
            })
            
            # Start a thread to monitor this process
            monitor_thread = threading.Thread(
                target=self.monitor_process,
                args=(process, name),
                daemon=True
            )
            monitor_thread.start()
            
            return process
            
        except Exception as e:
            print(f"Failed to start {name}: {e}")
            return None
    
    def monitor_process(self, process, name):
        """Monitor a process and print its output"""
        try:
            for line in iter(process.stdout.readline, ''):
                if line:
                    print(f"[{name}] {line.strip()}")
        except Exception as e:
            print(f"Error monitoring {name}: {e}")
    
    def start_imu_publisher(self):
        """Start the IMU ROS publisher"""
        print("Starting IMU ROS publisher...")
        
        # Check if ROS 2 is available
        try:
            result = subprocess.run(['which', 'ros2'], capture_output=True, text=True)
            if result.returncode != 0:
                print("ROS 2 not found. Please install ROS 2 Jazzy.")
                return None
        except:
            print("ROS 2 not found. Please install ROS 2 Jazzy.")
            return None
        
        # Create a single bash command that sources ROS 2 (no venv needed)
        cmd = """bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run m5_imu_pro_ros2 imu_publisher'"""
        
        return self.start_process(cmd, "IMU Publisher")
    
    def start_joystick_teleop(self):
        """Start the joystick teleop with LED control"""
        print("Starting joystick teleop with LED control...")
        
        joystick_script = self.script_dir / "joystick_combined_control.py"
        
        if not joystick_script.exists():
            print(f"Joystick script not found: {joystick_script}")
            return None
        
        cmd = f"python3 {joystick_script}"
        return self.start_process(cmd, "Joystick Teleop")
    
    def start_recording_control(self):
        """Start the recording control system"""
        print("Starting recording control...")
        
        # This could be a separate recording manager or integrated with joystick
        # For now, recording is controlled via the joystick button
        print("Recording control is integrated with joystick teleop")
        return None
    
    def check_dependencies(self):
        """Check if all required dependencies are available"""
        print("Checking dependencies...")
        
        # Check Python packages
        required_packages = ['smbus2', 'pymodbus', 'pyrealsense2']
        missing_packages = []
        missing_optional = []
        
        for package in required_packages:
            try:
                __import__(package)
                print(f"✓ {package}")
            except ImportError:
                missing_packages.append(package)
                print(f"✗ {package} - MISSING")
        
        if missing_packages:
            print(f"\nMissing packages: {', '.join(missing_packages)}")
            print("Please install them with: pip install " + " ".join(missing_packages))
            return False
        
        # Check ROS 2
        try:
            result = subprocess.run(['which', 'ros2'], capture_output=True, text=True)
            if result.returncode == 0:
                print("✓ ROS 2")
            else:
                print("✗ ROS 2 - MISSING")
                return False
        except:
            print("✗ ROS 2 - MISSING")
            return False
        
        # Check I2C
        try:
            result = subprocess.run(['ls', '/dev/i2c-1'], capture_output=True, text=True)
            if result.returncode == 0:
                print("✓ I2C")
            else:
                print("✗ I2C - MISSING")
                print("Enable I2C with: sudo raspi-config")
                return False
        except:
            print("✗ I2C - MISSING")
            return False
        
        return True
    
    def stop_all_processes(self):
        """Stop all running processes"""
        print("Stopping all processes...")
        
        for proc_info in self.processes:
            process = proc_info['process']
            name = proc_info['name']
            
            if process and process.poll() is None:
                print(f"Stopping {name}...")
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print(f"Force killing {name}...")
                    process.kill()
                except Exception as e:
                    print(f"Error stopping {name}: {e}")
        
        self.processes.clear()
    
    def run(self):
        """Main run loop"""
        print("=" * 60)
        print("Robotiq Gripper System Launcher")
        print("=" * 60)
        
        # Check dependencies
        if not self.check_dependencies():
            print("\nDependency check failed. Please install missing components.")
            return
        
        print("\nAll dependencies satisfied. Starting components...\n")
        
        # Start IMU publisher
        imu_process = self.start_imu_publisher()
        if not imu_process:
            print("Failed to start IMU publisher")
            return
        
        # Wait a moment for IMU to initialize
        time.sleep(2)
        
        # Start joystick teleop (includes recording control)
        joystick_process = self.start_joystick_teleop()
        if not joystick_process:
            print("Failed to start joystick teleop")
            self.stop_all_processes()
            return
        
        print("\n" + "=" * 60)
        print("All components started successfully!")
        print("=" * 60)
        print("Components running:")
        print("- IMU Publisher: Publishing to /imu/data and /imu/temperature")
        print("- Joystick Teleop: Control gripper with joystick Y-axis")
        print("- Recording Control: Press joystick button to start/stop recording")
        print("- LED Status: RED = not recording, GREEN = recording")
        print("\nPress Ctrl+C to stop all components")
        print("=" * 60)
        
        # Monitor processes
        try:
            while self.running:
                # Check if any process has died
                for proc_info in self.processes[:]:
                    process = proc_info['process']
                    name = proc_info['name']
                    
                    if process.poll() is not None:
                        print(f"Process {name} has stopped (exit code: {process.returncode})")
                        self.processes.remove(proc_info)
                
                # If no processes are running, exit
                if not self.processes:
                    print("All processes have stopped")
                    break
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\nReceived Ctrl+C, shutting down...")
        
        finally:
            self.stop_all_processes()
            print("Shutdown complete.")


def main():
    """Main entry point"""
    launcher = SystemLauncher()
    launcher.run()


if __name__ == "__main__":
    main()
