#!/usr/bin/env python3
"""
ROS2 Force/Torque sensor publisher for FT300-S sensor.

Publishes force/torque data to ROS2 topics:
- /ft_sensor/wrench: geometry_msgs/WrenchStamped (force and torque)
- /ft_sensor/acceleration: geometry_msgs/Vector3Stamped (accelerometer data)

Dependencies:
  pip install minimalmodbus pyserial
"""

import time
import signal
import sys
import threading
from datetime import datetime

import minimalmodbus
import serial

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import WrenchStamped, Vector3Stamped
    from std_msgs.msg import Header
except ImportError:
    print("ROS2 not available. Please install ROS2 Jazzy.")
    sys.exit(1)

# --- Register addresses from FT300-S manual ---
FX_REG = 180   # Fx..Mz are 180..185 (signed 16-bit)
ACC_X_REG = 190  # 190..192 for accel

N_FORCE_SCALE   = 100.0     # Fx/Fy/Fz in N = value / 100
NM_TORQUE_SCALE = 1000.0    # Mx/My/Mz in Nm = value / 1000
G_ACC_SCALE     = 1000.0    # Accel in g = value / 1000

# --- Hard-coded config ---
PORT  = "/dev/ttySC1"
SLAVE = 9

class FTSensorPublisher(Node):
    def __init__(self):
        super().__init__('ft_sensor_publisher')
        
        # Create publishers
        self.wrench_pub = self.create_publisher(WrenchStamped, '/ft_sensor/wrench', 10)
        self.accel_pub = self.create_publisher(Vector3Stamped, '/ft_sensor/acceleration', 10)
        
        # Initialize FT sensor
        self.ft_instrument = None
        self.running = True
        self.publish_rate = 100  # Hz
        
        # Initialize sensor
        if not self.init_ft_sensor():
            self.get_logger().error("Failed to initialize FT sensor")
            return
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_data)
        
        self.get_logger().info("FT Sensor ROS2 Publisher started")
        self.get_logger().info(f"Publishing at {self.publish_rate} Hz")
        self.get_logger().info("Topics: /ft_sensor/wrench, /ft_sensor/acceleration")
    
    def make_instrument(self):
        """Create and configure the FT sensor instrument"""
        try:
            inst = minimalmodbus.Instrument(PORT, SLAVE)
            inst.serial.baudrate = 19200
            inst.serial.bytesize = 8
            inst.serial.parity = serial.PARITY_NONE
            inst.serial.stopbits = 1
            inst.serial.timeout = 0.3
            inst.mode = minimalmodbus.MODE_RTU
            return inst
        except Exception as e:
            self.get_logger().error(f"Failed to create FT sensor instrument: {e}")
            return None
    
    def init_ft_sensor(self):
        """Initialize the FT sensor"""
        try:
            self.ft_instrument = self.make_instrument()
            if self.ft_instrument is None:
                return False
            
            # Test read to verify connection
            self.read_wrench()
            self.get_logger().info("FT sensor initialized successfully")
            return True
        except Exception as e:
            self.get_logger().error(f"FT sensor initialization failed: {e}")
            return False
    
    def read_wrench(self):
        """Read force and torque data from FT sensor"""
        try:
            regs = self.ft_instrument.read_registers(FX_REG, 6, functioncode=3)
            regs = [r - 65536 if r > 32767 else r for r in regs]  # signed16
            fx, fy, fz, mx, my, mz = regs
            return (fx / N_FORCE_SCALE,
                    fy / N_FORCE_SCALE,
                    fz / N_FORCE_SCALE,
                    mx / NM_TORQUE_SCALE,
                    my / NM_TORQUE_SCALE,
                    mz / NM_TORQUE_SCALE)
        except Exception as e:
            self.get_logger().warn(f"FT sensor read error: {e}")
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    def read_acceleration(self):
        """Read accelerometer data from FT sensor"""
        try:
            regs = self.ft_instrument.read_registers(ACC_X_REG, 3, functioncode=3)
            regs = [r - 65536 if r > 32767 else r for r in regs]
            return tuple(r / G_ACC_SCALE for r in regs)
        except Exception as e:
            self.get_logger().warn(f"Accelerometer read error: {e}")
            return (0.0, 0.0, 0.0)
    
    def publish_data(self):
        """Publish FT sensor data to ROS topics"""
        if not self.running or self.ft_instrument is None:
            return
        
        try:
            # Read sensor data
            fx, fy, fz, mx, my, mz = self.read_wrench()
            ax, ay, az = self.read_acceleration()
            
            # Create timestamp
            now = self.get_clock().now()
            
            # Create and publish wrench message
            wrench_msg = WrenchStamped()
            wrench_msg.header = Header()
            wrench_msg.header.stamp = now.to_msg()
            wrench_msg.header.frame_id = "ft_sensor_link"
            
            wrench_msg.wrench.force.x = float(fx)
            wrench_msg.wrench.force.y = float(fy)
            wrench_msg.wrench.force.z = float(fz)
            wrench_msg.wrench.torque.x = float(mx)
            wrench_msg.wrench.torque.y = float(my)
            wrench_msg.wrench.torque.z = float(mz)
            
            self.wrench_pub.publish(wrench_msg)
            
            # Create and publish acceleration message
            accel_msg = Vector3Stamped()
            accel_msg.header = Header()
            accel_msg.header.stamp = now.to_msg()
            accel_msg.header.frame_id = "ft_sensor_link"
            
            accel_msg.vector.x = float(ax)
            accel_msg.vector.y = float(ay)
            accel_msg.vector.z = float(az)
            
            self.accel_pub.publish(accel_msg)
            
            # Optional: Log data periodically (every 1 second)
            if hasattr(self, '_last_log_time'):
                if time.time() - self._last_log_time > 1.0:
                    self.get_logger().info(
                        f"FT: Fx={fx:.3f}N Fy={fy:.3f}N Fz={fz:.3f}N "
                        f"Mx={mx:.4f}Nm My={my:.4f}Nm Mz={mz:.4f}Nm | "
                        f"Accel: ax={ax:.4f}g ay={ay:.4f}g az={az:.4f}g"
                    )
                    self._last_log_time = time.time()
            else:
                self._last_log_time = time.time()
                
        except Exception as e:
            self.get_logger().error(f"Error publishing FT data: {e}")
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down FT sensor publisher...")
        self.running = False
        if self.ft_instrument:
            try:
                self.ft_instrument.serial.close()
            except:
                pass

def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        # Create and run the publisher
        publisher = FTSensorPublisher()
        
        # Set up signal handler for graceful shutdown
        def signal_handler(sig, frame):
            print("\nReceived shutdown signal...")
            publisher.shutdown()
            rclpy.shutdown()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Spin the node
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'publisher' in locals():
            publisher.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
