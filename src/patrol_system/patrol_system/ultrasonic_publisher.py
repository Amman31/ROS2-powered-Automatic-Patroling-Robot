import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time
import threading

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic_distance', 10)

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)
            self.get_logger().info("Serial connection with Arduino established")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise

        self.reader_thread = threading.Thread(target=self.read_loop)
        self.reader_thread.daemon = True
        self.reader_thread.start()

    def read_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if not line:
                    continue
                distance = float(line)

                msg = Float32()
                msg.data = distance
                self.publisher_.publish(msg)

                self.get_logger().info(f"Distance: {distance:.1f} cm")

            except ValueError:
                self.get_logger().warn(f"Invalid serial data: {line}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()