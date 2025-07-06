import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import subprocess
import requests
import serial
import time

THRESHOLD_CM = 40
SERVER_URL = 'http://192.168.0.16:5000/upload'
IMAGE_PATH = '/tmp/obstacle.jpg'

class CameraMonitor(Node):
    def __init__(self):
        super().__init__('camera_monitor')
        self.subscription = self.create_subscription(
            Float32,
            'ultrasonic_distance',
            self.listener_callback,
            10)

        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)
        self.alert_triggered = False

    def listener_callback(self, msg):
        distance = msg.data

        if distance < THRESHOLD_CM:
            if not self.alert_triggered:
                self.get_logger().warn(f"Obstacle very close ({distance:.1f} cm). Sending STOP.")
                self.ser.write(b'S\n')  # Stop robot
                self.take_picture()
                self.alert_triggered = True
            else:
                self.ser.write(b'S\n')  # Keep stopping while still blocked

        elif self.alert_triggered and distance >= THRESHOLD_CM:
            self.get_logger().info("Obstacle cleared. Sending RESUME.")
            self.ser.write(b'R\n')  # Resume robot
            self.alert_triggered = False

    def take_picture(self):
        subprocess.run(['libcamera-still', '-o', IMAGE_PATH, '-n'])
        try:
            with open(IMAGE_PATH, 'rb') as f:
                response = requests.post(SERVER_URL, files={'image': f})
                if response.status_code == 200:
                    self.get_logger().info("Image sent successfully.")
                else:
                    self.get_logger().error(f"Upload failed: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Camera/server error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
