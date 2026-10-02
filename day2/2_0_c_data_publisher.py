import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import time

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')
        self.publisher_ = self.create_publisher(String, 'data_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        self.id_counter = 1
        self.get_logger().info('Data Publisher Node has been started.')

    def publish_data(self):
        current_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        message = f"{self.id_counter},{current_time}"
        self.publisher_.publish(String(data=message))
        self.get_logger().info(f'Published data: ID={self.id_counter}, Time={current_time}')
        self.id_counter += 1  # Increment ID for the next message

def main(args=None):
    rclpy.init(args=args)
    node = DataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
