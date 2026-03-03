import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(
            String,
            'data_topic',
            self.data_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Data Subscriber Node has been started.')

    def data_callback(self, msg):
        # Parse the received message, expecting the format: "id,timestamp"
        data_parts = msg.data.split(',')
        if len(data_parts) == 2:
            data_id = data_parts[0].strip()
            timestamp = data_parts[1].strip()
            self.get_logger().info(f'Received data: ID={data_id}, Time={timestamp}')
        else:
            self.get_logger().warn('Received malformed data')

def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
