import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sqlite3
import os
from ament_index_python.packages import get_package_share_directory

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(
            String,
            'data_topic',
            self.data_callback,
            10)
        
        # Get the shared directory path for this package in the install space
        package_share_directory = get_package_share_directory('data_transfer')
        
        # Construct the path to `data.db` in the shared directory
        db_path = os.path.join(package_share_directory, 'data.db')
        
        if os.path.exists(db_path):
            os.remove(db_path)
            print(f"{db_path} has been deleted.")
        else:
            print(f"{db_path} does not exist.")

        # Initialize SQLite database at the specified location
        self.conn = sqlite3.connect(db_path)
        self.cursor = self.conn.cursor()
        self.create_table()
        self.get_logger().info(f'Data Subscriber Node has been started and connected to the database at {db_path}.')

    def create_table(self):
        # Create a table if it doesn't exist
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS data_log (
                id INTEGER PRIMARY KEY,
                timestamp TEXT NOT NULL
            )
        ''')
        self.conn.commit()

    def data_callback(self, msg):
        try:
            # Parse the received message
            data = msg.data.split(',')
            data_id = int(data[0])
            timestamp = data[1]

            # Insert the data into the SQLite database
            self.cursor.execute(
                'INSERT INTO data_log (id, timestamp) VALUES (?, ?)', (data_id, timestamp))
            self.conn.commit()
            self.get_logger().info(f'Received and stored data: ID={data_id}, Time={timestamp}')
        except Exception as e:
            self.get_logger().error(f"Failed to store data: {e}")

    def destroy_node(self):
        self.conn.close()  # Close database connection on shutdown
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
