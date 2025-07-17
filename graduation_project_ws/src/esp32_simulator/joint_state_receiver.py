import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from std_msgs.msg import Float64MultiArray

class JointStateBuffer(Node):
    def __init__(self):
        super().__init__('joint_state_buffer')
        
        # Configuration parameters (easy to change)
        self.update_interval = 0.1  # seconds
        self.change_threshold = 0.01  # radians (approx 0.57 degrees)
        
        # Initialize buffer with empty values
        self.joint_names = []
        self.buffer = []  # List to store position arrays
        self.latest_state = None
        
        # Create subscriber
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for filtered joint states
        self.publisher = self.create_publisher(
            Float64MultiArray,  # Changed message type
            'ESP_joint_state',  # Topic name
            10
        )
        
        # Create timer for periodic checking
        self.timer = self.create_timer(
            self.update_interval,
            self.check_for_changes
        )
        
        self.get_logger().info('Joint state buffer node started')
    
    def joint_state_callback(self, msg):
        """Store the latest joint state"""
        self.latest_state = msg
        # Initialize buffer with first message
        if not self.buffer:
            self.joint_names = msg.name
            self.buffer.append(msg.position)
            self.get_logger().info('Initialized buffer with first joint positions')
            self.print_buffer()
    
    def check_for_changes(self):
        """Check if new state differs from buffer by threshold"""
        if not self.latest_state:
            return
            
        current_positions = self.latest_state.position
        
        # Check if any joint exceeds the threshold
        significant_change = False
        for i, position in enumerate(current_positions):
            if abs(position - self.buffer[-1][i]) > self.change_threshold:
                significant_change = True
                break
                
        if significant_change:
            # Append new positions to buffer
            self.buffer.append(list(current_positions))
            self.get_logger().info(f'Updated buffer with new positions (size: {len(self.buffer)})')
            self.print_buffer()

        # Create and publish Float64MultiArray
            array_msg = Float64MultiArray()
            array_msg.data = list(current_positions)  # Convert to list of floats
            self.publisher.publish(array_msg)
            self.get_logger().info('Published to ESP_joint_state topic')
            
    
    def print_buffer(self):
        """Print buffer in degrees"""
        self.get_logger().info('\nCurrent buffer contents (degrees):')
        
        for idx, positions in enumerate(self.buffer):
            deg_values = [math.degrees(rad) for rad in positions]
            self.get_logger().info(f'Sample {idx}:')
            
            for name, value in zip(self.joint_names, deg_values):
                self.get_logger().info(f'  {name}: {value:.2f}°')
            self.get_logger().info('')

def main():
    rclpy.init()
    buffer_node = JointStateBuffer()
    
    try:
        rclpy.spin(buffer_node)
    except KeyboardInterrupt:
        buffer_node.get_logger().info('Shutting down...')
    finally:
        buffer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()