import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

class CustomTwistMux(Node):
    def __init__(self):
        super().__init__('custom_twist_mux')
        
        # Declare parameters
        self.declare_parameter('sources', ['cmd_vel_source1', 'cmd_vel_source2'])
        self.declare_parameter('priorities', [1, 2])
        self.declare_parameter('cmd_vel_out', 'cmd_vel_out')
        
        # Get parameters
        sources = self.get_parameter('sources').get_parameter_value().string_array_value
        priorities = self.get_parameter('priorities').get_parameter_value().integer_array_value
        cmd_vel_out = self.get_parameter('cmd_vel_out').get_parameter_value().string_value
        
        if len(sources) != len(priorities):
            self.get_logger().error("The length of sources and priorities must be equal")
            return
        
        # Create dictionaries to store the twist messages and priorities
        self.twist_sources = {source: Twist() for source in sources}
        self.priorities = {source: priority for source, priority in zip(sources, priorities)}
        self.last_msg_times = {source: self.get_clock().now()  for source in sources}
    
        self.publisher_ = self.create_publisher(Twist, cmd_vel_out, 10)
        
        self.get_logger().info(f"self.priorities = {self.priorities}")
        self.get_logger().info(f"self.twist_sources = {self.twist_sources}")
        self.get_logger().info(f"self.last_msg_times {self.last_msg_times}")
        
        # Create subscribers for each source
        for source in sources:
            self.create_subscription(Twist, source, self.create_callback(source), 10)
            
        
    def create_callback(self, source):
        def callback(msg):
            self.last_msg_times[source] = self.get_clock().now()
            self.twist_sources[source] = msg
            self.publish_twist(source)
        return callback

    def publish_twist(self, current_source):
        # Select the twist message from the highest priority source that has non-zero values
        highest_priority = float('inf')
        selected_twist = Twist()
        selected_source = None
        time_between_msgs = 70000000       # (nanoseconds) Time during which two speed messages are considered to be in conflict
        
        for source, priority in self.priorities.items():
            current_time = self.get_clock().now()
            if(current_time - self.last_msg_times[source] < Duration(seconds=0, nanoseconds=time_between_msgs) and priority < highest_priority):
                highest_priority = priority
                selected_twist = self.twist_sources[source]
                selected_source = source
        
        if selected_source is not None:
            # self.get_logger().info(f"Selected source: {selected_source}")
            # self.get_logger().info(f"Current source: {current_source}")
            if selected_source == current_source:
                self.publisher_.publish(selected_twist)

def main(args=None):
    rclpy.init(args=args)
    node = CustomTwistMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
