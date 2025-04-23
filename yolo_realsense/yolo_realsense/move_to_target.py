import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped

class MoveToTarget(Node):
    def __init__(self):
        super().__init__('move_to_target')
        
        # Subscribe to target point
        self.subscription = self.create_subscription(
            PointStamped, "/yolo/target_point", self.target_callback, 10
        )
        
        # Publisher for robot navigation goal
        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)

    def target_callback(self, msg):
        self.get_logger().info(f"Received Target: X={msg.point.x:.3f}, Y={msg.point.y:.3f}, Z={msg.point.z:.3f}")
        
        # Create a navigation goal
        goal_msg = PoseStamped()
        goal_msg.header = msg.header  # Keep the same timestamp and frame_id
        goal_msg.pose.position = msg.point  # Use the detected position
        
        # Set default orientation (no rotation)
        goal_msg.pose.orientation.w = 1.0  

        # Publish the goal
        self.goal_pub.publish(goal_msg)
        self.get_logger().info("Sent Goal to Navigation Stack")

def main(args=None):
    rclpy.init(args=args)
    node = MoveToTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
