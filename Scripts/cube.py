import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Empty 
from control_msgs.msg import JointTrajectoryControllerState

import time

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.Subscriber_ = self.create_subscription(JointTrajectoryControllerState,
                                                    "/scaled_joint_trajectory_controller/state",                          
                                                    self.lookup_transform_callback, 
                                                    10)
        
        # Setup Publishers
        self.publisher_goal_ = self.create_publisher(PoseStamped, '/rviz/moveit/move_marker/goal_tool0', 1)
        self.publisher_plan_ = self.create_publisher(Empty, '/rviz/moveit/plan', 1)
        self.publisher_execute_ = self.create_publisher(Empty, '/rviz/moveit/execute', 1)

    def lookup_transform_callback(self, msg):
        if sum(msg.actual.velocities) == 0.0:
            try:
                # Get the transform from 'purple_cube' to 'world'
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    'world', 'grasp_loc', rclpy.time.Time()
                )
    
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'world' # Or your desired frame ID

                # Populate the pose (example: a fixed pose)
                msg.pose.position.x = transform.transform.translation.x
                msg.pose.position.y = transform.transform.translation.y
                msg.pose.position.z = transform.transform.translation.z
                msg.pose.orientation.x = transform.transform.rotation.x
                msg.pose.orientation.y = transform.transform.rotation.y
                msg.pose.orientation.z = transform.transform.rotation.z
                msg.pose.orientation.w = transform.transform.rotation.w

                # Publish the goal pose
                self.publisher_goal_.publish(msg)
                self.get_logger().info(f'Publishing new pose')

                # Build empty message
                empty_msg = Empty()
                time.sleep(0.2)
                #Publish to the plan and execute topics
                self.publisher_plan_.publish(empty_msg)
                self.get_logger().info(f'Planning')
                time.sleep(0.3)
                self.publisher_execute_.publish(empty_msg)
                self.get_logger().info(f'Excecuting')
                
        
            except tf2_ros.TransformException as ex:
                self.get_logger().warn(f"Could not transform: {ex}")

def main(args=None):
    rclpy.init(args=args) 
    node = TfListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()