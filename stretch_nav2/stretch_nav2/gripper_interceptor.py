#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import SetParametersResult

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class GripperGoalInterceptor(Node):
    def __init__(self):
        super().__init__('gripper_goal_interceptor')

        # 1. Declare parameters with defaults
        self.declare_parameter('gripper_frame', 'wrist_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.add_on_set_parameters_callback(self.on_set_parameters_callback)

        # 2. Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 3. Create Subscription to the RViz click topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/gripper_goal_pose',
            self.goal_callback,
            10
        )

        # 4. Create Nav2 Action Client
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info("Gripper Goal Interceptor Node started.")
        self.get_logger().info(f"Tracking Gripper: {self.get_gripper_frame()} | Base: {self.get_base_frame()}")

    def on_set_parameters_callback(self, params):
        """Callback that triggers whenever a parameter is altered via CLI or services."""
        result = SetParametersResult()
        result.successful = True
        
        for param in params:
            if param.name == 'gripper_frame':
                if param.type_ == Parameter.Type.STRING:
                    self.get_logger().info(f"Parameter 'gripper_frame' dynamically updated to: '{param.value}'")
                else:
                    result.successful = False
                    result.reason = "gripper_frame must be a string"
                    
            elif param.name == 'base_frame':
                if param.type_ == Parameter.Type.STRING:
                    self.get_logger().info(f"Parameter 'base_frame' dynamically updated to: '{param.value}'")
                else:
                    result.successful = False
                    result.reason = "base_frame must be a string"
                    
            elif param.name == 'global_frame':
                if param.type_ == Parameter.Type.STRING:
                    self.get_logger().info(f"Parameter 'global_frame' dynamically updated to: '{param.value}'")
                else:
                    result.successful = False
                    result.reason = "global_frame must be a string"
                    
        return result

    def get_gripper_frame(self):
        return self.get_parameter('gripper_frame').get_parameter_value().string_value

    def get_base_frame(self):
        return self.get_parameter('base_frame').get_parameter_value().string_value

    def get_global_frame(self):
        return self.get_parameter('global_frame').get_parameter_value().string_value

    def goal_callback(self, msg: PoseStamped):
        gripper_frame = self.get_gripper_frame()
        global_frame = self.get_global_frame()

        self.get_logger().info(
            f"Received raw goal in {msg.header.frame_id} frame: "
            f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )

        # Ensure the incoming click message matches our expected global frame (usually 'map')
        # If it doesn't, we transform it to the global frame first.
        if msg.header.frame_id != global_frame:
            try:
                msg = self.tf_buffer.transform(msg, global_frame)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"Failed to transform clicked pose to global frame '{global_frame}': {e}")
                return

        # 5. Lookup the link_gripper -> map transform dynamically
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=gripper_frame,
                source_frame=global_frame,
                time=rclpy.time.Time() # Get latest available transform
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Could not look up transform from {global_frame} to {gripper_frame}: {e}")
            return

        # 6. Convert target map coordinates to a relative delta in the gripper's frame
        relative_pose = do_transform_pose(msg.pose, transform)

        self.get_logger().info(
            f"Calculated relative delta in '{gripper_frame}': "
            f"x={relative_pose.position.x:.2f}, y={relative_pose.position.y:.2f}"
        )

        # 7. Construct and send the action goal to Nav2
        self.send_nav2_goal(relative_pose, gripper_frame)

    def send_nav2_goal(self, relative_pose, gripper_frame):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 /navigate_to_pose action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = gripper_frame
        goal_msg.pose.pose = relative_pose

        self.get_logger().info(f"Sending action goal to Nav2 relative to {gripper_frame}...")
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperGoalInterceptor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()