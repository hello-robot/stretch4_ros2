#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import SetParametersResult

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class GripperGoalInterceptor(Node):
    def __init__(self):
        super().__init__('gripper_goal_interceptor')

        self.declare_parameter('gripper_frame', 'wrist_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.add_on_set_parameters_callback(self.on_set_parameters_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/gripper_goal_pose',
            self.goal_callback,
            10
        )

        self.debug_wrist_pub = self.create_publisher(PoseStamped, '/debug/target_wrist_pose', 10)
        self.debug_base_pub = self.create_publisher(PoseStamped, '/debug/target_base_pose', 10)

        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info("Gripper Goal Interceptor Node started (Fixed version).")

    def on_set_parameters_callback(self, params):
        result = SetParametersResult()
        result.successful = True
        for param in params:
            if param.name in ['gripper_frame', 'base_frame', 'global_frame']:
                if param.type_ == Parameter.Type.STRING:
                    self.get_logger().info(f"Parameter '{param.name}' updated to: '{param.value}'")
                else:
                    result.successful = False
                    result.reason = f"{param.name} must be a string"
        return result

    def get_gripper_frame(self): return self.get_parameter('gripper_frame').get_parameter_value().string_value
    def get_base_frame(self): return self.get_parameter('base_frame').get_parameter_value().string_value
    def get_global_frame(self): return self.get_parameter('global_frame').get_parameter_value().string_value

    def goal_callback(self, msg: PoseStamped):
        gripper_frame = self.get_gripper_frame()
        global_frame = self.get_global_frame()
        base_frame = self.get_base_frame()

        # Ensure goal is in global map frame
        if msg.header.frame_id != global_frame:
            try:
                msg = self.tf_buffer.transform(msg, global_frame)
            except Exception as e:
                self.get_logger().error(f"Transform to global frame failed: {e}")
                return

        wrist_goal_map = msg
        self.debug_wrist_pub.publish(wrist_goal_map) 

        # Calculate where the base needs to go
        try:
            # Get current physical offset of base relative to wrist
            wrist_to_base_tf = self.tf_buffer.lookup_transform(
                target_frame=gripper_frame,
                source_frame=base_frame,
                time=rclpy.time.Time()
            )

            # Convert to Pose
            base_in_wrist_pose = PoseStamped()
            base_in_wrist_pose.pose.position.x = wrist_to_base_tf.transform.translation.x
            base_in_wrist_pose.pose.position.y = wrist_to_base_tf.transform.translation.y
            base_in_wrist_pose.pose.position.z = wrist_to_base_tf.transform.translation.z
            base_in_wrist_pose.pose.orientation = wrist_to_base_tf.transform.rotation

            # Create transform from the clicked map goal
            wrist_to_map_tf = TransformStamped()
            wrist_to_map_tf.transform.translation.x = wrist_goal_map.pose.position.x
            wrist_to_map_tf.transform.translation.y = wrist_goal_map.pose.position.y
            wrist_to_map_tf.transform.translation.z = wrist_goal_map.pose.position.z
            wrist_to_map_tf.transform.rotation = wrist_goal_map.pose.orientation

            # Apply map transform to base offset
            base_goal_map_pose = do_transform_pose(base_in_wrist_pose.pose, wrist_to_map_tf)

            # RViz Base Target
            base_goal_msg = PoseStamped()
            base_goal_msg.header.stamp = self.get_clock().now().to_msg()
            base_goal_msg.header.frame_id = global_frame
            base_goal_msg.pose = base_goal_map_pose
            self.debug_base_pub.publish(base_goal_msg)
            
        
            self.send_nav2_goal(base_goal_map_pose, global_frame)

        except Exception as e:
            self.get_logger().error(f"Could not calculate/send base target pose: {e}")

    def send_nav2_goal(self, target_pose, target_frame):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 /navigate_to_pose server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = target_frame
        goal_msg.pose.pose = target_pose

        self.get_logger().info(f"Sending absolute base goal to Nav2 in '{target_frame}' frame.")
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