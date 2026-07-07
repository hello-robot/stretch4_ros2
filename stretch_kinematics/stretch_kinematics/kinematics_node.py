#!/usr/bin/env python3

import time
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from hello_helpers.hello_misc import HelloNode
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState

from stretch4_kinematics.kinematic_models.base_kinematic_models import StretchKinematics
from stretch4_kinematics.state.joint_positions import StretchJointPositions


class KinematicsNode(HelloNode):
    """
    ROS 2 node for providing kinematics services for the Stretch 4 robot.

    This node wraps the StretchKinematics solver and exposes Forward/Inverse
    Kinematics through standard moveit_msgs services. It also provides a
    sequential motion service for safe joint-by-joint movements.
    """

    def __init__(self) -> None:
        """
        Initialize the KinematicsNode and its services.
        """
        super().__init__()
        # Note: self.main initializes the HelloNode basics (subscribers, etc.)
        self.main("stretch_kinematics_node")

        self.solver = StretchKinematics()
        self.default_target_frame = "tool_attachment_site_link"

        # Use a ReentrantCallbackGroup for all services to allow them to call
        # other services/actions (like switch_mode and move_to_pose) without deadlocking.
        self.cb_group = ReentrantCallbackGroup()

        # IK Service
        self.ik_srv = self.create_service(
            GetPositionIK,
            "compute_ik",
            self.callback_compute_ik,
            callback_group=self.cb_group,
        )

        # FK Service
        self.fk_srv = self.create_service(
            GetPositionFK,
            "compute_fk",
            self.callback_compute_fk,
            callback_group=self.cb_group,
        )

        # Sequential Move Service (using IK message type as requested)
        self.seq_move_srv = self.create_service(
            GetPositionIK,
            "sequential_move_to_pose",
            self.callback_sequential_move,
            callback_group=self.cb_group,
        )

        self.get_logger().info("Stretch Kinematics Node Initialized.")

    def callback_compute_ik(
        self,
        request: GetPositionIK.Request,
        response: GetPositionIK.Response,
    ) -> GetPositionIK.Response:
        """
        Compute Inverse Kinematics using StretchKinematics.

        Args:
            request: The service request containing the target pose.
            response: The service response to be populated.

        Returns:
            The populated service response with the joint solution.
        """
        target_pose_stamped = request.ik_request.pose_stamped
        target_frame = (
            request.ik_request.ik_link_name
            if request.ik_request.ik_link_name
            else self.default_target_frame
        )

        # Convert ROS pose to Pinocchio SE3
        p = target_pose_stamped.pose.position
        q = target_pose_stamped.pose.orientation
        translation = np.array([p.x, p.y, p.z])
        # Pinocchio expects [x, y, z, w] for some functions, but SE3 constructor takes rotation matrix
        quat = pin.Quaternion(q.w, q.x, q.y, q.z)
        target_se3 = pin.SE3(quat.matrix(), translation)

        # Use current joint state as guess if not provided
        q_guess = self._ros_joint_state_to_stretch_pos(self.joint_state)

        try:
            solution = self.solver.inverse_6dof_local(
                target_frame=target_frame,
                target_pose=target_se3,
                q_guess=q_guess,
            )

            # Populate response
            response.solution.joint_state = self._stretch_pos_to_ros_joint_state(solution)
            response.error_code.val = response.error_code.SUCCESS
        except Exception as e:
            self.get_logger().error(f"IK solver failed: {e}")
            response.error_code.val = response.error_code.NO_IK_SOLUTION

        return response

    def callback_compute_fk(
        self,
        request: GetPositionFK.Request,
        response: GetPositionFK.Response,
    ) -> GetPositionFK.Response:
        """
        Compute Forward Kinematics using StretchKinematics.

        Args:
            request: The service request containing links and joint states.
            response: The service response to be populated.

        Returns:
            The populated service response with Cartesian poses.
        """
        target_frames = (
            request.fk_link_names
            if request.fk_link_names
            else [self.default_target_frame]
        )
        joint_state = request.robot_state.joint_state

        if not joint_state.name:
            stretch_pos = self._ros_joint_state_to_stretch_pos(self.joint_state)
        else:
            stretch_pos = self._ros_joint_state_to_stretch_pos(joint_state)

        for frame in target_frames:
            try:
                pose_se3 = self.solver.forward(stretch_pos, frame)

                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "base_link"  # Solvers are local
                pose_stamped.header.stamp = self.get_clock().now().to_msg()

                pose_stamped.pose.position.x = pose_se3.translation[0]
                pose_stamped.pose.position.y = pose_se3.translation[1]
                pose_stamped.pose.position.z = pose_se3.translation[2]

                quat = pin.Quaternion(pose_se3.rotation)
                pose_stamped.pose.orientation.x = quat.x
                pose_stamped.pose.orientation.y = quat.y
                pose_stamped.pose.orientation.z = quat.z
                pose_stamped.pose.orientation.w = quat.w

                response.pose_stamped.append(pose_stamped)
                response.fk_link_names.append(frame)
            except Exception as e:
                self.get_logger().error(f"FK solver failed for frame {frame}: {e}")

        if response.pose_stamped:
            response.error_code.val = response.error_code.SUCCESS
        else:
            response.error_code.val = response.error_code.FAILURE

        return response

    def callback_sequential_move(
        self,
        request: GetPositionIK.Request,
        response: GetPositionIK.Response,
    ) -> GetPositionIK.Response:
        """
        Perform safe sequential motion toward a target pose.

        Moves Base -> Lift -> Arm -> Wrist for extension, or the reverse for stowing.

        Args:
            request: The service request containing the target pose.
            response: The service response to be populated.

        Returns:
            The populated service response indicating success or failure.
        """
        # First compute IK to get the target joint positions
        ik_resp = self.callback_compute_ik(request, GetPositionIK.Response())

        if ik_resp.error_code.val != ik_resp.error_code.SUCCESS:
            response.error_code.val = ik_resp.error_code.val
            return response

        target_joint_state = ik_resp.solution.joint_state
        target_dict = dict(zip(target_joint_state.name, target_joint_state.position))

        # Determine if we are extending or stowing (heuristic: check arm extension)
        current_stretch_pos = self._ros_joint_state_to_stretch_pos(self.joint_state)
        target_stretch_pos = self._ros_joint_state_to_stretch_pos(target_joint_state)

        # Let's check if the target arm is less than current arm -> likely stowing.
        is_stowing = target_stretch_pos.arm < current_stretch_pos.arm

        self.get_logger().info("Switching to position mode...")
        self.switch_to_position_mode()

        if not is_stowing:
            self.get_logger().info(
                "Executing Sequential Extension: Base -> Lift -> Arm -> Wrist"
            )
            # Base motion
            base_goal = {}
            if "translate_mobile_base" in target_dict:
                base_goal["translate_mobile_base"] = target_dict["translate_mobile_base"]
            if "rotate_mobile_base" in target_dict:
                base_goal["rotate_mobile_base"] = target_dict["rotate_mobile_base"]
            if base_goal:
                self.move_to_pose(base_goal)

            # Lift
            if "lift_joint" in target_dict:
                self.move_to_pose({"lift_joint": target_dict["lift_joint"]})

            # Arm
            if "arm_joint" in target_dict:
                self.move_to_pose({"arm_joint": target_dict["arm_joint"]})

            # Wrist
            wrist_goal = {}
            for joint in ["wrist_yaw_joint", "wrist_pitch_joint", "wrist_roll_joint"]:
                if joint in target_dict:
                    wrist_goal[joint] = target_dict[joint]
            if wrist_goal:
                self.move_to_pose(wrist_goal)
        else:
            self.get_logger().info(
                "Executing Sequential Stow: Wrist -> Arm -> Lift -> Base"
            )
            # Wrist
            wrist_goal = {}
            for joint in ["wrist_yaw_joint", "wrist_pitch_joint", "wrist_roll_joint"]:
                if joint in target_dict:
                    wrist_goal[joint] = target_dict[joint]
            if wrist_goal:
                self.move_to_pose(wrist_goal)

            # Arm
            if "arm_joint" in target_dict:
                self.move_to_pose({"arm_joint": target_dict["arm_joint"]})

            # Lift
            if "lift_joint" in target_dict:
                self.move_to_pose({"lift_joint": target_dict["lift_joint"]})

            # Base
            base_goal = {}
            if "translate_mobile_base" in target_dict:
                base_goal["translate_mobile_base"] = target_dict["translate_mobile_base"]
            if "rotate_mobile_base" in target_dict:
                base_goal["rotate_mobile_base"] = target_dict["rotate_mobile_base"]
            if base_goal:
                self.move_to_pose(base_goal)

        response.error_code.val = response.error_code.SUCCESS
        return response

    def _ros_joint_state_to_stretch_pos(
        self,
        ros_joint_state: JointState,
    ) -> StretchJointPositions:
        """
        Convert a ROS JointState message to a StretchJointPositions object.

        Args:
            ros_joint_state: The ROS joint state message.

        Returns:
            The converted StretchJointPositions object.
        """
        joint_dict = dict(zip(ros_joint_state.name, ros_joint_state.position))

        # Lift
        lift = joint_dict.get("lift_joint", 0.5)

        # Arm
        if "arm_l4_joint" in joint_dict:
            arm = joint_dict["arm_l4_joint"] * 5.0
        elif "arm_joint" in joint_dict:
            arm = joint_dict["arm_joint"]
        else:
            arm = 0.0

        # Wrists
        wrist_yaw = joint_dict.get("wrist_yaw_joint", 0.0)
        wrist_pitch = joint_dict.get("wrist_pitch_joint", 0.0)
        wrist_roll = joint_dict.get("wrist_roll_joint", 0.0)

        # Base (local odom)
        base_x = joint_dict.get("translate_mobile_base", 0.0)
        base_theta = joint_dict.get("rotate_mobile_base", 0.0)

        return StretchJointPositions(
            base_x=base_x,
            base_theta=base_theta,
            lift=lift,
            arm=arm,
            wrist_yaw=wrist_yaw,
            wrist_pitch=wrist_pitch,
            wrist_roll=wrist_roll,
        )

    def _stretch_pos_to_ros_joint_state(
        self,
        stretch_pos: StretchJointPositions,
    ) -> JointState:
        """
        Convert a StretchJointPositions object to a ROS JointState message.

        Args:
            stretch_pos: The StretchJointPositions object.

        Returns:
            The converted ROS JointState message.
        """
        js = JointState()
        js.name = [
            "translate_mobile_base",
            "rotate_mobile_base",
            "lift_joint",
            "arm_joint",
            "wrist_yaw_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
        ]
        js.position = [
            float(stretch_pos.base_x),
            float(stretch_pos.base_theta),
            float(stretch_pos.lift),
            float(stretch_pos.arm),
            float(stretch_pos.wrist_yaw),
            float(stretch_pos.wrist_pitch),
            float(stretch_pos.wrist_roll),
        ]
        return js


def main() -> None:
    """
    Main entry point for the kinematics node.
    """
    node = KinematicsNode()
    # HelloNode.main handles spinning in a thread, so we just wait for shutdown.
    # Avoid calling rclpy.spin_once(node) here as HelloNode already has an executor.
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
