#!/usr/bin/env python3

from hello_helpers.hello_misc import HelloNode

import numpy as np
import time
import rclpy
from moveit_msgs.srv import GetPositionFK, GetPositionIK
# from rclpy.node import Node


class KinematicsTestClient(HelloNode):
    """
    Test client for the Stretch Kinematics Node.
    """

    def __init__(self) -> None:
        """
        Initialize the test client and its service clients.
        """
        super().__init__()
        self.main("kinematics_test_client_node")
        
        # Create clients FIRST before self.main() so they are registered with the background executor
        self.ik_client = self.create_client(GetPositionIK, "compute_ik")
        self.fk_client = self.create_client(GetPositionFK, "compute_fk")

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for IK service...")

    def sequential_move_to_pose(self, target_dict: dict):
        """
        Perform safe sequential motion toward a target pose.

        Moves Base -> Lift -> Arm -> Wrist for extension, or the reverse for stowing.

        Args:
            target_dict (dict): Target joint positions for the robot.
        """
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

    def sequential_stow(self):
        STOW_POSE = {
            "wrist_yaw_joint": np.pi,
            "wrist_pitch_joint": 0.0,
            "wrist_roll_joint": 0.0,
            "arm_joint": 0.0,
            "lift_joint": 0.3,
        }

        self.get_logger().info("Executing Sequential Stow: Wrist -> Arm -> Lift")

        # Wrist
        self.move_to_pose({
            "wrist_yaw_joint": STOW_POSE["wrist_yaw_joint"],
            "wrist_pitch_joint": STOW_POSE["wrist_pitch_joint"],
            "wrist_roll_joint": STOW_POSE["wrist_roll_joint"],
        })

        # Arm
        self.move_to_pose({"arm_joint": STOW_POSE["arm_joint"]})

        # Lift
        self.move_to_pose({"lift_joint": STOW_POSE["lift_joint"]})

    def _ik_response_to_dict(self, ik_response: GetPositionIK.Response) -> dict:
        """
        Convert an IK response to a dictionary of joint positions.

        Args:
            ik_response: The IK service response.

        Returns:
            dict: A dictionary mapping joint names to their positions.
        """
        joint_dict = {}
        for name, pos in zip(
            ik_response.solution.joint_state.name, ik_response.solution.joint_state.position
        ):
            joint_dict[name] = pos

        # Remove near-zero base translation/rotation values
        for key in ["translate_mobile_base", "rotate_mobile_base"]:
            if key in joint_dict and abs(joint_dict[key]) < 1e-6:
                del joint_dict[key]

        # Check for simultaneous base translation and rotation
        if (
            "translate_mobile_base" in joint_dict
            and "rotate_mobile_base" in joint_dict
        ):
            self.get_logger().error(
                "Simultaneous base translation and rotation is not allowed."
            )
            return {}

        return joint_dict

    def test_ik(self, x: float, y: float, z: float) -> GetPositionIK.Response | None:
        """
        Test the Inverse Kinematics service.

        Args:
            x: Target X position.
            y: Target Y position.
            z: Target Z position.

        Returns:
            The service response or None if failed.
        """
        req = GetPositionIK.Request()
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = float(x)
        req.ik_request.pose_stamped.pose.position.y = float(y)
        req.ik_request.pose_stamped.pose.position.z = float(z)
        req.ik_request.pose_stamped.pose.orientation.w = 1.0  # Neutral

        self.get_logger().info(f"Requesting IK for position: [{x}, {y}, {z}]")
        future = self.ik_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        return future.result()

    def test_fk(self) -> GetPositionFK.Response | None:
        """
        Test the Forward Kinematics service.

        Returns:
            The service response or None if failed.
        """
        req = GetPositionFK.Request()
        req.fk_link_names = ["tool_attachment_site_link"]
        # Use current state (empty robot_state in request)

        self.get_logger().info("Requesting FK for tool_attachment_site_link")
        future = self.fk_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        return future.result()

    def test_sequential_move(
        self, x: float, y: float, z: float
    ) -> GetPositionIK.Response | None:
        """
        Test the Sequential Move service.

        Args:
            x: Target X position.
            y: Target Y position.
            z: Target Z position.

        Returns:
            The service response or None if failed.
        """
        req = GetPositionIK.Request()
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = float(x)
        req.ik_request.pose_stamped.pose.position.y = float(y)
        req.ik_request.pose_stamped.pose.position.z = float(z)
        req.ik_request.pose_stamped.pose.orientation.w = 1.0

        self.get_logger().info(f"Requesting Sequential Move to position: [{x}, {y}, {z}]")
        future = self.ik_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        result = future.result()

        if result:
            self.get_logger().info(
                f"Sequential Move Response: Error Code = {result.error_code.val}"
            )
        else:
            self.get_logger().error("Sequential Move service call failed.")
            return

        self.sequential_stow()  # Stow the robot before starting the test

        # for _ in range(10):
        #     time.sleep(1.0)
        pose = self._ik_response_to_dict(result)
        self.sequential_move_to_pose(pose)

        time.sleep(1.0)  # Allow time for the robot to reach the final pose
        self.sequential_stow()  # Stow the robot after the test


def main(args: list[str] | None = None) -> None:
    """
    Main entry point for the kinematics test client.

    Args:
        args: Command-line arguments.
    """
    client = KinematicsTestClient()

    # FK Test
    fk_res = client.test_fk()
    if fk_res and fk_res.pose_stamped:
        p = fk_res.pose_stamped[0].pose.position
        print(f"FK Result: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

    # IK Test
    ik_res = client.test_ik(0.4, -0.3, 0.8)
    if ik_res and ik_res.error_code.val == ik_res.error_code.SUCCESS:
        print("IK Success!")
        for name, pos in zip(
            ik_res.solution.joint_state.name, ik_res.solution.joint_state.position
        ):
            print(f"  {name}: {pos:.3f}")
    else:
        print(f"IK Failed with code: {ik_res.error_code.val if ik_res else 'None'}")

    # Sequential Move Test
    seq_res = client.test_sequential_move(0.4, -0.3, 1.2)
    if seq_res and seq_res.error_code.val == seq_res.error_code.SUCCESS:
        print("Sequential Move Success!")
        for name, pos in zip(
            seq_res.solution.joint_state.name, seq_res.solution.joint_state.position
        ):
            print(f"  {name}: {pos:.3f}")
    else:
        print(f"Sequential Move Failed with code: {seq_res.error_code.val if seq_res else 'None'}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
