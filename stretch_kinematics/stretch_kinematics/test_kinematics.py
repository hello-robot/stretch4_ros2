#!/usr/bin/env python3

import rclpy
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node


class KinematicsTestClient(Node):
    """
    Test client for the Stretch Kinematics Node.
    """

    def __init__(self) -> None:
        """
        Initialize the test client and its service clients.
        """
        super().__init__("kinematics_test_client")

        self.ik_client = self.create_client(GetPositionIK, "compute_ik")
        self.fk_client = self.create_client(GetPositionFK, "compute_fk")
        self.seq_move_client = self.create_client(
            GetPositionIK, "sequential_move_to_pose"
        )

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for IK service...")

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
        rclpy.spin_until_future_complete(self, future)
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
        rclpy.spin_until_future_complete(self, future)
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
        future = self.seq_move_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args: list[str] | None = None) -> None:
    """
    Main entry point for the kinematics test client.

    Args:
        args: Command-line arguments.
    """
    rclpy.init(args=args)
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
    seq_res = client.test_sequential_move(0.4, -0.3, 0.8)
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
