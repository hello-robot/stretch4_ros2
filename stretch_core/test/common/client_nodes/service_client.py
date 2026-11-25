from typing import Any

import rclpy
from rclpy.client import Client
from rclpy.node import Node


class ServiceClientNode(Node):
    """A common test node to call services on stretch_driver."""

    def __init__(self, name: str = 'test_service_client_node') -> None:
        """
        Initialize the service client node and client cache.

        Args:
            name: ROS 2 node name.
        """
        super().__init__(name)
        self._service_clients: dict[str, Client] = {}

    def get_client(self, srv_type: Any, srv_name: str) -> Client:
        """
        Return a cached service client for ``srv_name``, creating it if needed.

        Args:
            srv_type: ROS 2 service message type.
            srv_name: Name of the service.

        Returns:
            The ``rclpy.client.Client`` for ``srv_name``.
        """
        if srv_name not in self._service_clients:
            self._service_clients[srv_name] = self.create_client(srv_type, srv_name)
        return self._service_clients[srv_name]

    def wait_for_service(self, srv_type, srv_name, timeout_sec=1.0):
        client = self.get_client(srv_type, srv_name)
        return client.wait_for_service(timeout_sec=timeout_sec)

    def call_service(self, srv_type, srv_name, request, timeout_sec=2.0):
        client = self.get_client(srv_type, srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result()
