import rclpy
from rclpy.node import Node

class ServiceClientNode(Node):
    """
    A common test node to call services on the stretch_driver.
    """
    def __init__(self, name='test_service_client_node'):
        super().__init__(name)
        self.clients = {}

    def get_client(self, srv_type, srv_name):
        if srv_name not in self.clients:
            self.clients[srv_name] = self.create_client(srv_type, srv_name)
        return self.clients[srv_name]

    def wait_for_service(self, srv_type, srv_name, timeout_sec=1.0):
        client = self.get_client(srv_type, srv_name)
        return client.wait_for_service(timeout_sec=timeout_sec)

    def call_service(self, srv_type, srv_name, request, timeout_sec=2.0):
        client = self.get_client(srv_type, srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result()
