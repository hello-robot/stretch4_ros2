import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

class ParamClient(Node):
    """
    A common test node to change parameters on the stretch_driver.
    Useful for dynamically switching modes or timeouts during tests.
    """
    def __init__(self, name='test_param_client'):
        super().__init__(name)
        self.client = self.create_client(SetParameters, '/stretch_driver/set_parameters')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /stretch_driver/set_parameters not available")

    def set_parameter(self, name, value):
        """
        Sets a parameter on the stretch driver and blocks until complete.
        Returns True if successful, False otherwise.
        """
        req = SetParameters.Request()
        req.parameters = [Parameter(name, value=value).to_parameter_msg()]
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        res = future.result()
        if res is not None and len(res.results) > 0 and res.results[0].successful:
            return True
        return False
