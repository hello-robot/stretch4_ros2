import pytest
import rclpy

from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.service_client import ServiceClientNode
from std_srvs.srv import Trigger, SetBool

@pytest.fixture
def service_client_node():
    rclpy.init()
    node = ServiceClientNode('test_services_client_node')
    yield node
    node.destroy_node()

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_stop_home_stow_runstop_services(service_client_node):
    """
    Test sending requests to the basic operational services exposed by
    stretch_driver. Verifies that the services are available and respond
    with a success=True.
    """
    assert service_client_node.wait_for_service(Trigger, '/stop_the_robot', timeout_sec=10.0), "stop_the_robot service not available"
    assert service_client_node.wait_for_service(Trigger, '/home_the_robot'), "home_the_robot service not available"
    assert service_client_node.wait_for_service(Trigger, '/stow_the_robot'), "stow_the_robot service not available"
    assert service_client_node.wait_for_service(SetBool, '/runstop_the_robot'), "runstop_the_robot service not available"

    # Test stop_the_robot
    req = Trigger.Request()
    res = service_client_node.call_service(Trigger, '/stop_the_robot', req)
    assert res is not None, "Failed to get response from stop_the_robot"
    assert res.success == True
    assert res.message == "Stopped the robot."

    # Test runstop_the_robot (enable runstop)
    req = SetBool.Request()
    req.data = True
    res = service_client_node.call_service(SetBool, '/runstop_the_robot', req)
    assert res is not None, "Failed to get response from runstop_the_robot (enable)"
    assert res.success == True
    assert res.message == "is_runstopped: True"

    # Test runstop_the_robot (disable runstop)
    req.data = False
    res = service_client_node.call_service(SetBool, '/runstop_the_robot', req)
    assert res is not None, "Failed to get response from runstop_the_robot (disable)"
    assert res.success == True
    assert res.message == "is_runstopped: False"

    # We do not call home or stow services in this baseline test to avoid 
    # unwanted actual robot motion when these tests are run by default.
    # Calling wait_for_service confirms they exist and were initialized.

    rclpy.shutdown()
