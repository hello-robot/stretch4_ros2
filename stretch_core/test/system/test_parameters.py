from common.client_nodes.param_client import ParamClient
from common.launch_descriptions import stretch_driver_ld
import pytest


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_change_mode() -> None:
    """Verify setting the stretch_driver mode parameter succeeds."""
    node = ParamClient('param_test_node')
    try:
        success = node.set_parameter('mode', 'position')
        assert success, "Mode change failed"
    finally:
        node.destroy_node()


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_timeout_parameter() -> None:
    """Verify setting the stretch_driver action_timeout parameter succeeds."""
    node = ParamClient('param_test_node_2')
    try:
        success = node.set_parameter('action_timeout', 0.5)
        assert success, "Timeout param change failed"
    finally:
        node.destroy_node()
