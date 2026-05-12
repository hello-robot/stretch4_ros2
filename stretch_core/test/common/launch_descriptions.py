import pytest
import launch
import launch_pytest
from pathlib import Path

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


@launch_pytest.fixture
def stretch_driver_ld():
    """Construct the LaunchDescription with stretch_driver
    """
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                str(Path(get_package_share_directory('stretch_core')) / 'launch' / 'stretch_driver.launch.py'),
            ])
        ),
        # Tell launch when to start the test
        # If no ReadyToTest action is added, one will be appended automatically.
        launch_pytest.actions.ReadyToTest()
    ])
