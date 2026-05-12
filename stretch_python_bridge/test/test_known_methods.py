import pytest
import time
from stretch_python_bridge import *

methods = [
    stream_camera_center, stream_camera_center_rotated, stream_camera_center_info,
    stream_camera_left, stream_camera_left_rotated, stream_camera_left_info,
    stream_camera_right, stream_camera_right_rotated, stream_camera_right_info,
    stream_lidar_points_left, stream_lidar_points_right,
    stream_gripper_imu, stream_gripper_right_info, stream_gripper_right,
    stream_gripper_stereo_info, stream_gripper_stereo, stream_gripper_stereo_points
]

def test_exported_methods_exist():
    """Ensure all prepared wrapper methods are exported and callable."""

    for method in methods:
        assert callable(method), f"{method} is not callable"

def test_methods_return_a_frame():
    """
    Test that calling a method returns a frame. 
    Note: This requires lidar and camera launch files to be running.
    """
    for method in methods:
        frame = next(method(timeout=1.0, blocking=True))
        assert frame is not None, f"{method.__name__} returned None, are the cameras and lidars launch files running?"
        assert isinstance(frame, ImageFrame) or isinstance(frame, PointCloudFrame) or isinstance(frame, TransformsFrame) or isinstance(frame, CameraInfoFrame)  or isinstance(frame, ImuFrame), f"{method.__name__} returned an unexpected type {type(frame)}"

def test_manager_stream():
    """
    Test that the stream manager works as expected.
    Note: This requires lidar and camera launch files to be running.
    """
    manager = StreamManager()
    camera_right = stream_camera_right(stream_manager=manager)
    camera_left = stream_camera_left(stream_manager=manager)
    
    c_left = manager.get(camera_left, block=True, timeout=5.0)
    c_right = manager.get(camera_right, block=True, timeout=5.0)
    
    assert c_left is not None 
    assert c_right is not None



def test_stream_methods_timeout_gracefully():
    """Test that calling a method with a short timeout doesn't crash but yields None."""
    # We test an invalid topic to properly trigger the timeout empty state.
    # We test just one to avoid blocking too long.
    gen = image_stream("/test/invalid/topic/camera", timeout=0.1, block=True)
    start_time = time.time()
    
    frame = next(gen)
    
    elapsed = time.time() - start_time
    assert frame is None
    # Assuming slight overhead, it shouldn't take more than 0.5s for a 0.1s timeout
    assert elapsed < 0.5

def test_stream_manager_methods():
    """Test the methods on StreamManager."""
    manager = StreamManager()
    stream_camera_center(stream_manager=manager)
    stream_camera_left(stream_manager=manager)
    stream_camera_right(stream_manager=manager)
    stream_gripper_stereo_points(stream_manager=manager)
    
    # We should be able to configure it without errors
    assert len(manager.latest_frames) == 4
    
    manager.close()

if __name__ == "__main__":
    pytest.main([__file__])