"""Undistortion of camera frames, driven by the calibration stretch4_body holds for each camera."""

import numpy as np

from stretch4_body.subsystem.cameras.cv_utils import (
    RectifyMaps,
    get_recify_maps,
    rectify,
)
from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

# The same values stretch4_body's own camera pipeline rectifies with.
DEFAULT_BALANCE = 0.0
DEFAULT_FOV_SCALE = 0.8


class CameraRectifier:
    """Rectifies frames from one camera according to its calibration's distortion model.

    The fisheye path needs remap tables sized to the image, so they are built from the first frame
    and reused for every frame after it. The other models go straight through OpenCV and need no
    setup.
    """

    def __init__(
        self,
        camera_type: RGBCameras,
        *,
        balance: float = DEFAULT_BALANCE,
        fov_scale: float = DEFAULT_FOV_SCALE,
    ):
        self.camera_type = camera_type
        self.calibration = camera_type.load_calibration()
        if self.calibration is None:
            raise RuntimeError(f"No calibration is available for {camera_type.name}.")

        self.balance = balance
        self.fov_scale = fov_scale
        self.rectify_maps: RectifyMaps | None = None

    @property
    def needs_rectify_maps(self) -> bool:
        """Whether this camera rectifies through remap tables. Omnidir has its own undistort."""
        distortion_model = self.calibration.distortion_model
        return distortion_model.is_fisheye() and distortion_model is not DistortionModels.omnidir

    def rectify(self, image: np.ndarray) -> np.ndarray:
        """The image undistorted. The input is left untouched."""
        if self.needs_rectify_maps and self.rectify_maps is None:
            self.rectify_maps = get_recify_maps(
                image,
                sim_cam_matrix=self.calibration.camera_matrix,
                sim_cam_distortion_coeffs=self.calibration.distortion_coefficients,
                balance=self.balance,
                fov_scale=self.fov_scale,
            )

        return rectify(
            image,
            self.calibration.camera_matrix,
            self.calibration.distortion_coefficients,
            self.calibration.distortion_model,
            rectify_maps=self.rectify_maps,
        )
