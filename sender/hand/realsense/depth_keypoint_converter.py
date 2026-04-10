"""Convert MediaPipe 2D landmarks + RealSense depth → MANO-frame 3D keypoints.

Unlike the phone module's KeypointConverter (which uses MediaPipe's estimated
world_landmarks z-coordinate), this module uses **real depth** from the D405
sensor and ``rs2_deproject_pixel_to_point`` for accurate 3D positions.

After computing xyz from depth, the same **MANO transform** as the phone
module is applied (via :mod:`sensing.core.mano_transform`) so the output is
directly compatible with dex-retargeting's optimizers.

Output: np.ndarray shape (21, 3), float32, meters, wrist = [0, 0, 0],
        aligned to MANO convention.
"""

import numpy as np

from sender.hand.core.mano_transform import apply_mano_transform
from sender.hand.realsense.hand_detector import HandDetection
from sender.hand.realsense.constants import DEPTH_MAX_M, DEPTH_MIN_M, DEPTH_SEARCH_RADIUS


class DepthKeypointConverter:
    """Convert MediaPipe 2D landmarks + depth frame → MANO-frame 3D keypoints.

    Parameters
    ----------
    intrinsics : pyrealsense2.intrinsics
        Intrinsics from the color stream (aligned).
    hand_type : str
        "right" or "left" — determines MANO operator2mano rotation.
    search_radius : int
        Pixel radius for depth neighborhood sampling (median filter).
    apply_mano : bool
        If True (default), apply MANO transform after 3D reconstruction.
    """

    WRIST_INDEX = 0

    def __init__(
        self,
        intrinsics,
        hand_type: str = "right",
        search_radius: int = DEPTH_SEARCH_RADIUS,
        apply_mano: bool = True,
    ):
        self._intrinsics = intrinsics
        self._radius = search_radius
        self._hand_type = hand_type.lower()
        self._apply_mano = apply_mano

        if self._hand_type not in ("right", "left"):
            raise ValueError(f"hand_type must be 'right' or 'left', got: {hand_type}")

    def convert(
        self,
        detection: HandDetection,
        depth_m: np.ndarray,
        img_w: int,
        img_h: int,
    ) -> np.ndarray:
        """Convert 2D landmarks + depth to MANO-frame 3D keypoints.

        Args:
            detection: MediaPipe detection with landmarks_2d and world_landmarks.
            depth_m: Aligned depth image in meters (H, W), float32.
            img_w: Color image width in pixels.
            img_h: Color image height in pixels.

        Returns:
            (21, 3) float32 array in meters, wrist at origin, MANO-aligned.
        """
        import pyrealsense2 as rs

        pts_3d = np.zeros((21, 3), dtype=np.float32)

        for i in range(21):
            nx, ny = detection.landmarks_2d[i, 0], detection.landmarks_2d[i, 1]
            px = int(nx * img_w)
            py = int(ny * img_h)

            # Clamp to image bounds
            px = max(0, min(px, img_w - 1))
            py = max(0, min(py, img_h - 1))

            d = self._sample_depth(depth_m, px, py)

            if DEPTH_MIN_M < d < DEPTH_MAX_M:
                # True 3D via camera intrinsics
                pts_3d[i] = rs.rs2_deproject_pixel_to_point(
                    self._intrinsics, [float(px), float(py)], d,
                )
            else:
                # Fallback to MediaPipe's estimated world_landmarks
                pts_3d[i] = detection.world_landmarks[i]

        # Shift to wrist origin
        pts_3d -= pts_3d[self.WRIST_INDEX]

        # Apply MANO transform (required by dex-retargeting).
        # MediaPipe convention — RealSense uses MediaPipe HandLandmarker for
        # 2D detection, so the input chirality matches phone (and matches
        # dex-retargeting upstream's single_hand_detector.py exactly).
        if self._apply_mano:
            pts_3d = apply_mano_transform(
                pts_3d, hand_type=self._hand_type, convention="mediapipe",
            )

        return pts_3d.astype(np.float32)

    def _sample_depth(self, depth_m: np.ndarray, px: int, py: int) -> float:
        """Sample depth at (px, py) using a neighborhood median for noise rejection.

        Returns depth in meters, or 0.0 if no valid depth found.
        """
        h, w = depth_m.shape[:2]
        r = self._radius

        y0 = max(0, py - r)
        y1 = min(h, py + r + 1)
        x0 = max(0, px - r)
        x1 = min(w, px + r + 1)

        patch = depth_m[y0:y1, x0:x1]
        valid = patch[(patch > DEPTH_MIN_M) & (patch < DEPTH_MAX_M)]

        if len(valid) == 0:
            return 0.0

        return float(np.median(valid))

    @staticmethod
    def extract_2d(detection: HandDetection) -> np.ndarray:
        """Extract normalized 2D image coordinates (21, 2) for visualization."""
        return detection.landmarks_2d[:, :2].astype(np.float32)
