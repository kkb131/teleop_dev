"""OpenCV-based debug visualization for hand landmarks.

Draws keypoints, bone connections, FPS/latency info, and 3D depth bars
on the camera frame.
"""

import cv2
import numpy as np

from sender.hand.realsense.hand_keypoints import (
    BONE_CONNECTIONS,
    FINGER_COLORS_BGR,
    FINGER_GROUPS,
    HandKeypoints,
)


def _finger_color(kp_idx: int) -> tuple[int, int, int]:
    """Return BGR color for a keypoint index based on its finger."""
    for i, (_, indices) in enumerate(FINGER_GROUPS.items()):
        if kp_idx in indices:
            return FINGER_COLORS_BGR[i]
    return (200, 200, 200)  # wrist = gray


class HandVisualizer:
    """OpenCV debug overlay for hand landmarks on camera frames."""

    def __init__(self, window_name: str = "RealSense Hand Sensing"):
        self._window_name = window_name

    def draw(
        self,
        frame: np.ndarray,
        keypoints: HandKeypoints | None = None,
        fps: float = 0.0,
        detection_ms: float = 0.0,
    ) -> np.ndarray:
        """Draw landmarks and info on a copy of the frame.

        Args:
            frame: BGR camera frame.
            keypoints: Detected hand keypoints (may be None).
            fps: Current capture FPS.
            detection_ms: Detection time in milliseconds.

        Returns:
            Annotated BGR frame (copy).
        """
        out = frame.copy()
        h, w = out.shape[:2]

        if keypoints is not None and keypoints.keypoints_2d is not None:
            pts_2d = keypoints.keypoints_2d  # (21, 2) normalized
            # Convert to pixel coordinates
            px = (pts_2d[:, 0] * w).astype(int)
            py = (pts_2d[:, 1] * h).astype(int)

            # Draw bones
            for i, j in BONE_CONNECTIONS:
                color = _finger_color(j)
                cv2.line(out, (px[i], py[i]), (px[j], py[j]), color, 2, cv2.LINE_AA)

            # Draw keypoints
            for idx in range(len(px)):
                color = _finger_color(idx)
                cv2.circle(out, (px[idx], py[idx]), 5, color, -1, cv2.LINE_AA)
                cv2.circle(out, (px[idx], py[idx]), 5, (255, 255, 255), 1, cv2.LINE_AA)

            # Handedness label
            label = f"{keypoints.handedness.upper()} ({keypoints.confidence:.0%})"
            cv2.putText(
                out, label, (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA,
            )

        # Info bar
        info = f"FPS: {fps:.1f}  Det: {detection_ms:.1f}ms"
        cv2.putText(
            out, info, (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA,
        )

        if keypoints is None:
            cv2.putText(
                out, "No hand detected", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA,
            )

        return out

    def show(self, annotated_frame: np.ndarray) -> bool:
        """Display the frame. Returns False if user pressed ESC or closed window."""
        cv2.imshow(self._window_name, annotated_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            return False
        if cv2.getWindowProperty(self._window_name, cv2.WND_PROP_VISIBLE) < 1:
            return False
        return True

    def close(self) -> None:
        cv2.destroyWindow(self._window_name)
