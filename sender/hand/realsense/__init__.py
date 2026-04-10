"""RealSense D405 sensing pipeline for realsense_sender.

Wraps pyrealsense2 + MediaPipe HandLandmarker + depth deprojection
into a HandKeypoints stream that feeds DexRetargetWrapper.
"""

from sender.hand.realsense.config import RealsenseConfig
from sender.hand.realsense.hand_keypoints import HandKeypoints
from sender.hand.realsense.reader import RealSenseReader

__all__ = ["RealSenseReader", "RealsenseConfig", "HandKeypoints"]
