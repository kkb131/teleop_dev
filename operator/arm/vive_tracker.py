#!/usr/bin/env python3
"""Vive Tracker 3.0 interface via SteamVR OpenVR API.

Reads 6-DOF pose (position + orientation) from a Vive Tracker.
Requires SteamVR running and the tracker paired + tracked.

Usage (standalone test):
    python3 -m vive.vive_tracker [--serial <TRACKER_SERIAL>]
"""

import argparse
import math
import time

import numpy as np

try:
    import openvr
except ImportError:
    openvr = None


def _hmd_matrix_to_pos_quat(mat) -> tuple[np.ndarray, np.ndarray]:
    """Convert OpenVR HmdMatrix34_t to position(3) and quaternion(4, wxyz)."""
    # Extract 3x3 rotation and translation
    m = np.array([
        [mat[0][0], mat[0][1], mat[0][2]],
        [mat[1][0], mat[1][1], mat[1][2]],
        [mat[2][0], mat[2][1], mat[2][2]],
    ])
    pos = np.array([mat[0][3], mat[1][3], mat[2][3]])

    # Rotation matrix → quaternion (wxyz)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    quat = np.array([w, x, y, z])
    # Normalize
    quat /= np.linalg.norm(quat)
    return pos, quat


class ViveTracker:
    """Reads 6-DOF pose from a Vive Tracker 3.0 via OpenVR.

    Parameters
    ----------
    tracker_serial : str or None
        Serial number of the target tracker. If None, uses the first
        GenericTracker found.
    """

    def __init__(self, tracker_serial: str | None = None):
        if openvr is None:
            raise ImportError(
                "openvr not installed. Run: pip install openvr"
            )
        self._serial = tracker_serial
        self._vr_system = None
        self._tracker_idx: int | None = None

    def connect(self):
        """Initialize OpenVR and find the tracker device."""
        self._vr_system = openvr.init(openvr.VRApplication_Other)
        print("[ViveTracker] OpenVR initialized")

        self._find_tracker()
        if self._tracker_idx is None:
            serial_msg = f" (serial={self._serial})" if self._serial else ""
            raise RuntimeError(
                f"No Vive Tracker found{serial_msg}. "
                "Check SteamVR and tracker pairing."
            )
        serial = self._get_serial(self._tracker_idx)
        print(f"[ViveTracker] Using tracker index={self._tracker_idx}, serial={serial}")

    def disconnect(self):
        """Shut down OpenVR."""
        if self._vr_system is not None:
            openvr.shutdown()
            self._vr_system = None
            self._tracker_idx = None
            print("[ViveTracker] OpenVR shut down")

    def get_pose(self) -> tuple[np.ndarray, np.ndarray] | None:
        """Get current tracker pose.

        Returns
        -------
        (position, quaternion) : (ndarray[3], ndarray[4]) or None
            position in meters (SteamVR coords: Y-up, right-handed).
            quaternion in wxyz convention.
            Returns None if tracker is not currently tracked.
        """
        if self._vr_system is None or self._tracker_idx is None:
            return None

        poses = self._vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0.0,  # predicted seconds from now
            openvr.k_unMaxTrackedDeviceCount,
        )

        pose = poses[self._tracker_idx]
        if not pose.bPoseIsValid:
            return None
        if pose.eTrackingResult != openvr.TrackingResult_Running_OK:
            return None

        return _hmd_matrix_to_pos_quat(pose.mDeviceToAbsoluteTracking)

    def get_all_trackers(self) -> list[dict]:
        """List all connected GenericTracker devices.

        Returns list of dicts with keys: index, serial, tracking.
        """
        if self._vr_system is None:
            return []

        trackers = []
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            dev_class = self._vr_system.getTrackedDeviceClass(i)
            if dev_class == openvr.TrackedDeviceClass_GenericTracker:
                serial = self._get_serial(i)
                poses = self._vr_system.getDeviceToAbsoluteTrackingPose(
                    openvr.TrackingUniverseStanding, 0.0,
                    openvr.k_unMaxTrackedDeviceCount,
                )
                tracking = poses[i].bPoseIsValid
                trackers.append({
                    "index": i,
                    "serial": serial,
                    "tracking": tracking,
                })
        return trackers

    def _find_tracker(self):
        """Find the target tracker device index."""
        self._tracker_idx = None
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            dev_class = self._vr_system.getTrackedDeviceClass(i)
            if dev_class != openvr.TrackedDeviceClass_GenericTracker:
                continue

            serial = self._get_serial(i)
            if self._serial is None or serial == self._serial:
                self._tracker_idx = i
                return

    def _get_serial(self, device_idx: int) -> str:
        """Get device serial number string."""
        return self._vr_system.getStringTrackedDeviceProperty(
            device_idx,
            openvr.Prop_SerialNumber_String,
        )

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Vive Tracker 3.0 pose reader")
    parser.add_argument("--serial", default=None,
                        help="Tracker serial number (default: first found)")
    parser.add_argument("--hz", type=int, default=50,
                        help="Print rate in Hz (default: 50)")
    parser.add_argument("--list", action="store_true",
                        help="List all connected trackers and exit")
    args = parser.parse_args()

    with ViveTracker(tracker_serial=args.serial) as tracker:
        if args.list:
            trackers = tracker.get_all_trackers()
            if not trackers:
                print("[ViveTracker] No trackers found")
            for t in trackers:
                status = "TRACKING" if t["tracking"] else "NOT TRACKING"
                print(f"  index={t['index']}  serial={t['serial']}  [{status}]")
            return

        print(f"[ViveTracker] Streaming pose at {args.hz} Hz. Ctrl+C to stop.")
        dt = 1.0 / args.hz
        count = 0

        try:
            while True:
                t_start = time.perf_counter()

                result = tracker.get_pose()
                if result is None:
                    print("\r[LOST] Tracker not tracked", end="", flush=True)
                else:
                    pos, quat = result
                    print(
                        f"\r[{count:6d}] "
                        f"pos=({pos[0]:+7.4f}, {pos[1]:+7.4f}, {pos[2]:+7.4f})  "
                        f"quat=({quat[0]:+6.3f}, {quat[1]:+6.3f}, "
                        f"{quat[2]:+6.3f}, {quat[3]:+6.3f})",
                        end="", flush=True,
                    )
                count += 1

                elapsed = time.perf_counter() - t_start
                remaining = dt - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:
            print(f"\n[ViveTracker] Stopped. Total reads: {count}")


if __name__ == "__main__":
    main()
