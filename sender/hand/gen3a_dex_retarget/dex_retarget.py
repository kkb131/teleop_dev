"""[3A] Manus skeleton / RealSense keypoints → dex_retargeting → DG5F joint angles.

Wrapper around the dex-retargeting library (DexPilot/Vector/Position
optimizers, AnyTeleop RSS 2023) that adapts it to the teleop_dev
HandRetargetBase interface.

Pipeline (skeleton path — Manus glove):
    HandData.skeleton (21, 7) raw 25→21 remapped, in Manus VUH world frame
    → extract xyz, shift to wrist origin
    → apply_mano_transform(convention="manus")        ← NEW (when mano_convention="manus")
    → build optimizer-specific ref_value
    → SeqRetargeting.retarget(ref_value)
    → DG5F joint angles (20,) in radians

Pipeline (keypoints path — RealSense / phone):
    HandKeypoints.keypoints_3d (21, 3) — already MANO-frame rotated by
    DepthKeypointConverter (phone path would be analogous)
    → wrist origin shift (idempotent on already-centered input)
    → no MANO transform here (already done upstream — caller passes
      mano_convention=None or just leaves it default)
    → optimizer

The per-source MANO matrix split lives in
:mod:`sender.hand.core.mano_transform`. See its module docstring for the
"mediapipe" vs "manus" convention rationale.

Usage:
    from sender.hand.gen3a_dex_retarget import DexRetargetWrapper

    # Manus glove (skeleton in raw VUH frame)
    rt = DexRetargetWrapper(hand_side="right", mano_convention="manus")
    q = rt.retarget(skeleton=hand_data.skeleton)

    # RealSense (keypoints already MANO-rotated by DepthKeypointConverter)
    rt = DexRetargetWrapper(hand_side="right")  # mano_convention=None
    q = rt.retarget(keypoints=hand_data.keypoints_3d)
"""

import logging
import time
from pathlib import Path
from typing import Optional

import numpy as np

from sender.hand.core.dg5f_config import NUM_JOINTS
from sender.hand.core.mano_transform import apply_mano_transform
from sender.hand.core.retarget_base import HandRetargetBase

logger = logging.getLogger(__name__)


class DexRetargetWrapper(HandRetargetBase):
    """[3A] dex-retargeting wrapper for Manus skeleton / RealSense keypoints → DG5F.

    Parameters
    ----------
    hand_side : str
        "left" or "right" (only "right" config currently provided).
    optimizer : str
        Built-in config selector: ``"dexpilot"`` (default, fast +
        thumb-pinch focused) or ``"vector"`` (multi-task palm→tip and
        MCP→DIP). Ignored when ``config_path`` is given.
    config_path : str or None
        Path to a custom YAML config. Overrides ``optimizer`` if set.
        Defaults to ``gen3a_dex_retarget/config/dg5f_{hand_side}_{optimizer}.yml``.
        The YAML's ``urdf_path`` is resolved relative to the YAML file
        itself, so configs can ship with relative paths.
    mano_convention : str or None
        Operator → MANO matrix to apply inside :meth:`retarget` AFTER the
        wrist origin shift. Pick the value that matches the input source:

        * ``None`` (default) — input is **already** in MANO frame, do not
          re-rotate. Use this when the upstream sensing module
          (e.g. :class:`DepthKeypointConverter` for RealSense, or any
          phone path that calls ``apply_mano_transform`` itself) has
          already done the rotation. Only the wrist origin shift will be
          applied (which is idempotent on wrist-centered input).
        * ``"manus"`` — input is the raw 25→21-remapped Manus skeleton in
          VUH world frame. Applies the Manus operator → MANO matrix.
        * ``"mediapipe"`` — input is wrist-centered MediaPipe-derived
          keypoints with no MANO rotation yet. Applies the MediaPipe
          operator → MANO matrix. Provided for symmetry; the existing
          RealSense path normally pre-applies and uses ``None``.

        See :mod:`sender.hand.core.mano_transform` for the full rationale
        behind the per-source split.
    """

    def __init__(self, hand_side: str = "right",
                 optimizer: str = "dexpilot",
                 config_path: Optional[str] = None,
                 mano_convention: Optional[str] = None):
        super().__init__(hand_side)

        if optimizer not in ("dexpilot", "vector"):
            raise ValueError(
                f"optimizer must be 'dexpilot' or 'vector', got '{optimizer}'"
            )

        if mano_convention not in (None, "manus", "mediapipe"):
            raise ValueError(
                f"mano_convention must be None, 'manus', or 'mediapipe', "
                f"got {mano_convention!r}"
            )
        self._mano_convention = mano_convention

        # Default to packaged YAML for the requested optimizer
        here = Path(__file__).parent
        if config_path is None:
            config_path = here / "config" / f"dg5f_{hand_side}_{optimizer}.yml"
        config_path = Path(config_path).resolve()
        if not config_path.exists():
            raise FileNotFoundError(f"DexRetarget config not found: {config_path}")

        # dex-retargeting expects an absolute urdf_path. The shipped YAML uses
        # a relative one (just the filename), so patch it before loading.
        from dex_retargeting.retargeting_config import RetargetingConfig

        urdf_dir = config_path.parent
        # RetargetingConfig.set_default_urdf_dir() lets us point at the YAML's
        # own directory; library then resolves relative urdf_path against it.
        try:
            RetargetingConfig.set_default_urdf_dir(str(urdf_dir))
        except AttributeError:
            # Older library versions: fall back to in-place YAML rewrite
            pass

        cfg = RetargetingConfig.load_from_file(str(config_path))
        self._retargeting = cfg.build()

        # Single source of truth: the optimizer's own indices.
        # - vector:   (2, N) array from YAML target_link_human_indices
        # - dexpilot: (2, num_pairs + num_fingers) auto-generated by
        #             DexPilotOptimizer.__init__ via generate_link_indices()
        # - position: (N,) array of fingertip indices
        opt = self._retargeting.optimizer
        self._retargeting_type = opt.retargeting_type
        self._human_indices = opt.target_link_human_indices

        self._wrist_idx = 0  # MANO 21 wrist
        self._last_solve_ms = 0.0

        logger.info(
            "DexRetargetWrapper loaded (hand=%s, type=%s, config=%s, "
            "joints=%d, mano_convention=%s)",
            hand_side,
            self._retargeting_type,
            config_path.name,
            len(self._retargeting.joint_names),
            self._mano_convention,
        )

    def retarget(self,
                 skeleton: Optional[np.ndarray] = None,
                 keypoints: Optional[np.ndarray] = None,
                 **kwargs) -> np.ndarray:
        """Manus skeleton (21, 7) OR raw keypoints (21, 3) → DG5F angles (20,).

        Parameters
        ----------
        skeleton : ndarray[21, 7] or None
            MANO 21-node array [x, y, z, qw, qx, qy, qz] from the Manus ROS2
            reader. Quaternion columns are ignored. Typically the wrapper is
            constructed with ``mano_convention="manus"`` so the raw VUH-frame
            xyz gets rotated to MANO before reaching the optimizer.
        keypoints : ndarray[21, 3] or None
            3D keypoints from non-Manus sources such as RealSense. Normally
            already wrist-centered and MANO-frame rotated by the upstream
            sensing module (e.g. ``DepthKeypointConverter``); in that case
            construct the wrapper with ``mano_convention=None`` so this
            method only does an idempotent wrist origin shift.

        Pass exactly one of ``skeleton`` or ``keypoints``. If both/neither
        given, returns zeros.

        Returns
        -------
        ndarray[20] DG5F joint angles in radians (clamped by underlying
        SeqRetargeting to URDF joint limits).
        """
        if keypoints is not None:
            kp = np.asarray(keypoints, dtype=np.float32)
            if kp.ndim < 2 or kp.shape[0] < 21:
                return np.zeros(NUM_JOINTS, dtype=np.float32)
            kp = kp[:21, :3]
        elif skeleton is not None:
            if skeleton.shape[0] < 21:
                return np.zeros(NUM_JOINTS, dtype=np.float32)
            kp = skeleton[:21, :3].astype(np.float32)
        else:
            return np.zeros(NUM_JOINTS, dtype=np.float32)

        # Wrist origin shift (idempotent — safe even if input is already
        # wrist-relative, e.g. from RealSense DepthKeypointConverter).
        kp = kp - kp[self._wrist_idx]

        # Optionally rotate to MANO frame here. Skip when None — the upstream
        # sensing module is expected to have already applied the transform
        # (RealSense path). When set, the convention selects which operator →
        # MANO matrix lands the input in the canonical frame the optimizer
        # expects. See sender.hand.core.mano_transform for the rationale.
        if self._mano_convention is not None:
            kp = apply_mano_transform(
                kp, hand_type=self._hand_side, convention=self._mano_convention,
            )

        ref_value = self._build_ref_value(kp)

        t0 = time.perf_counter()
        q = self._retargeting.retarget(ref_value)
        self._last_solve_ms = (time.perf_counter() - t0) * 1000

        return q.astype(np.float32)

    def get_method_name(self) -> str:
        return "3A-dex-retarget"

    def get_debug_info(self) -> Optional[dict]:
        return {
            "method": self.get_method_name(),
            "type": self._retargeting_type,
            "solve_time_ms": self._last_solve_ms,
        }

    def get_joint_names(self) -> list[str]:
        return list(self._retargeting.joint_names)

    @property
    def solve_time_ms(self) -> float:
        return self._last_solve_ms

    def _build_ref_value(self, kp: np.ndarray) -> np.ndarray:
        """Convert (21, 3) keypoints to optimizer-specific input.

        Mirrors retarget_dev/models/dex_retarget/dex_retarget_model.py — uses
        the optimizer's stored target_link_human_indices so vector and dexpilot
        share the exact same code path the library expects.
        """
        idx = self._human_indices
        if self._retargeting_type == "POSITION":
            return kp[idx, :]
        origin_indices = idx[0, :]
        task_indices = idx[1, :]
        return kp[task_indices, :] - kp[origin_indices, :]
