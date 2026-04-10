"""Backwards-compat shim — moved to ``sender.hand.core.mano_transform``.

The MANO transform helpers used to live here when only the RealSense path
needed them. They are now shared with ``sender.hand.gen3a_dex_retarget`` for
the Manus skeleton path, so the implementation has been promoted to
``sender.hand.core.mano_transform`` and given a per-source ``convention``
argument (``"mediapipe"`` for RealSense/phone, ``"manus"`` for the Manus
glove). See that module's docstring for the full rationale.

This file exists only so that any external code that still does
``from sender.hand.realsense.mano_transform import apply_mano_transform``
keeps working. Prefer importing from ``sender.hand.core.mano_transform``
in new code.
"""

from sender.hand.core.mano_transform import (  # noqa: F401
    MANUS_OPERATOR2MANO_LEFT,
    MANUS_OPERATOR2MANO_RIGHT,
    MEDIAPIPE_OPERATOR2MANO_LEFT,
    MEDIAPIPE_OPERATOR2MANO_RIGHT,
    OPERATOR2MANO_LEFT,
    OPERATOR2MANO_RIGHT,
    apply_mano_transform,
    estimate_wrist_frame,
)
