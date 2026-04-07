"""Hand retargeting module — factory for retarget modes.

Modes:
    vector: Skeleton 3D → fingertip direction vector optimization (2.5gen)
    direct: Skeleton 3D → angle extraction → linear mapping (1.5gen)
"""

from typing import Optional
import numpy as np


def create_retarget(mode: str, hand_side: str = "right",
                    urdf_path: str = None, config_path: str = None):
    """Factory: create retarget instance by mode name.

    Parameters
    ----------
    mode : str
        "vector" or "direct"
    hand_side : str
        "left" or "right"
    urdf_path : str or None
        DG5F URDF path override
    config_path : str or None
        Calibration config YAML (for direct mode)
    """
    if mode == "vector":
        from sender.hand.retarget.vector_retarget import VectorRetarget
        return VectorRetarget(hand_side=hand_side, urdf_path=urdf_path)
    elif mode == "direct":
        from sender.hand.retarget.direct_mapping import DirectMappingRetarget
        return DirectMappingRetarget(hand_side=hand_side, config_path=config_path)
    else:
        raise ValueError(f"Unknown retarget mode: {mode!r} (choose: vector, direct)")
