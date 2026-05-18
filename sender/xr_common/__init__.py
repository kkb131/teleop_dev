"""Shared XR (Galaxy XR / Quest 3) bridge primitives.

The :class:`BridgePoseStore` class spins up a self-contained aiohttp HTTP+WS
server that the :file:`assets/webxr_to_pose.html` client connects to from the
headset's browser. WebXR frame events are decoded into multiprocessing-shared
NumPy arrays so that arm and hand senders running in different threads (or
processes) can read head / wrist / 25-joint hand data from a single source of
truth.

Single source of truth pattern:
    The store is a singleton — instantiate it once per process. Both
    ``sender.arm.xr_sender`` and ``sender.hand.xr_hand_sender`` read from the
    same instance so the WebSocket port (default 8013) is never opened twice.

Cross-reference:
    Originated as ``xr_teleop/scripts/bridge_pose_store.py``. Ported to
    teleop_dev with the vuer-specific noise stripped — teleop_dev never uses
    vuer, so the constructor accepts and silently ignores those kwargs for
    historical compatibility.
"""

from sender.xr_common.bridge_pose_store import BridgePoseStore, load_config, JOINT_NAMES

__all__ = ["BridgePoseStore", "load_config", "JOINT_NAMES"]
