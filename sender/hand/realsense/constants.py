"""RealSense D405 sensing configuration constants."""

# ── Stream defaults ──────────────────────────────────────
RS_WIDTH = 640
RS_HEIGHT = 480
RS_FPS = 30

# ── Depth processing ────────────────────────────────────
# Radius (pixels) to search around each landmark for a valid depth value.
# A small neighborhood median rejects depth noise / holes.
DEPTH_SEARCH_RADIUS = 4

# Maximum valid depth in meters (reject far background).
DEPTH_MAX_M = 1.0

# Minimum valid depth in meters (reject too-close noise).
DEPTH_MIN_M = 0.02
