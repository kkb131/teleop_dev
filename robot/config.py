"""Robot configuration — shared across teleop_dev robot sub-packages."""

from pathlib import Path

# Paths
_TELEOP_DEV = Path(__file__).resolve().parent.parent  # teleop_dev/
_TAMP_DEV = _TELEOP_DEV.parent / "tamp_dev"           # sibling tamp_dev/
XRDF_PATH = str(
    _TAMP_DEV
    / "cumotion/isaac_ros_cumotion/isaac_ros_cumotion_robot_description/xrdf/ur10e.xrdf"
)
URDF_PATH = str(_TAMP_DEV / ".docker/assets/ur10e.urdf")

# Robot
DEFAULT_ROBOT_IP = "192.168.0.2"
RTDE_FREQUENCY = 125  # Hz

# UR10e joint names
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Safety limits (UR10e)
MAX_JOINT_VEL_RAD_S = 2.094  # ~120 deg/s (shoulder/elbow)
MAX_JOINT_ACCEL_RAD_S2 = 12.0
DEFAULT_VELOCITY_SCALE = 0.05  # 5%
UR10E_MAX_TORQUES = [150.0, 150.0, 56.0, 56.0, 28.0, 28.0]  # Nm per joint

# Direct torque control (PolyScope 5.23.0+)
DIRECT_TORQUE_FREQUENCY = 500  # Hz (UR controller timestep)
UR_SECONDARY_PORT = 30002  # URScript transmission port

# servoJ parameters
SERVOJ_DT = 1.0 / RTDE_FREQUENCY  # 0.008s
SERVOJ_LOOKAHEAD = 0.1  # seconds
SERVOJ_GAIN = 300

# cuMotion planner defaults
INTERPOLATION_DT = 0.025  # 40Hz
NUM_GRAPH_SEEDS = 6
NUM_TRAJOPT_SEEDS = 6
TRAJOPT_TSTEPS = 32
MAX_ATTEMPTS = 10

# Mode
DEFAULT_MODE = "sim"  # "sim" (Isaac Sim) or "rtde" (real robot)
SIM_PUBLISH_RATE = 125  # Hz (Isaac Sim command publish rate)

# Servo loop rate
SERVO_RATE_HZ = 50  # keyboard/joystick teleop common rate

# Controller names (servo)
TRAJECTORY_CONTROLLER = "joint_trajectory_controller"
SCALED_TRAJECTORY_CONTROLLER = "scaled_joint_trajectory_controller"
FORWARD_POSITION_CONTROLLER = "forward_position_controller"
FORWARD_VELOCITY_CONTROLLER = "forward_velocity_controller"

# Predefined poses (radians) — UR10e
HOME_JOINTS = [2.24, -1.2808, 2.16, -0.8848, 2.24, 0.0]
UP_JOINTS = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]

# Near-home waypoints for multi-goal test (small deltas from HOME)
NEAR_HOME_WAYPOINTS = [
    [2.40, -1.18, 2.06, -0.88, 2.24, 0.15],   # A: shoulder+elbow shift
    [2.10, -1.40, 2.30, -0.75, 2.24, -0.15],   # B: opposite direction
    [2.24, -1.10, 2.00, -1.05, 2.40, 0.10],    # C: wrist emphasis
    [2.35, -1.35, 2.25, -0.80, 2.10, -0.10],   # D: mixed shift
    [2.15, -1.20, 2.10, -0.95, 2.30, 0.20],    # E: slight variation
]
