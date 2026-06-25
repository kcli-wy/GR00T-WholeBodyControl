"""
Hand Pose Publisher for G1 Robot Control

This script publishes specified hand poses to the G1 robot control loop via ROS 2.
It uses the existing IK pipeline (TeleopRetargetingIK) to compute joint angles from
hand poses and publishes them to the ControlPolicy/upper_body_pose topic.

Usage:
    # Run with default poses (hands at sides)
    python run_hand_pose_publisher.py

    # Run with custom poses via CLI
    python run_hand_pose_publisher.py \
        --left-pos 0.3 0.2 0.5 --left-quat 1.0 0.0 0.0 0.0 \
        --right-pos 0.3 -0.2 0.5 --right-quat 1.0 0.0 0.0 0.0

    # Run with YAML config
    python run_hand_pose_publisher.py --config hand_poses.yaml

    # Run in simulation mode
    python run_hand_pose_publisher.py --interface sim

    # Run with hand open/close control
    python run_hand_pose_publisher.py --left-hand-open --right-hand-closed

    # Run with smooth trajectory (interpolate between waypoints)
    python run_hand_pose_publisher.py --waypoints waypoints.yaml --interp-freq 50

Message format published to ControlPolicy/upper_body_pose:
    {
        "target_upper_body_pose": np.ndarray (17,),  # joint angles for upper body
        "target_time": float,                         # monotonic time to reach target
        "base_height_command": float,                 # base height (default 0.74)
        "navigate_cmd": np.ndarray (3,),              # [vx, vy, wz] (default zeros)
        "torso_orientation_rpy": np.ndarray (3,),    # [roll, pitch, yaw] in radians
        "wrist_pose": np.ndarray (14,),               # left(7) + right(7), each [x,y,z,w,qx,qy,qz]
        "timestamp": float,                           # wall clock time
        "left_wrist": np.ndarray (4,4),               # left wrist 4x4 matrix
        "right_wrist": np.ndarray (4,4),              # right wrist 4x4 matrix
        "left_fingers": dict,                          # finger data
        "right_fingers": dict,                         # finger data
    }
"""

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from decoupled_wbc.control.main.constants import (
    CONTROL_GOAL_TOPIC,
    DEFAULT_BASE_HEIGHT,
    DEFAULT_NAV_CMD,
)
from decoupled_wbc.control.robot_model.instantiation.g1 import instantiate_g1_robot_model
from decoupled_wbc.control.teleop.solver.hand.instantiation.g1_hand_ik_instantiation import (
    instantiate_g1_hand_ik_solver,
)
from decoupled_wbc.control.teleop.teleop_retargeting_ik import TeleopRetargetingIK
from decoupled_wbc.control.utils.ros_utils import ROSManager, ROSMsgPublisher
from decoupled_wbc.control.utils.telemetry import Telemetry


@dataclass
class HandPose:
    """Represents a hand pose with position and orientation."""

    position: np.ndarray  # (3,) - [x, y, z] in world frame
    quaternion: np.ndarray  # (4,) - [w, x, y, z] scalar-first format

    def to_matrix(self) -> np.ndarray:
        """Convert to 4x4 homogeneous transformation matrix."""
        T = np.eye(4)
        T[:3, 3] = self.position
        T[:3, :3] = R.from_quat(self.quaternion, scalar_first=True).as_matrix()
        return T

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> "HandPose":
        """Create from 4x4 homogeneous transformation matrix."""
        return cls(
            position=matrix[:3, 3],
            quaternion=R.from_matrix(matrix[:3, :3]).as_quat(scalar_first=True),
        )

    @classmethod
    def identity(cls) -> "HandPose":
        """Create identity pose (origin, no rotation)."""
        return cls(
            position=np.zeros(3),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        )


@dataclass
class FingerState:
    """Represents finger open/close state."""

    thumb: float = 0.0  # 0.0 = open, 1.0 = closed
    index: float = 0.0
    middle: float = 0.0
    ring: float = 0.0
    pinky: float = 0.0

    def to_position_array(self) -> np.ndarray:
        """Convert to (25, 4, 4) position array expected by hand IK solver.

        The format is 5 fingers × 5 joints each = 25 joint transforms.
        We use simplified transforms where only the x-position indicates open/close state.
        """
        positions = np.zeros((25, 4, 4))
        # Set identity transforms for all joints
        for i in range(25):
            positions[i] = np.eye(4)

        # Set open/close state via x-position of fingertip (index 4, 9, 14, 19, 24)
        # Thumb tip at index 4
        positions[4, 0, 3] = 1.0 - self.thumb
        # Index tip at index 9
        positions[9, 0, 3] = 1.0 - self.index
        # Middle tip at index 14
        positions[14, 0, 3] = 1.0 - self.middle
        # Ring tip at index 19
        positions[19, 0, 3] = 1.0 - self.ring
        # Pinky tip at index 24
        positions[24, 0, 3] = 1.0 - self.pinky

        return positions

    @classmethod
    def open(cls) -> "FingerState":
        """All fingers open."""
        return cls(thumb=0.0, index=0.0, middle=0.0, ring=0.0, pinky=0.0)

    @classmethod
    def closed(cls) -> "FingerState":
        """All fingers closed."""
        return cls(thumb=1.0, index=1.0, middle=1.0, ring=1.0, pinky=1.0)


@dataclass
class HandPoseCommand:
    """Complete command for both hands."""

    left_pose: HandPose
    right_pose: HandPose
    left_fingers: FingerState
    right_fingers: FingerState
    base_height: float = DEFAULT_BASE_HEIGHT
    navigate_cmd: np.ndarray = None
    torso_orientation_rpy: np.ndarray = None  # [roll, pitch, yaw] in radians

    def __post_init__(self):
        if self.navigate_cmd is None:
            self.navigate_cmd = np.array(DEFAULT_NAV_CMD)
        if self.torso_orientation_rpy is None:
            self.torso_orientation_rpy = np.array([0.0, 0.0, 0.0])


class HandPosePublisher:
    """Publishes hand poses to the G1 robot control loop.

    This class handles:
    1. IK computation using TeleopRetargetingIK
    2. Message formatting for the control loop
    3. ROS 2 publishing to ControlPolicy/upper_body_pose
    """

    def __init__(
        self,
        interface: str = "sim",
        waist_location: str = "lower_body",
        high_elbow_pose: bool = False,
        publish_frequency: float = 50.0,
    ):
        """Initialize the hand pose publisher.

        Args:
            interface: Robot interface ("sim" for simulation, "real" for real robot)
            waist_location: Waist joint assignment ("lower_body", "upper_body", or "lower_and_upper_body")
            high_elbow_pose: Whether to use high elbow pose configuration
            publish_frequency: Publishing frequency in Hz
        """
        self.publish_frequency = publish_frequency

        # Initialize ROS 2
        self.ros_manager = ROSManager(node_name="HandPosePublisher")
        self.node = self.ros_manager.node

        # Create publisher
        self.publisher = ROSMsgPublisher(CONTROL_GOAL_TOPIC)

        # Initialize robot model
        self.robot_model = instantiate_g1_robot_model(
            waist_location=waist_location, high_elbow_pose=high_elbow_pose
        )

        # Initialize IK solvers
        left_hand_ik_solver, right_hand_ik_solver = instantiate_g1_hand_ik_solver()

        # Initialize retargeting IK
        self.retargeting_ik = TeleopRetargetingIK(
            robot_model=self.robot_model,
            left_hand_ik_solver=left_hand_ik_solver,
            right_hand_ik_solver=right_hand_ik_solver,
            enable_visualization=True,
            body_active_joint_groups=["upper_body"],
        )

        # Initialize telemetry
        self.telemetry = Telemetry(window_size=100)

        # Get joint group indices for upper body
        self.upper_body_indices = self.robot_model.get_joint_group_indices("upper_body")

        # Store initial pose for warmup
        self._warmed_up = False

        print(f"HandPosePublisher initialized (interface={interface}, freq={publish_frequency}Hz)")
        print(f"Upper body joint count: {len(self.upper_body_indices)}")

    def _warmup_ik(self):
        """Perform IK warmup to initialize the solver."""
        if not self._warmed_up:
            print("Warming up IK solver...")
            # Run a few iterations with default pose
            default_pose = HandPose.identity()
            body_data = {
                "left_wrist_yaw_link": default_pose.to_matrix(),
                "right_wrist_yaw_link": default_pose.to_matrix(),
            }
            self.retargeting_ik.set_goal({
                "body_data": body_data,
                "left_hand_data": {"position": FingerState.open().to_position_array()},
                "right_hand_data": {"position": FingerState.open().to_position_array()},
            })
            # Warmup runs internally in compute_joint_positions
            self.retargeting_ik.compute_joint_positions(
                body_data,
                {"position": FingerState.open().to_position_array()},
                {"position": FingerState.open().to_position_array()},
            )
            self._warmed_up = True
            print("IK warmup complete")

    def compute_ik(self, command: HandPoseCommand) -> np.ndarray:
        """Compute joint angles from hand poses using IK.

        Args:
            command: Hand pose command with left/right poses and finger states

        Returns:
            np.ndarray: Upper body joint angles (17 DOFs)
        """
        # Prepare body data (wrist poses as 4x4 matrices)
        body_data = {
            "left_wrist_yaw_link": command.left_pose.to_matrix(),
            "right_wrist_yaw_link": command.right_pose.to_matrix(),
        }

        # Prepare hand data
        left_hand_data = {"position": command.left_fingers.to_position_array()}
        right_hand_data = {"position": command.right_fingers.to_position_array()}

        # Set goal and compute IK
        self.retargeting_ik.set_goal({
            "body_data": body_data,
            "left_hand_data": left_hand_data,
            "right_hand_data": right_hand_data,
        })

        upper_body_pose = self.retargeting_ik.get_action()
        return upper_body_pose

    def publish_command(self, command: HandPoseCommand) -> dict:
        """Compute IK and publish command to control loop.

        Args:
            command: Hand pose command

        Returns:
            dict: The published message
        """
        with self.telemetry.timer("compute_ik"):
            upper_body_pose = self.compute_ik(command)

        t_now = time.monotonic()

        # Construct wrist pose (14 values: left 7 + right 7)
        left_wrist = command.left_pose
        right_wrist = command.right_pose
        wrist_pose = np.concatenate([
            left_wrist.position,
            left_wrist.quaternion,
            right_wrist.position,
            right_wrist.quaternion,
        ])

        # Construct message
        msg = {
            "target_upper_body_pose": upper_body_pose,
            "target_time": t_now + (1.0 / self.publish_frequency),
            "base_height_command": command.base_height,
            "navigate_cmd": command.navigate_cmd,
            "torso_orientation_rpy": np.deg2rad(command.torso_orientation_rpy),
            "wrist_pose": wrist_pose,
            "timestamp": time.time(),
            "left_wrist": command.left_pose.to_matrix(),
            "right_wrist": command.right_pose.to_matrix(),
            "left_fingers": {"position": command.left_fingers.to_position_array()},
            "right_fingers": {"position": command.right_fingers.to_position_array()},
        }

        with self.telemetry.timer("publish"):
            self.publisher.publish(msg)

        return msg

    def run_interpolation(
        self,
        waypoints: List[HandPoseCommand],
        duration: float = 5.0,
        loop: bool = False,
    ):
        """Run smooth interpolation between waypoints.

        Args:
            waypoints: List of hand pose commands to interpolate between
            duration: Total duration in seconds
            loop: Whether to loop continuously
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints for interpolation")

        self._warmup_ik()

        rate = self.node.create_rate(self.publish_frequency)
        start_time = time.monotonic()

        try:
            while self.ros_manager.ok():
                elapsed = time.monotonic() - start_time

                if not loop and elapsed >= duration:
                    print(f"Interpolation complete after {duration:.1f}s")
                    break

                # Compute interpolation parameter
                t = (elapsed % duration) / duration
                segment_idx = int(t * (len(waypoints) - 1))
                segment_t = (t * (len(waypoints) - 1)) - segment_idx

                # Clamp indices
                segment_idx = min(segment_idx, len(waypoints) - 2)

                # Interpolate between waypoints
                wp1 = waypoints[segment_idx]
                wp2 = waypoints[segment_idx + 1]

                # Linear interpolation for positions
                left_pos = wp1.left_pose.position * (1 - segment_t) + wp2.left_pose.position * segment_t
                right_pos = wp1.right_pose.position * (1 - segment_t) + wp2.right_pose.position * segment_t

                # SLERP for rotations
                left_rot1 = R.from_quat(wp1.left_pose.quaternion, scalar_first=True)
                left_rot2 = R.from_quat(wp2.left_pose.quaternion, scalar_first=True)
                left_rot_interp = R.from_quat(
                    R.from_matrix(
                        left_rot1.as_matrix() * (1 - segment_t) + left_rot2.as_matrix() * segment_t
                    ).as_quat(),
                    scalar_first=False,
                )
                # Use proper SLERP
                left_slerp = R.from_matrix(
                    left_rot1.inv().as_matrix() @ left_rot2.as_matrix()
                )
                left_rot_final = left_rot1 * R.from_rotvec(left_slerp.as_rotvec() * segment_t)

                right_rot1 = R.from_quat(wp1.right_pose.quaternion, scalar_first=True)
                right_rot2 = R.from_quat(wp2.right_pose.quaternion, scalar_first=True)
                right_slerp = R.from_matrix(
                    right_rot1.inv().as_matrix() @ right_rot2.as_matrix()
                )
                right_rot_final = right_rot1 * R.from_rotvec(right_slerp.as_rotvec() * segment_t)

                # Interpolate finger states
                left_fingers = FingerState(
                    thumb=wp1.left_fingers.thumb * (1 - segment_t) + wp2.left_fingers.thumb * segment_t,
                    index=wp1.left_fingers.index * (1 - segment_t) + wp2.left_fingers.index * segment_t,
                    middle=wp1.left_fingers.middle * (1 - segment_t) + wp2.left_fingers.middle * segment_t,
                    ring=wp1.left_fingers.ring * (1 - segment_t) + wp2.left_fingers.ring * segment_t,
                    pinky=wp1.left_fingers.pinky * (1 - segment_t) + wp2.left_fingers.pinky * segment_t,
                )
                right_fingers = FingerState(
                    thumb=wp1.right_fingers.thumb * (1 - segment_t) + wp2.right_fingers.thumb * segment_t,
                    index=wp1.right_fingers.index * (1 - segment_t) + wp2.right_fingers.index * segment_t,
                    middle=wp1.right_fingers.middle * (1 - segment_t) + wp2.right_fingers.middle * segment_t,
                    ring=wp1.right_fingers.ring * (1 - segment_t) + wp2.right_fingers.ring * segment_t,
                    pinky=wp1.right_fingers.pinky * (1 - segment_t) + wp2.right_fingers.pinky * segment_t,
                )

                # Create interpolated command
                interp_cmd = HandPoseCommand(
                    left_pose=HandPose(position=left_pos, quaternion=left_rot_final.as_quat(scalar_first=True)),
                    right_pose=HandPose(position=right_pos, quaternion=right_rot_final.as_quat(scalar_first=True)),
                    left_fingers=left_fingers,
                    right_fingers=right_fingers,
                    base_height=wp1.base_height * (1 - segment_t) + wp2.base_height * segment_t,
                    navigate_cmd=wp1.navigate_cmd * (1 - segment_t) + wp2.navigate_cmd * segment_t,
                    torso_orientation_rpy=wp1.torso_orientation_rpy * (1 - segment_t) + wp2.torso_orientation_rpy * segment_t,
                )

                self.publish_command(interp_cmd)
                rate.sleep()

        except self.ros_manager.exceptions() as e:
            print(f"Interpolation interrupted: {e}")

    def run_static(self, command: HandPoseCommand, duration: float = None):
        """Run with a static pose (publish continuously).

        Args:
            command: Hand pose command to hold
            duration: Duration in seconds (None for indefinite)
        """
        self._warmup_ik()

        rate = self.node.create_rate(self.publish_frequency)
        start_time = time.monotonic()

        try:
            while self.ros_manager.ok():
                if duration and (time.monotonic() - start_time) >= duration:
                    print(f"Static pose complete after {duration:.1f}s")
                    break

                self.publish_command(command)
                rate.sleep()

        except self.ros_manager.exceptions() as e:
            print(f"Static pose interrupted: {e}")

    def shutdown(self):
        """Clean up resources."""
        self.ros_manager.shutdown()


def load_poses_from_yaml(yaml_path: str) -> List[HandPoseCommand]:
    """Load hand poses from a YAML configuration file.

    Expected YAML format:
    ```yaml
    waypoints:
      - left_pose:
          position: [0.3, 0.2, 0.5]
          quaternion: [1.0, 0.0, 0.0, 0.0]  # [w, x, y, z]
        right_pose:
          position: [0.3, -0.2, 0.5]
          quaternion: [1.0, 0.0, 0.0, 0.0]
        left_fingers:
          thumb: 0.0
          index: 0.0
          middle: 0.0
          ring: 0.0
          pinky: 0.0
        right_fingers:
          thumb: 0.0
          index: 0.0
          middle: 0.0
          ring: 0.0
          pinky: 0.0
        base_height: 0.74
        navigate_cmd: [0.0, 0.0, 0.0]
    ```
    """
    with open(yaml_path, "r") as f:
        config = yaml.safe_load(f)

    waypoints = []
    for wp in config.get("waypoints", []):
        left_pose = HandPose(
            position=np.array(wp["left_pose"]["position"]),
            quaternion=np.array(wp["left_pose"]["quaternion"]),
        )
        right_pose = HandPose(
            position=np.array(wp["right_pose"]["position"]),
            quaternion=np.array(wp["right_pose"]["quaternion"]),
        )

        left_finger_cfg = wp.get("left_fingers", {})
        left_fingers = FingerState(**left_finger_cfg)

        right_finger_cfg = wp.get("right_fingers", {})
        right_fingers = FingerState(**right_finger_cfg)

        base_height = wp.get("base_height", DEFAULT_BASE_HEIGHT)
        navigate_cmd = np.array(wp.get("navigate_cmd", DEFAULT_NAV_CMD))
        torso_orientation_rpy = np.array(wp.get("torso_orientation_rpy", [0.0, 0.0, 0.0]))

        waypoints.append(HandPoseCommand(
            left_pose=left_pose,
            right_pose=right_pose,
            left_fingers=left_fingers,
            right_fingers=right_fingers,
            base_height=base_height,
            navigate_cmd=navigate_cmd,
            torso_orientation_rpy=torso_orientation_rpy,
        ))

    return waypoints


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Publish hand poses to G1 robot control loop",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with default poses (arms at sides)
  python run_hand_pose_publisher.py

  # Run with custom poses
  python run_hand_pose_publisher.py \\
      --left-pos 0.3 0.2 0.5 --left-quat 1.0 0.0 0.0 0.0 \\
      --right-pos 0.3 -0.2 0.5 --right-quat 1.0 0.0 0.0 0.0

  # Run with YAML waypoints (smooth interpolation)
  python run_hand_pose_publisher.py --waypoints hand_poses.yaml --duration 10.0

  # Run with hands open/closed
  python run_hand_pose_publisher.py --left-hand-open --right-hand-closed

  # Run in simulation
  python run_hand_pose_publisher.py --interface sim

  # Run with custom base height
  python run_hand_pose_publisher.py --base-height 0.6
        """,
    )

    # Interface options
    parser.add_argument(
        "--interface",
        type=str,
        default="sim",
        choices=["sim", "real"],
        help="Robot interface (default: sim)",
    )
    parser.add_argument(
        "--waist-location",
        type=str,
        default="lower_body",
        choices=["lower_body", "upper_body", "lower_and_upper_body"],
        help="Waist joint assignment (default: lower_body)",
    )
    parser.add_argument(
        "--high-elbow-pose",
        action="store_true",
        help="Use high elbow pose configuration",
    )
    parser.add_argument(
        "--freq",
        type=float,
        default=50.0,
        help="Publishing frequency in Hz (default: 50.0)",
    )

    # Pose options
    parser.add_argument(
        "--left-pos",
        type=float,
        nargs=3,
        default=[0.15, 0.2, 0.1],
        metavar=("X", "Y", "Z"),
        help="Left hand position [x, y, z] (default: 0.2 0.2 0.0)",
    )
    parser.add_argument(
        "--left-quat",
        type=float,
        nargs=4,
        default=(0.7071068, 0, 0.7071068, 0),
        metavar=("W", "X", "Y", "Z"),
        help="Left hand quaternion [w, x, y, z] (default: 1.0 0.0 0.0 0.0)",
    )
    parser.add_argument(
        "--right-pos",
        type=float,
        nargs=3,
        default=[0.15, -0.2, 0.1],
        metavar=("X", "Y", "Z"),
        help="Right hand position [x, y, z] (default: 0.2 -0.2 0.0)",
    )
    parser.add_argument(
        "--right-quat",
        type=float,
        nargs=4,
        default=(1, 0, 0, 0),
        metavar=("W", "X", "Y", "Z"),
        help="Right hand quaternion [w, x, y, z] (default: 1.0 0.0 0.0 0.0)",
    )

    # Finger options
    parser.add_argument(
        "--left-hand-open",
        action="store_true",
        help="Left hand fingers open (default)",
    )
    parser.add_argument(
        "--left-hand-closed",
        action="store_true",
        help="Left hand fingers closed",
    )
    parser.add_argument(
        "--right-hand-open",
        action="store_true",
        help="Right hand fingers open (default)",
    )
    parser.add_argument(
        "--right-hand-closed",
        action="store_true",
        help="Right hand fingers closed",
    )

    # Control options
    parser.add_argument(
        "--base-height",
        type=float,
        default=DEFAULT_BASE_HEIGHT,
        help=f"Base height command (default: {DEFAULT_BASE_HEIGHT})",
    )
    parser.add_argument(
        "--navigate-cmd",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        metavar=("VX", "VY", "WZ"),
        help="Navigation command [vx, vy, wz] (default: 0.0 0.0 0.0)",
    )
    parser.add_argument(
        "--torso-rpy",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        metavar=("ROLL", "PITCH", "YAW"),
        help="Torso orientation [roll, pitch, yaw] in degrees (default: 0.0 0.0 0.0)",
    )

    # Waypoint options
    parser.add_argument(
        "--waypoints",
        type=str,
        default=None,
        help="Path to YAML file with waypoints for interpolation",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3,
        help="Duration in seconds (None for indefinite)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Loop continuously (for waypoint interpolation)",
    )

    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_args()

    # Create publisher
    publisher = HandPosePublisher(
        interface=args.interface,
        waist_location=args.waist_location,
        high_elbow_pose=args.high_elbow_pose,
        publish_frequency=args.freq,
    )

    # Determine finger states
    if args.left_hand_closed:
        left_fingers = FingerState.closed()
    else:
        left_fingers = FingerState.open()

    if args.right_hand_closed:
        right_fingers = FingerState.closed()
    else:
        right_fingers = FingerState.open()

    # Create command
    command = HandPoseCommand(
        left_pose=HandPose(
            position=np.array(args.left_pos),
            quaternion=np.array(args.left_quat),
        ),
        right_pose=HandPose(
            position=np.array(args.right_pos),
            quaternion=np.array(args.right_quat),
        ),
        left_fingers=left_fingers,
        right_fingers=right_fingers,
        base_height=args.base_height,
        navigate_cmd=np.array(args.navigate_cmd),
        torso_orientation_rpy=np.array(args.torso_rpy),
    )

    try:
        if args.waypoints:
            # Load waypoints from YAML and interpolate
            print(f"Loading waypoints from {args.waypoints}")
            waypoints = load_poses_from_yaml(args.waypoints)
            print(f"Loaded {len(waypoints)} waypoints")
            publisher.run_interpolation(
                waypoints=waypoints,
                duration=args.duration or 10.0,
                loop=args.loop,
            )
        else:
            # Run with static pose
            print(f"Publishing static pose:")
            print(f"  Left hand:  pos={command.left_pose.position}, quat={command.left_pose.quaternion}")
            print(f"  Right hand: pos={command.right_pose.position}, quat={command.right_pose.quaternion}")
            print(f"  Left fingers:  {'closed' if args.left_hand_closed else 'open'}")
            print(f"  Right fingers: {'closed' if args.right_hand_closed else 'open'}")
            print(f"  Base height: {command.base_height}")
            print(f"  Navigate cmd: {command.navigate_cmd}")
            print(f"  Torso RPY (deg): {command.torso_orientation_rpy}")
            publisher.run_static(command=command, duration=args.duration)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        publisher.shutdown()


if __name__ == "__main__":
    main()
