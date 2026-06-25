"""
Test script for stretcher task ROS2 communication.

This script tests the communication between:
1. VLN navigation model (publishes nav commands)
2. Pose estimation model (publishes handle poses)
3. Stretcher task controller (subscribes to both)

Usage:
    # Run all tests
    python test_stretcher_comm.py

    # Run specific test
    python test_stretcher_comm.py --test nav
    python test_stretcher_comm.py --test pose
    python test_stretcher_comm.py --test full
"""

import argparse
import time
from typing import Dict, List

import numpy as np

from decoupled_wbc.control.main.constants import (
    DEFAULT_STRETCHER_POSE_TOPIC_PREFIX,
    STRETCHER_LEFT_HANDLE_ID,
    STRETCHER_NAV_CMD_TOPIC,
    STRETCHER_RIGHT_HANDLE_ID,
    STRETCHER_TASK_STATUS_TOPIC,
)
from decoupled_wbc.control.utils.ros_utils import ROSManager, ROSMsgPublisher, ROSMsgSubscriber


class MockVLNPublisher:
    """Mock VLN model that publishes navigation commands."""

    def __init__(self):
        self.ros_manager = ROSManager(node_name="MockVLN")
        self.publisher = ROSMsgPublisher(STRETCHER_NAV_CMD_TOPIC)
        self.counter = 0

    def publish_test_command(self, nav_cmd: List[float], arrived: bool = False):
        """Publish a test navigation command."""
        msg = {
            "navigate_cmd": nav_cmd,
            "arrived": arrived,
            "timestamp": time.time(),
        }
        self.publisher.publish(msg)
        self.counter += 1
        print(f"[VLN] Published nav_cmd={nav_cmd}, arrived={arrived}")

    def shutdown(self):
        self.ros_manager.shutdown()


class MockPosePublisher:
    """Mock FoundationPose++ publisher.

    真实 FP++ 对每个物体发独立 topic (FoundationPose/pose/<object_id>), 各自带
    pose_robot_matrix (4x4). 这里镜像该接口: 两个独立 publisher, 每条消息带
    pose_robot_matrix, 由调用方传 position 构造 (orientation 置单位阵 —— 真实
    StretcherHandle.from_msg 只取平移列, 不读朝向).
    """

    def __init__(self, pose_topic_prefix: str = DEFAULT_STRETCHER_POSE_TOPIC_PREFIX):
        self.ros_manager = ROSManager(node_name="MockPose")
        self.left_publisher = ROSMsgPublisher(f"{pose_topic_prefix}/{STRETCHER_LEFT_HANDLE_ID}")
        self.right_publisher = ROSMsgPublisher(f"{pose_topic_prefix}/{STRETCHER_RIGHT_HANDLE_ID}")
        self.counter = 0

    @staticmethod
    def _position_to_robot_matrix(position: List[float]) -> list:
        """把 [x,y,z] 包成 4x4 齐次变换矩阵 (单位旋转 + 平移), 模拟 FP++ 的 pose_robot_matrix."""
        T = np.eye(4)
        T[:3, 3] = position
        return T.tolist()

    def publish_test_pose(
        self,
        left_position: List[float] = None,
        right_position: List[float] = None,
        ready_to_grab: bool = False,
        nav_cmd: List[float] = None,
    ):
        """Publish a test handle pose message (两侧独立 topic).

        ready_to_grab / nav_cmd 仅用于向后兼容调用方签名, 当前 controller 不再读取
        (FineTuning 改用内置 PD), 这里忽略不发布.
        """
        if left_position is not None:
            self.left_publisher.publish({
                "object_id": STRETCHER_LEFT_HANDLE_ID,
                "frame_id": "head_camera",
                "stamp": time.time_ns(),
                "pose_robot_matrix": self._position_to_robot_matrix(left_position),
                "timestamp": time.time(),
            })

        if right_position is not None:
            self.right_publisher.publish({
                "object_id": STRETCHER_RIGHT_HANDLE_ID,
                "frame_id": "head_camera",
                "stamp": time.time_ns(),
                "pose_robot_matrix": self._position_to_robot_matrix(right_position),
                "timestamp": time.time(),
            })

        self.counter += 1
        print(f"[Pose] Published handles: left={left_position}, right={right_position}")

    def shutdown(self):
        self.ros_manager.shutdown()


class CommunicationTester:
    """Tests ROS2 communication for the stretcher task."""

    def __init__(self):
        self.ros_manager = ROSManager(node_name="CommTester")
        self.node = self.ros_manager.node

        # Subscribers
        self.status_subscriber = ROSMsgSubscriber(STRETCHER_TASK_STATUS_TOPIC)

        # Publishers (mock models)
        self.mock_vln = MockVLNPublisher()
        self.mock_pose = MockPosePublisher()

    def test_nav_communication(self, duration: float = 5.0):
        """Test VLN navigation command communication."""
        print("\n" + "=" * 60)
        print("TEST: Navigation Command Communication")
        print("=" * 60)

        rate = self.node.create_rate(10)  # 10 Hz
        start_time = time.monotonic()

        try:
            while self.ros_manager.ok() and (time.monotonic() - start_time) < duration:
                # Publish test navigation commands
                t = time.monotonic() - start_time

                # Simulate forward movement
                nav_cmd = [0.3, 0.0, 0.0]
                self.mock_vln.publish_test_command(nav_cmd)

                # Check for status feedback
                status_msg = self.status_subscriber.get_msg()
                if status_msg:
                    print(f"[Status] Phase: {status_msg.get('phase')}, Status: {status_msg.get('status')}")

                rate.sleep()

            # Signal arrival
            self.mock_vln.publish_test_command([0.0, 0.0, 0.0], arrived=True)
            print("\n✓ Navigation communication test complete")

        except Exception as e:
            print(f"\n✗ Navigation test failed: {e}")

    def test_pose_communication(self, duration: float = 5.0):
        """Test pose estimation communication."""
        print("\n" + "=" * 60)
        print("TEST: Pose Estimation Communication")
        print("=" * 60)

        rate = self.node.create_rate(10)  # 10 Hz
        start_time = time.monotonic()

        try:
            while self.ros_manager.ok() and (time.monotonic() - start_time) < duration:
                t = time.monotonic() - start_time

                # Simulate handle poses
                left_pos = [0.5, 0.3, 0.8]
                right_pos = [0.5, -0.3, 0.8]

                # Simulate fine-tuning navigation
                nav_cmd = [0.0, 0.1, 0.0]  # Slight lateral movement

                self.mock_pose.publish_test_pose(
                    left_position=left_pos,
                    right_position=right_pos,
                    nav_cmd=nav_cmd,
                )

                # Check for status feedback
                status_msg = self.status_subscriber.get_msg()
                if status_msg:
                    print(f"[Status] Phase: {status_msg.get('phase')}, Status: {status_msg.get('status')}")

                rate.sleep()

            # Signal ready to grab
            self.mock_pose.publish_test_pose(
                left_position=[0.4, 0.25, 0.8],
                right_position=[0.4, -0.25, 0.8],
                ready_to_grab=True,
            )
            print("\n✓ Pose estimation communication test complete")

        except Exception as e:
            print(f"\n✗ Pose estimation test failed: {e}")

    def test_full_workflow(self, duration: float = 20.0):
        """Test the full workflow simulation."""
        print("\n" + "=" * 60)
        print("TEST: Full Workflow Simulation")
        print("=" * 60)

        rate = self.node.create_rate(10)  # 10 Hz
        start_time = time.monotonic()
        phase = "navigating"

        try:
            while self.ros_manager.ok() and (time.monotonic() - start_time) < duration:
                t = time.monotonic() - start_time

                # Simulate different phases
                if phase == "navigating" and t < 5.0:
                    # Phase 1: Navigation
                    nav_cmd = [0.3, 0.0, 0.0]
                    self.mock_vln.publish_test_command(nav_cmd)

                elif phase == "navigating" and t >= 5.0:
                    # Transition to fine-tuning
                    self.mock_vln.publish_test_command([0.0, 0.0, 0.0], arrived=True)
                    phase = "fine_tuning"
                    print(f"\n--- Transitioning to {phase} at t={t:.1f}s ---")

                elif phase == "fine_tuning" and t < 10.0:
                    # Phase 2: Fine-tuning
                    nav_cmd = [0.0, 0.05, 0.0]
                    self.mock_pose.publish_test_pose(
                        left_position=[0.5, 0.3, 0.8],
                        right_position=[0.5, -0.3, 0.8],
                        nav_cmd=nav_cmd,
                    )

                elif phase == "fine_tuning" and t >= 10.0:
                    # Transition to grabbing
                    self.mock_pose.publish_test_pose(
                        left_position=[0.4, 0.25, 0.8],
                        right_position=[0.4, -0.25, 0.8],
                        ready_to_grab=True,
                    )
                    phase = "grabbing"
                    print(f"\n--- Transitioning to {phase} at t={t:.1f}s ---")

                elif phase == "grabbing":
                    # Phase 3: Grabbing (just wait for controller to handle)
                    self.mock_pose.publish_test_pose(
                        left_position=[0.4, 0.25, 0.8],
                        right_position=[0.4, -0.25, 0.8],
                    )

                # Check for status feedback
                status_msg = self.status_subscriber.get_msg()
                if status_msg:
                    print(f"[Status] Phase: {status_msg.get('phase')}, Status: {status_msg.get('status')}")

                rate.sleep()

            print("\n✓ Full workflow simulation complete")

        except Exception as e:
            print(f"\n✗ Full workflow test failed: {e}")

    def run_all_tests(self):
        """Run all communication tests."""
        print("\n" + "=" * 60)
        print("STRETCHER TASK COMMUNICATION TESTS")
        print("=" * 60)

        self.test_nav_communication(duration=3.0)
        time.sleep(1.0)

        self.test_pose_communication(duration=3.0)
        time.sleep(1.0)

        self.test_full_workflow(duration=15.0)

        print("\n" + "=" * 60)
        print("ALL TESTS COMPLETE")
        print("=" * 60)

    def shutdown(self):
        """Clean up resources."""
        self.mock_vln.shutdown()
        self.mock_pose.shutdown()
        self.ros_manager.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test stretcher task ROS2 communication",
        epilog="""
Examples:
  # Run all tests
  python test_stretcher_comm.py

  # Run specific test
  python test_stretcher_comm.py --test nav
  python test_stretcher_comm.py --test pose
  python test_stretcher_comm.py --test full
        """,
    )

    parser.add_argument(
        "--test",
        type=str,
        default="all",
        choices=["all", "nav", "pose", "full"],
        help="Test to run (default: all)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Override test duration in seconds",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    tester = CommunicationTester()

    try:
        if args.test == "all":
            tester.run_all_tests()
        elif args.test == "nav":
            tester.test_nav_communication(duration=args.duration or 5.0)
        elif args.test == "pose":
            tester.test_pose_communication(duration=args.duration or 5.0)
        elif args.test == "full":
            tester.test_full_workflow(duration=args.duration or 20.0)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        tester.shutdown()


if __name__ == "__main__":
    main()
