"""
Test script for camera publisher communication.

This script subscribes to camera topics and verifies that images are being published correctly.

Usage:
    # Test all cameras
    python test_camera_publisher.py

    # Test specific camera
    python test_camera_publisher.py --camera chest
    python test_camera_publisher.py --camera head

    # Test with verbose output
    python test_camera_publisher.py --verbose
"""

import argparse
import time
from typing import Dict, Optional

import numpy as np

from decoupled_wbc.control.main.constants import CAMERA_CHEST_TOPIC, CAMERA_HEAD_TOPIC
from decoupled_wbc.control.utils.ros_utils import ROSManager, ROSMsgSubscriber


class CameraSubscriber:
    """Subscribes to camera topics and verifies communication."""

    def __init__(self, camera_id: str = "both", verbose: bool = False):
        """Initialize camera subscriber.

        Args:
            camera_id: Which camera to test ("chest", "head", or "both")
            verbose: Enable verbose output
        """
        self.camera_id = camera_id
        self.verbose = verbose

        # Initialize ROS 2
        self.ros_manager = ROSManager(node_name="CameraTester")
        self.node = self.ros_manager.node

        # Create subscribers based on camera selection
        self.subscribers = {}
        self.received = {}

        if camera_id in ["chest", "both"]:
            self.subscribers["chest"] = ROSMsgSubscriber(CAMERA_CHEST_TOPIC)
            self.received["chest"] = {"count": 0, "last_time": 0, "shape": None}
            print(f"Subscribed to chest camera on {CAMERA_CHEST_TOPIC}")

        if camera_id in ["head", "both"]:
            self.subscribers["head"] = ROSMsgSubscriber(CAMERA_HEAD_TOPIC)
            self.received["head"] = {"count": 0, "last_time": 0, "shape": None}
            print(f"Subscribed to head camera on {CAMERA_HEAD_TOPIC}")

    def _check_message(self, name: str, msg: Optional[Dict]):
        """Check a received message."""
        if msg is None:
            return

        self.received[name]["count"] += 1
        current_time = time.time()

        # Check RGB image data
        if "rgb" in msg:
            rgb = msg["rgb"]
            if isinstance(rgb, np.ndarray):
                self.received[name]["shape"] = rgb.shape
                if self.verbose:
                    print(f"[{name}] RGB shape: {rgb.shape}, dtype: {rgb.dtype}")
                    print(f"[{name}] RGB range: [{rgb.min()}, {rgb.max()}]")

        # Check depth image data
        if "depth" in msg:
            depth = msg["depth"]
            if isinstance(depth, np.ndarray):
                if self.verbose:
                    print(f"[{name}] Depth shape: {depth.shape}, dtype: {depth.dtype}")
                    print(f"[{name}] Depth range: [{depth.min()}, {depth.max()}] mm")

        # Check camera info
        if "camera_info" in msg and self.verbose:
            info = msg["camera_info"]
            print(f"[{name}] Camera info: {info.get('width')}x{info.get('height')}")
            if "fx" in info:
                print(f"[{name}] Intrinsics: fx={info.get('fx')}, fy={info.get('fy')}, "
                      f"cx={info.get('cx')}, cy={info.get('cy')}")

        # Check frame rate
        if self.received[name]["last_time"] > 0:
            dt = current_time - self.received[name]["last_time"]
            fps = 1.0 / dt if dt > 0 else 0
            if self.verbose:
                print(f"[{name}] FPS: {fps:.1f}")

        self.received[name]["last_time"] = current_time

    def run(self, duration: float = 5.0):
        """Run the camera test.

        Args:
            duration: Test duration in seconds
        """
        print(f"\nTesting camera communication for {duration} seconds...")
        print("=" * 60)

        rate = self.node.create_rate(10)  # Check at 10 Hz
        start_time = time.monotonic()

        try:
            while self.ros_manager.ok() and (time.monotonic() - start_time) < duration:
                # Check all subscribed cameras
                if "chest" in self.subscribers:
                    msg = self.subscribers["chest"].get_msg()
                    self._check_message("chest", msg)

                if "head" in self.subscribers:
                    msg = self.subscribers["head"].get_msg()
                    self._check_message("head", msg)

                rate.sleep()

        except self.ros_manager.exceptions() as e:
            print(f"Test interrupted: {e}")

        # Print summary
        self._print_summary()

    def _print_summary(self):
        """Print test summary."""
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)

        all_ok = True

        for name, stats in self.received.items():
            count = stats["count"]
            shape = stats["shape"]

            if count == 0:
                print(f"✗ {name}: No messages received")
                all_ok = False
            else:
                fps = count / 5.0  # Approximate FPS
                print(f"✓ {name}: {count} messages, ~{fps:.1f} FPS, shape={shape}")

        print("=" * 60)

        if all_ok:
            print("✓ All camera communication tests PASSED")
        else:
            print("✗ Some camera communication tests FAILED")

    def shutdown(self):
        """Clean up resources."""
        self.ros_manager.shutdown()


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Test camera publisher communication",
        epilog="""
Examples:
    # Test all cameras
    python test_camera_publisher.py

    # Test specific camera
    python test_camera_publisher.py --camera chest

    # Test with verbose output
    python test_camera_publisher.py --verbose
        """,
    )

    parser.add_argument(
        "--camera",
        type=str,
        default="both",
        choices=["chest", "head", "both"],
        help="Which camera to test (default: both)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Test duration in seconds (default: 5.0)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose output",
    )

    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_args()

    # Create subscriber
    subscriber = CameraSubscriber(
        camera_id=args.camera,
        verbose=args.verbose,
    )

    try:
        subscriber.run(duration=args.duration)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        subscriber.shutdown()


if __name__ == "__main__":
    main()
