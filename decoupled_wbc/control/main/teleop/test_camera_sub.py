"""
Camera Subscriber Test for G1 Robot

Subscribes to camera topics and displays/saves received images.

Usage:
    # Subscribe to all cameras
    python camera_sub.py

    # Subscribe to chest camera only
    python camera_sub.py --camera chest

    # Subscribe to head camera only
    python camera_sub.py --camera head

    # Save images to directory
    python camera_sub.py --save-dir ./camera_images

    # Display images (requires OpenCV)
    python camera_sub.py --display

    # Verbose output
    python camera_sub.py --verbose
"""

import argparse
import os
import time
from typing import Dict, Optional

import numpy as np

from decoupled_wbc.control.main.constants import CAMERA_CHEST_TOPIC, CAMERA_HEAD_TOPIC
from decoupled_wbc.control.utils.ros_utils import ROSManager, ROSMsgSubscriber


class CameraSubscriber:
    """Subscribes to camera topics for testing."""

    def __init__(
        self,
        camera_id: str = "both",
        save_dir: str = None,
        display: bool = False,
        verbose: bool = False,
    ):
        """Initialize camera subscriber.

        Args:
            camera_id: Which camera to subscribe ("chest", "head", or "both")
            save_dir: Directory to save images (None to skip saving)
            display: Display images using OpenCV
            verbose: Enable verbose output
        """
        self.camera_id = camera_id
        self.save_dir = save_dir
        self.display = display
        self.verbose = verbose

        # Create save directory if needed
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
            print(f"Saving images to: {save_dir}")

        # Initialize ROS 2
        self.ros_manager = ROSManager(node_name="CameraSubscriber")
        self.node = self.ros_manager.node

        # Create subscribers based on camera selection
        self.subscribers = {}
        self.stats = {}

        if camera_id in ["chest", "both"]:
            self.subscribers["chest"] = ROSMsgSubscriber(CAMERA_CHEST_TOPIC)
            self.stats["chest"] = {"count": 0, "last_time": 0}
            print(f"Subscribed to chest camera: {CAMERA_CHEST_TOPIC}")

        if camera_id in ["head", "both"]:
            self.subscribers["head"] = ROSMsgSubscriber(CAMERA_HEAD_TOPIC)
            self.stats["head"] = {"count": 0, "last_time": 0}
            print(f"Subscribed to head camera: {CAMERA_HEAD_TOPIC}")

        # Initialize OpenCV for display
        if display:
            try:
                import cv2
                self.cv2 = cv2
                print("OpenCV loaded for display")
            except ImportError:
                print("WARNING: OpenCV not available, display disabled")
                self.display = False

    def _save_image(self, name: str, image: np.ndarray, image_type: str = "rgb"):
        """Save image to file."""
        if not self.save_dir:
            return

        timestamp = int(time.time() * 1000)
        filename = f"{name}_{image_type}_{timestamp}.npy"
        filepath = os.path.join(self.save_dir, filename)
        np.save(filepath, image)

        if self.verbose:
            print(f"Saved: {filepath}")

    def _display_image(self, name: str, image: np.ndarray, image_type: str = "rgb"):
        """Display image using OpenCV."""
        if not self.display:
            return

        try:
            if image_type == "rgb":
                # Convert RGB to BGR for OpenCV
                display_img = self.cv2.cvtColor(image, self.cv2.COLOR_RGB2BGR)
            elif image_type == "depth":
                # Normalize depth for display
                depth_normalized = (image / 3000.0 * 255).astype(np.uint8)
                display_img = self.cv2.applyColorMap(depth_normalized, self.cv2.COLORMAP_JET)
            else:
                return

            window_name = f"{name} ({image_type})"
            self.cv2.imshow(window_name, display_img)
            self.cv2.waitKey(1)
        except Exception as e:
            if self.verbose:
                print(f"Display error: {e}")

    def _process_chest(self):
        """Process chest camera message."""
        msg = self.subscribers["chest"].get_msg()
        if msg is None:
            return

        self.stats["chest"]["count"] += 1
        current_time = time.time()

        # Extract RGB image
        rgb = msg.get("rgb")
        if rgb is not None and isinstance(rgb, np.ndarray):
            if self.verbose:
                fps = 1.0 / (current_time - self.stats["chest"]["last_time"]) if self.stats["chest"]["last_time"] > 0 else 0
                print(f"[Chest] RGB: shape={rgb.shape}, dtype={rgb.dtype}, "
                      f"range=[{rgb.min()}, {rgb.max()}], fps={fps:.1f}")

            self._save_image("chest", rgb, "rgb")
            self._display_image("chest", rgb, "rgb")

        # Print camera info
        if self.verbose and "camera_info" in msg:
            info = msg["camera_info"]
            print(f"[Chest] Camera: {info.get('width')}x{info.get('height')}")

        self.stats["chest"]["last_time"] = current_time

    def _process_head(self):
        """Process head camera message (RGB + Depth)."""
        msg = self.subscribers["head"].get_msg()
        if msg is None:
            return

        self.stats["head"]["count"] += 1
        current_time = time.time()

        # Extract RGB image
        rgb = msg.get("rgb")
        if rgb is not None and isinstance(rgb, np.ndarray):
            if self.verbose:
                fps = 1.0 / (current_time - self.stats["head"]["last_time"]) if self.stats["head"]["last_time"] > 0 else 0
                print(f"[Head RGB] shape={rgb.shape}, dtype={rgb.dtype}, "
                      f"range=[{rgb.min()}, {rgb.max()}], fps={fps:.1f}")

            self._save_image("head", rgb, "rgb")
            self._display_image("head", rgb, "rgb")

        # Extract depth image
        depth = msg.get("depth")
        if depth is not None and isinstance(depth, np.ndarray):
            if self.verbose:
                print(f"[Head Depth] shape={depth.shape}, dtype={depth.dtype}, "
                      f"range=[{depth.min()}, {depth.max()}] mm")

            self._save_image("head", depth, "depth")
            self._display_image("head", depth, "depth")

        # Print camera info
        if self.verbose and "camera_info" in msg:
            info = msg["camera_info"]
            print(f"[Head] Camera: {info.get('width')}x{info.get('height')}")

        self.stats["head"]["last_time"] = current_time

    def run(self, duration: float = None):
        """Run the camera subscriber.

        Args:
            duration: Duration in seconds (None for indefinite)
        """
        print(f"\nCamera subscriber running... (Ctrl+C to stop)")
        print("=" * 60)

        rate = self.node.create_rate(30)  # 30 Hz check rate
        start_time = time.monotonic()

        try:
            while self.ros_manager.ok():
                if duration and (time.monotonic() - start_time) >= duration:
                    print(f"\nDuration limit reached ({duration}s)")
                    break

                # Process all subscribed cameras
                if "chest" in self.subscribers:
                    self._process_chest()

                if "head" in self.subscribers:
                    self._process_head()

                rate.sleep()

        except self.ros_manager.exceptions() as e:
            print(f"\nSubscriber interrupted: {e}")
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self._print_summary()
            self.shutdown()

    def _print_summary(self):
        """Print summary statistics."""
        print("\n" + "=" * 60)
        print("SUMMARY")
        print("=" * 60)

        for name, stat in self.stats.items():
            count = stat["count"]
            if count > 0:
                elapsed = time.monotonic() - stat.get("start_time", time.monotonic())
                avg_fps = count / elapsed if elapsed > 0 else 0
                print(f"✓ {name}: {count} messages, avg {avg_fps:.1f} FPS")
            else:
                print(f"✗ {name}: No messages received")

        print("=" * 60)

    def shutdown(self):
        """Clean up resources."""
        if self.display:
            try:
                self.cv2.destroyAllWindows()
            except:
                pass
        self.ros_manager.shutdown()


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Camera subscriber test for G1 robot",
        epilog="""
Examples:
    # Subscribe to all cameras
    python camera_sub.py

    # Subscribe to chest camera only
    python camera_sub.py --camera chest

    # Subscribe with display
    python camera_sub.py --display

    # Save images
    python camera_sub.py --save-dir ./images

    # Verbose output
    python camera_sub.py --verbose
        """,
    )

    parser.add_argument(
        "--camera",
        type=str,
        default="both",
        choices=["chest", "head", "both"],
        help="Which camera to subscribe (default: both)",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default=None,
        help="Directory to save images (None to skip)",
    )
    parser.add_argument(
        "--display",
        action="store_true",
        help="Display images using OpenCV",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose output",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Duration in seconds (None for indefinite)",
    )

    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_args()

    # Create subscriber
    subscriber = CameraSubscriber(
        camera_id=args.camera,
        save_dir=args.save_dir,
        display=args.display,
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
