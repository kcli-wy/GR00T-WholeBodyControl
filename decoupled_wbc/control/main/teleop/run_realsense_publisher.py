"""
RealSense Camera Publisher for G1 Robot

Publishes RGB and depth images from two RealSense cameras:
- Camera A (chest): RGB only, for navigation/VLN
- Camera B (head): RGB + Depth, for pose estimation

Features:
- Auto-calibration intrinsics from RealSense SDK
- Depth aligned to RGB
- Post-processing filters (decimation, spatial, temporal, hole-filling)

Usage:
    # Publish both cameras
    python realsense_publisher.py

    # Publish only chest camera
    python realsense_publisher.py --camera chest

    # Publish only head camera
    python realsense_publisher.py --camera head

    # Publish with custom resolution
    python realsense_publisher.py --width 640 --height 480 --fps 30
"""

import argparse
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from decoupled_wbc.control.main.constants import CAMERA_CHEST_TOPIC, CAMERA_HEAD_TOPIC
from decoupled_wbc.control.utils.ros_utils import ROSManager, ROSMsgPublisher

# ============================================================
# TODO: Fill in actual serial numbers when available
# ============================================================
CHEST_CAMERA_SERIAL = "352122272920"  # e.g., "1234567890"
HEAD_CAMERA_SERIAL = "352122272920"   # e.g., "9876543210"
# ============================================================


@dataclass
class RealSenseConfig:
    """RealSense camera configuration."""
    # Image dimensions
    width: int = 640
    height: int = 480
    fps: int = 30

    # Depth settings
    depth_enabled: bool = True
    align_depth_to_color: bool = True

    # Post-processing filters
    decimation_filter: bool = True
    spatial_filter: bool = True
    temporal_filter: bool = True
    hole_filling_filter: bool = True

    # Filter parameters
    decimation_magnitude: int = 1  # 1 = no decimation, 2 = half resolution
    spatial_smooth_alpha: float = 0.5
    spatial_smooth_delta: int = 20
    spatial_iterations: int = 1
    temporal_smooth_alpha: float = 0.4
    temporal_smooth_delta: int = 20
    temporal_persistence: int = 3
    hole_filling_mode: int = 1  # 0=disabled, 1=left, 2=farest, 3=nearest


class RealSenseCamera:
    """Wrapper for a single RealSense camera."""

    def __init__(
        self,
        serial_number: Optional[str],
        config: RealSenseConfig,
        enable_depth: bool = False,
    ):
        """Initialize RealSense camera.

        Args:
            serial_number: Camera serial number (None for auto-detect)
            config: Camera configuration
            enable_depth: Enable depth stream
        """
        self.serial_number = serial_number
        self.config = config
        self.enable_depth = enable_depth

        # RealSense pipeline and config
        self.pipeline = None
        self.rs_config = None
        self.align = None
        self.filters = []

        # Camera intrinsics (auto-calibrated)
        self.intrinsics = None
        self.depth_scale = None

        # Initialize camera
        self._init_camera()

    def _init_camera(self):
        """Initialize RealSense camera."""
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise ImportError(
                "pyrealsense2 not installed. Install with: pip install pyrealsense2"
            )

        print(f"Initializing RealSense camera (serial={self.serial_number})")

        # Create pipeline
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()

        # Configure streams
        if self.serial_number:
            self.rs_config.enable_device(self.serial_number)

        self.rs_config.enable_stream(
            rs.stream.color,
            self.config.width,
            self.config.height,
            rs.format.rgb8,  # RGB 格式, 与 FoundationPose 一致
            self.config.fps,
        )

        if self.enable_depth:
            self.rs_config.enable_stream(
                rs.stream.depth,
                self.config.width,
                self.config.height,
                rs.format.z16,
                self.config.fps,
            )

        # Start pipeline
        profile = self.pipeline.start(self.rs_config)

        # Get intrinsics from the camera
        color_stream = profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        print(f"Camera intrinsics: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}, "
              f"cx={self.intrinsics.ppx:.2f}, cy={self.intrinsics.ppy:.2f}")

        # Get depth scale (meters per raw unit)
        if self.enable_depth:
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            print(f"Depth scale: {self.depth_scale}")

        # Setup depth alignment
        if self.enable_depth and self.config.align_depth_to_color:
            self.align = rs.align(rs.stream.color)

        # Setup filters
        if self.enable_depth:
            self._setup_filters(rs)

        # Wait for auto-exposure to stabilize
        print("Waiting for auto-exposure to stabilize...")
        for _ in range(30):
            self.pipeline.wait_for_frames()
        print("Camera ready")

    def _setup_filters(self, rs):
        """Setup post-processing filters."""
        self.filters = []

        if self.config.decimation_filter:
            decimation = rs.decimation_filter()
            decimation.set_option(rs.option.filter_magnitude, self.config.decimation_magnitude)
            self.filters.append(decimation)

        if self.config.spatial_filter:
            spatial = rs.spatial_filter()
            spatial.set_option(rs.option.filter_smooth_alpha, self.config.spatial_smooth_alpha)
            spatial.set_option(rs.option.filter_smooth_delta, self.config.spatial_smooth_delta)
            spatial.set_option(rs.option.filter_magnitude, self.config.spatial_iterations)
            self.filters.append(spatial)

        if self.config.temporal_filter:
            temporal = rs.temporal_filter()
            temporal.set_option(rs.option.filter_smooth_alpha, self.config.temporal_smooth_alpha)
            temporal.set_option(rs.option.filter_smooth_delta, self.config.temporal_smooth_delta)
            temporal.set_option(rs.option.holes_fill, self.config.temporal_persistence)
            self.filters.append(temporal)

        if self.config.hole_filling_filter:
            hole_filling = rs.hole_filling_filter(self.config.hole_filling_mode)
            self.filters.append(hole_filling)

        print(f"Enabled {len(self.filters)} post-processing filters")

    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get RGB and depth frames from camera.

        Returns:
            Tuple of (rgb_frame, depth_frame) or (None, None) on error
            - rgb_frame: BGR image (H, W, 3) uint8
            - depth_frame: Depth image (H, W) uint16 (millimeters)
        """
        try:
            import pyrealsense2 as rs

            # Wait for frames
            frames = self.pipeline.wait_for_frames()

            # Align depth to color if enabled
            if self.align:
                frames = self.align.process(frames)

            # Get color frame
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None, None

            rgb = np.asanyarray(color_frame.get_data())  # RGB format (rs.format.rgb8)

            # Get depth frame if enabled
            depth = None
            if self.enable_depth:
                depth_frame = frames.get_depth_frame()

                # Apply filters
                for filter in self.filters:
                    depth_frame = filter.process(depth_frame)

                if depth_frame:
                    depth = np.asanyarray(depth_frame.get_data())

            return rgb, depth

        except Exception as e:
            print(f"Error getting frames: {e}")
            return None, None

    def get_intrinsics_dict(self) -> dict:
        """Get camera intrinsics as dictionary."""
        if self.intrinsics is None:
            return {}

        return {
            "width": self.intrinsics.width,
            "height": self.intrinsics.height,
            "fx": self.intrinsics.fx,
            "fy": self.intrinsics.fy,
            "cx": self.intrinsics.ppx,
            "cy": self.intrinsics.ppy,
            "distortion": list(self.intrinsics.coeffs),
            "depth_scale": self.depth_scale,
        }

    def shutdown(self):
        """Stop the camera pipeline."""
        if self.pipeline:
            try:
                self.pipeline.stop()
                print(f"Camera {self.serial_number} stopped")
            except Exception as e:
                print(f"Error stopping camera: {e}")


class RealSensePublisher:
    """Publishes RealSense camera images to ROS2 topics."""

    def __init__(
        self,
        camera_id: str = "both",
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ):
        """Initialize RealSense publisher.

        Args:
            camera_id: Which camera to publish ("chest", "head", or "both")
            width: Image width in pixels
            height: Image height in pixels
            fps: Publishing frequency in Hz
        """
        self.camera_id = camera_id

        # Camera configuration
        config = RealSenseConfig(
            width=width,
            height=height,
            fps=fps,
            depth_enabled=False,  # Will be overridden for head camera
            align_depth_to_color=True,
            decimation_filter=True,
            spatial_filter=True,
            temporal_filter=True,
            hole_filling_filter=True,
        )

        # Initialize ROS 2
        self.ros_manager = ROSManager(node_name="RealSensePublisher")
        self.node = self.ros_manager.node

        # Initialize cameras and publishers
        self.cameras = {}
        self.publishers = {}

        if camera_id in ["chest", "both"]:
            print("\n--- Initializing Chest Camera ---")
            self.cameras["chest"] = RealSenseCamera(
                serial_number=CHEST_CAMERA_SERIAL,
                config=config,
                enable_depth=False,  # Chest camera: RGB only
            )
            self.publishers["chest"] = ROSMsgPublisher(CAMERA_CHEST_TOPIC)
            print(f"[Chest] Topic: {CAMERA_CHEST_TOPIC}")
            print(f"[Chest] Resolution: {width}x{height} @ {fps}Hz")
            print(f"[Chest] Stream: RGB only")

        if camera_id in ["head", "both"]:
            print("\n--- Initializing Head Camera ---")
            head_config = RealSenseConfig(
                width=width,
                height=height,
                fps=fps,
                depth_enabled=True,
                align_depth_to_color=True,
                decimation_filter=True,
                spatial_filter=True,
                temporal_filter=True,
                hole_filling_filter=True,
            )
            self.cameras["head"] = RealSenseCamera(
                serial_number=HEAD_CAMERA_SERIAL,
                config=head_config,
                enable_depth=True,  # Head camera: RGB + Depth
            )
            self.publishers["head"] = ROSMsgPublisher(CAMERA_HEAD_TOPIC)
            print(f"[Head] Topic: {CAMERA_HEAD_TOPIC}")
            print(f"[Head] RGB: {width}x{height} @ {fps}Hz")
            print(f"[Head] Depth: {width}x{height} @ {fps}Hz (aligned to RGB)")
            print(f"[Head] Filters: Spatial, Temporal, Hole-filling")

        print(f"\nRealSensePublisher initialized (camera={camera_id}, {width}x{height} @ {fps}Hz)")

    def _publish_chest_camera(self):
        """Publish chest camera RGB image."""
        if "chest" not in self.cameras:
            return

        rgb, _ = self.cameras["chest"].get_frames()

        if rgb is not None:
            msg = {
                "rgb": rgb,
                "camera_info": self.cameras["chest"].get_intrinsics_dict(),
                "timestamp": time.time(),
                "frame_id": "chest_camera",
            }
            self.publishers["chest"].publish(msg)

    def _publish_head_camera(self):
        """Publish head camera RGB + depth in a single message."""
        if "head" not in self.cameras:
            return

        rgb, depth = self.cameras["head"].get_frames()

        if rgb is not None:
            msg = {
                "rgb": rgb,
                "depth": depth,  # None if depth not available
                "camera_info": self.cameras["head"].get_intrinsics_dict(),
                "timestamp": time.time(),
                "frame_id": "head_camera",
            }
            self.publishers["head"].publish(msg)

    def run(self, duration: float = None):
        """Run the publisher.

        Args:
            duration: Duration in seconds (None for indefinite)
        """
        rate = self.node.create_rate(30)
        start_time = time.monotonic()
        frame_count = 0

        print(f"\nPublishing RealSense data... (Ctrl+C to stop)")

        try:
            while self.ros_manager.ok():
                if duration and (time.monotonic() - start_time) >= duration:
                    print(f"\nDuration limit reached ({duration}s)")
                    break

                # Publish cameras
                self._publish_chest_camera()
                self._publish_head_camera()

                frame_count += 1
                if frame_count % 30 == 0:
                    elapsed = time.monotonic() - start_time
                    fps = frame_count / elapsed
                    print(f"Published {frame_count} frames ({elapsed:.1f}s, {fps:.1f} FPS)")

                rate.sleep()

        except self.ros_manager.exceptions() as e:
            print(f"\nPublisher interrupted: {e}")
        finally:
            self.shutdown()

    def shutdown(self):
        """Clean up resources."""
        for name, camera in self.cameras.items():
            camera.shutdown()
        self.ros_manager.shutdown()


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="RealSense camera publisher for G1 robot",
        epilog="""
Examples:
    # Publish both cameras
    python realsense_publisher.py

    # Publish only chest camera
    python realsense_publisher.py --camera chest

    # Publish only head camera
    python realsense_publisher.py --camera head

    # Publish with custom resolution
    python realsense_publisher.py --width 1280 --height 720 --fps 30
        """,
    )

    parser.add_argument(
        "--camera",
        type=str,
        default="both",
        choices=["chest", "head", "both"],
        help="Which camera to publish (default: both)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help="Image width in pixels (default: 640)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Image height in pixels (default: 480)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Publishing frequency in Hz (default: 30)",
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

    # Check serial numbers
    if args.camera in ["chest", "both"] and CHEST_CAMERA_SERIAL is None:
        print("WARNING: CHEST_CAMERA_SERIAL not set, will use auto-detect")
    if args.camera in ["head", "both"] and HEAD_CAMERA_SERIAL is None:
        print("WARNING: HEAD_CAMERA_SERIAL not set, will use auto-detect")

    # Create publisher
    publisher = RealSensePublisher(
        camera_id=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )

    try:
        publisher.run(duration=args.duration)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        publisher.shutdown()


if __name__ == "__main__":
    main()
