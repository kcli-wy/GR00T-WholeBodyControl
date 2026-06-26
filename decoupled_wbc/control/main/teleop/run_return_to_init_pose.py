"""
Return Upper Body to Init Pose for G1 Robot

仅发布上半身关节角命令, 让双臂回到 run_g1_control_loop.py 的初始化位置
(robot_model.get_initial_upper_body_pose(), 默认 high_elbow_pose=True, 即 G1
高肘姿态的 upper_body 关节角; 与 configs.py BaseConfig.high_elbow_pose=True 一致).

只发 target_upper_body_pose, 不发 base_height_command / navigate_cmd /
torso_orientation_rpy —— 下游 G1GearWbcPolicy 对这三个字段是 "有则更新、无则维持"
(g1_gear_wbc_policy.py: if x is not None), 缺字段时底盘/躯干保持上次/默认值不动,
不会清零或归位.

下游 InterpolationPolicy 会按 target_time 平滑插值过去, 避免关节瞬跳.

Usage:
    # 默认: 50Hz 持续发, 0.5s 过渡
    python run_return_to_init_pose.py
    # 默认: 发 move_duration+1.0s (插值过渡 + 维持稳定) 后自动退出

    # 自定义过渡时长 (自动退出时长随之 = move_duration + 1.0)
    python run_return_to_init_pose.py --move-duration 1.0

    # 一直发到 Ctrl+C (不自动退出)
    python run_return_to_init_pose.py --duration -1

发布的消息 (ControlPolicy/upper_body_pose):
    {
        "target_upper_body_pose": np.ndarray,  # upper_body 初始关节角
        "target_time": float,                   # 到达目标的 monotonic 时间
        "timestamp": float,                     # wall clock
    }
"""

import argparse
import time

import numpy as np

from decoupled_wbc.control.main.constants import CONTROL_GOAL_TOPIC
from decoupled_wbc.control.robot_model.instantiation.g1 import instantiate_g1_robot_model
from decoupled_wbc.control.utils.ros_utils import ROSManager, ROSMsgPublisher
from decoupled_wbc.control.utils.telemetry import Telemetry


class ReturnToInitPosePublisher:
    """持续发布 upper_body 初始关节角命令, 让双臂回到初始化位置."""

    def __init__(
        self,
        interface: str = "sim",
        waist_location: str = "lower_body",
        high_elbow_pose: bool = False,
        move_duration: float = 0.5,
        publish_frequency: float = 50.0,
    ):
        self.move_duration = move_duration
        self.publish_frequency = publish_frequency

        # Initialize ROS 2
        self.ros_manager = ROSManager(node_name="ReturnToInitPose")
        self.node = self.ros_manager.node

        # Publisher
        self.publisher = ROSMsgPublisher(CONTROL_GOAL_TOPIC)

        # Robot model (只取初始 upper_body 关节角, 不需要 IK solver)
        self.robot_model = instantiate_g1_robot_model(
            waist_location=waist_location, high_elbow_pose=high_elbow_pose
        )
        # run_g1_control_loop.py 的初始化位置 (默认 high_elbow_pose=True, 高肘 upper_body 关节角).
        # 与 wbc_policy_factory.py:24 用作 InterpolationPolicy init_values 的值一致.
        self.init_upper_body_pose = self.robot_model.get_initial_upper_body_pose()

        # Telemetry
        self.telemetry = Telemetry(window_size=100)

        print(f"ReturnToInitPosePublisher initialized (interface={interface}, freq={publish_frequency}Hz)")
        print(f"  upper_body joints: {len(self.init_upper_body_pose)}")
        print(f"  init pose (deg): {np.rad2deg(self.init_upper_body_pose)}")

    def _build_msg(self) -> dict:
        t_now = time.monotonic()
        return {
            "target_upper_body_pose": self.init_upper_body_pose,
            # target_time 给 InterpolationPolicy.set_goal(): 到该 monotonic 时刻插值到目标.
            # move_duration 控制平滑过渡, 避免关节瞬跳.
            "target_time": t_now + self.move_duration,
            "timestamp": time.time(),
            # InterpolationPolicy.set_goal() 必需: 清掉该时间点之前的旧 waypoint
            "interpolation_garbage_collection_time": t_now - 2 * (1.0 / self.publish_frequency),
        }

    def run(self, duration: float = None):
        """持续发布, 直到超时自动退出或 Ctrl+C.

        Args:
            duration: 持续时长 (秒). main() 默认传 move_duration+1.0 (插值过渡 + 维持
                      稳定后自动退出). None 表示一直发到 Ctrl+C.
        """
        rate = self.node.create_rate(self.publish_frequency)
        start_time = time.monotonic()

        try:
            while self.ros_manager.ok():
                if duration is not None and (time.monotonic() - start_time) >= duration:
                    print(f"Return-to-init complete after {duration:.1f}s")
                    break

                with self.telemetry.timer("publish"):
                    self.publisher.publish(self._build_msg())

                rate.sleep()

        except self.ros_manager.exceptions() as e:
            print(f"Interrupted: {e}")
        finally:
            self.shutdown()

    def shutdown(self):
        self.ros_manager.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(
        description="发布 upper_body 初始关节角, 让双臂回到 run_g1_control_loop 的初始化位置",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--interface", type=str, default="sim", choices=["sim", "real"],
        help="Robot interface (default: sim, 仅用于日志说明, 本脚本不连机器人)",
    )
    parser.add_argument(
        "--waist-location", type=str, default="lower_body",
        choices=["lower_body", "upper_body", "lower_and_upper_body"],
        help="Waist joint assignment (default: lower_body, 应与 run_g1_control_loop 一致)",
    )
    # 默认 high_elbow_pose=True: 控制循环默认用高肘姿态 (configs.py BaseConfig.high_elbow_pose=True),
    # 即 "初始位置" 是高肘. 用 --no-high-elbow-pose 关闭.
    parser.add_argument(
        "--high-elbow-pose", action=argparse.BooleanOptionalAction, default=True,
        help="Use high elbow pose (default: True, 与 run_g1_control_loop 默认一致). "
             "用 --no-high-elbow-pose 关闭",
    )
    parser.add_argument(
        "--move-duration", type=float, default=0.5,
        help="IK 目标运动持续时间 (秒, default: 0.5), 控制 InterpolationPolicy 平滑过渡",
    )
    parser.add_argument(
        "--freq", type=float, default=50.0,
        help="Publishing frequency in Hz (default: 50.0)",
    )
    parser.add_argument(
        "--duration", type=float, default=None,
        help="持续时长 (秒). 默认 None = 自动: move_duration + 1.0s (插值过渡 + 维持稳定后自动退出). "
             "显式传值则按该时长退出; 传负数则一直发到 Ctrl+C",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    publisher = ReturnToInitPosePublisher(
        interface=args.interface,
        waist_location=args.waist_location,
        high_elbow_pose=args.high_elbow_pose,
        move_duration=args.move_duration,
        publish_frequency=args.freq,
    )
    # 默认自动退出: move_duration (InterpolationPolicy 插值过渡) + 1.0s (维持稳定裕量).
    # InterpolationPolicy 对超过 target_time 的查询做夹断返回终点 (__call__ clip),
    # 故 move_duration 后双臂已到达初始位置; 再发 1s 确保稳定维持后退出.
    duration = args.duration if args.duration is not None else (args.move_duration + 1.0)
    if duration < 0:
        duration = None  # 一直发到 Ctrl+C
    publisher.run(duration=duration)


if __name__ == "__main__":
    main()
