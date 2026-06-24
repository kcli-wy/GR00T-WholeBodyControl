"""
Stretcher Task Controller for G1 Robot

State machine-based controller for the stretcher-grabbing task.

Usage:
    # Run full task from beginning
    python run_stretcher_task.py --auto-start

    # Start from specific phase (for testing)
    python run_stretcher_task.py --start-phase grabbing

    # Run only one phase
    python run_stretcher_task.py --start-phase approaching --single-phase

    # Run with custom parameters
    python run_stretcher_task.py --target-height 0.34 --target-torso-rpy 0 60 0
"""

import argparse
import subprocess
import sys
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Type

import numpy as np
from scipy.spatial.transform import Rotation as R

from decoupled_wbc.control.main.constants import (
    CONTROL_GOAL_TOPIC,
    DEFAULT_BASE_HEIGHT,
    DEFAULT_NAV_CMD,
    STATE_TOPIC_NAME,
    STRETCHER_NAV_CMD_TOPIC,
    STRETCHER_POSE_TOPIC,
    STRETCHER_TASK_STATUS_TOPIC,
)
from decoupled_wbc.control.robot_model.instantiation.g1 import instantiate_g1_robot_model
from decoupled_wbc.control.teleop.solver.hand.instantiation.g1_hand_ik_instantiation import (
    instantiate_g1_hand_ik_solver,
)
from decoupled_wbc.control.teleop.teleop_retargeting_ik import TeleopRetargetingIK
from decoupled_wbc.control.utils.ros_utils import ROSManager, ROSMsgPublisher, ROSMsgSubscriber
from decoupled_wbc.control.utils.telemetry import Telemetry


class TaskPhase(Enum):
    """Task phases for the stretcher-grabbing workflow."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    FINE_TUNING = "fine_tuning"
    APPROACHING = "approaching"
    GRABBING = "grabbing"
    STANDING_UP = "standing_up"
    COMPLETED = "completed"


@dataclass
class StretcherHandle:
    """Represents a stretcher handle pose."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))

    @classmethod
    def from_msg(cls, msg: dict) -> "StretcherHandle":
        return cls(
            position=np.array(msg.get("position", [0, 0, 0])),
            orientation=np.array(msg.get("orientation", [1, 0, 0, 0])),
        )


@dataclass
class TaskContext:
    """Shared context across all phases."""
    # Robot commands
    nav_cmd: np.ndarray = field(default_factory=lambda: np.array(DEFAULT_NAV_CMD))
    base_height: float = DEFAULT_BASE_HEIGHT
    torso_rpy: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))

    # Initial values (for returning to default)
    initial_base_height: float = DEFAULT_BASE_HEIGHT
    initial_nav_cmd: np.ndarray = field(default_factory=lambda: np.array(DEFAULT_NAV_CMD))

    # Handle poses from pose estimation
    left_handle: Optional[StretcherHandle] = None
    right_handle: Optional[StretcherHandle] = None

    # Target values for grabbing
    target_height: float = 0.34
    target_torso_rpy: np.ndarray = field(default_factory=lambda: np.array([0.0, 60.0, 0.0]))

    # 默认手腕 position (pelvis 系, 米), 仅在没订阅到 handle 时作为 fallback.
    # orientation 不走默认值 —— 统一由 _compute_wrist_orientation 按 waist_pitch 补偿,
    # 保证 fallback 与实测 handle 朝向语义一致 (世界系下垂直向下).
    default_left_wrist_position: Optional[np.ndarray] = None   # shape (3,)
    default_right_wrist_position: Optional[np.ndarray] = None  # shape (3,)

    # 当前腰部 pitch 实测值 (弧度), 由 controller 订阅 G1Env/env_state_act 持续更新
    # 用于把世界系下的手腕目标朝向补偿到 pelvis 系 (IK 求解参考系)
    current_waist_pitch: float = 0.0


class BasePhase(ABC):
    """Base class for task phases."""

    def __init__(self, context: TaskContext, controller: "StretcherTaskController"):
        self.context = context
        self.controller = controller
        self.start_time = 0.0

    def enter(self):
        """Called when entering this phase."""
        self.start_time = time.monotonic()
        print(f"Entering phase: {self.__class__.__name__}")

    @abstractmethod
    def update(self) -> Optional[TaskPhase]:
        """Update phase logic. Return next phase or None to stay."""
        pass

    def exit(self):
        """Called when exiting this phase."""
        pass

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self.start_time


class IdlePhase(BasePhase):
    """Phase 0: 等待启动指令.

    什么都不发布，只上报状态。
    需要外部调用 start_task() 或 --auto-start 才进入下一阶段。
    """

    def update(self) -> Optional[TaskPhase]:
        self.controller._publish_status("Idle - waiting for start command")
        return None


class NavigatingPhase(BasePhase):
    """Phase 1: VLN 导航到担架附近.

    - 输入: VLN 模型 → StretcherTask/nav_cmd
    - 输出: navigate_cmd → 底盘行走
    - 跳转条件: VLN 发送 arrived=True
    """

    def update(self) -> Optional[TaskPhase]:
        self.controller._publish_status(f"Navigating... ({self.elapsed:.1f}s)")

        # 从 VLN 模型读取导航指令
        nav_msg = self.controller.nav_cmd_subscriber.get_msg()
        if nav_msg and "navigate_cmd" in nav_msg:
            self.context.nav_cmd = np.array(nav_msg["navigate_cmd"])
            self.controller._publish_command(nav_cmd=self.context.nav_cmd)

        # VLN 报告到达
        if nav_msg and nav_msg.get("arrived", False):
            print("VLN reports arrival at stretcher")
            return TaskPhase.FINE_TUNING

        return None


class FineTuningPhase(BasePhase):
    """Phase 2: 基于位姿估计的微调定位.

    - 输入: FoundationPose++ → StretcherTask/pose
    - 输出: navigate_cmd → 底盘微调位置
    - 副作用: 更新 context.left_handle / right_handle
    - 跳转条件: 位姿估计发送 ready_to_grab=True
    """

    def update(self) -> Optional[TaskPhase]:
        self.controller._publish_status(f"Fine-tuning position... ({self.elapsed:.1f}s)")

        # 更新 handle pose (存储到 context 供后续 IK 使用)
        self.controller._update_handles()

        # 从位姿估计读取微调导航指令
        pose_msg = self.controller.pose_subscriber.get_msg()
        if pose_msg and "navigate_cmd" in pose_msg:
            self.context.nav_cmd = np.array(pose_msg["navigate_cmd"])
            self.controller._publish_command(nav_cmd=self.context.nav_cmd)

        # 位姿估计报告就绪
        if pose_msg and pose_msg.get("ready_to_grab", False):
            print("Pose estimation reports ready to grab")
            return TaskPhase.APPROACHING

        return None


class ApproachingPhase(BasePhase):
    """Phase 3a: 下蹲 + 弯腰 (同步插值).

    - 输出: base_height_command + torso_orientation_rpy (同步线性插值)
    - 效果: 机器人从站立姿态平滑过渡到弯腰下蹲姿态
    - 跳转条件: 时间到 (默认 2s)

    注意: height 和 waist_rpy 使用同一个 progress 同步插值，
    不会先弯腰再下蹲。
    """

    def __init__(self, context: TaskContext, controller: "StretcherTaskController", duration: float = 2.0):
        super().__init__(context, controller)
        self.duration = duration

    def update(self) -> Optional[TaskPhase]:
        progress = self.elapsed / self.duration

        self.controller._publish_status(
            f"Approaching stretcher... ({self.elapsed:.1f}s)",
            progress=min(progress, 1.0),
        )

        if progress >= 1.0:
            print("Approach complete, starting grab")
            return TaskPhase.GRABBING

        # 同步插值: height 和 waist_rpy 共享同一个 progress
        current_height = self._lerp(
            self.context.initial_base_height, self.context.target_height, progress
        )
        current_rpy = self._lerp_array(
            np.array([0.0, 0.0, 0.0]), self.context.target_torso_rpy, progress
        )

        # 停止行走，同步下蹲+弯腰
        self.controller._publish_command(
            nav_cmd=np.array([0.0, 0.0, 0.0]),
            base_height=current_height,
            torso_rpy=current_rpy,
        )

        return None

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        return a + (b - a) * min(max(t, 0.0), 1.0)

    @staticmethod
    def _lerp_array(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
        return a + (b - a) * min(max(t, 0.0), 1.0)


class GrabbingPhase(BasePhase):
    """Phase 3b: IK 求解 + 抓取担架把手.

    - 输入: FoundationPose++ → handle pose (持续更新)
    - IK 求解: handle pose → 4x4 wrist 矩阵 → TeleopRetargetingIK → 关节角
    - 输出: target_upper_body_pose (双臂关节角) + base_height + torso_rpy
    - 副作用: 50% 进度时启动外部抓取脚本
    - 跳转条件: 时间到 (默认 3s)

    与 APPROACHING 不同，本阶段直接得到 IK 目标关节角 (无中间状态)，
    因此需要通过 target_time 指定运动持续时间，让 InterpolationPolicy
    生成平滑轨迹。target_time = 当前时间 + move_duration。
    """

    def __init__(
        self,
        context: TaskContext,
        controller: "StretcherTaskController",
        duration: float = 3.0,
        move_duration: float = 0.5,
        grab_script: str = None,
    ):
        super().__init__(context, controller)
        self.duration = duration
        self.move_duration = move_duration  # IK 目标运动的持续时间 (秒)
        self.grab_script = grab_script
        self._grab_script_triggered = False

    def enter(self):
        super().enter()
        self._grab_script_triggered = False

    def update(self) -> Optional[TaskPhase]:
        progress = self.elapsed / self.duration

        self.controller._publish_status(
            f"Grabbing stretcher... ({self.elapsed:.1f}s)",
            progress=min(progress, 1.0),
        )

        if progress >= 1.0:
            print("Grab phase complete")
            return TaskPhase.STANDING_UP

        # 持续更新 handle pose
        self.controller._update_handles()

        # IK 求解: 得到双臂关节角
        upper_body_pose = self.controller._solve_ik_for_handles()

        if upper_body_pose is not None:
            t_now = time.monotonic()
            self.controller._publish_command(
                nav_cmd=np.array([0.0, 0.0, 0.0]),
                base_height=self.context.target_height,
                torso_rpy=self.context.target_torso_rpy,
                target_upper_body_pose=upper_body_pose,
                # target_time 指定运动持续时间，让 InterpolationPolicy 平滑过渡
                # 如果用 1/freq (一个控制周期)，机器人会瞬间跳到目标
                target_time=t_now + self.move_duration,
            )

        # 50% 进度时启动抓取脚本
        if progress >= 0.5 and not self._grab_script_triggered:
            self._run_grab_script()
            self._grab_script_triggered = True

        return None

    def _run_grab_script(self):
        if self.grab_script is None:
            print("WARNING: No grab script specified")
            return

        print(f"Running grab script: {self.grab_script}")
        try:
            subprocess.Popen([sys.executable, self.grab_script])
            print("Grab script started")
        except Exception as e:
            print(f"ERROR: Failed to run grab script: {e}")


class StandingUpPhase(BasePhase):
    """Phase 4: 抬起担架 (反向插值回站立姿态).

    - 输出: base_height + torso_rpy (反向插值) + IK 求解的 upper body 关节角
    - 效果: 双手保持抓握, pelvis 系下手腕 z 从 GRABBING 末尾值线性插值到 handle_z_target,
            xy 冻结; orientation 由 _compute_wrist_orientation 用最新 waist_pitch 补偿,
            保证手腕在世界系下始终垂直向下, 担架被水平抬起.
    - 跳转条件: 时间到 (默认 3s)

    注: 站起期间相机看不到 handle, STRETCHER_POSE_TOPIC 不再发布, 所以本阶段
    直接就地覆写 context.{left,right}_handle.position[2], 不依赖外部位姿估计.
    """

    def __init__(
        self,
        context: TaskContext,
        controller: "StretcherTaskController",
        duration: float = 3.0,
        handle_z_target: float = 0.0,
        move_duration: float = 0.5,
    ):
        super().__init__(context, controller)
        self.duration = duration
        # pelvis 系下手腕 z 的最终目标 (米), 单一标量左右共用
        self.handle_z_target = handle_z_target
        # IK 目标传给下游 InterpolationPolicy 的过渡时间 (秒)
        self.move_duration = move_duration
        self._initial_left_z: float = 0.0
        self._initial_right_z: float = 0.0

    def enter(self):
        super().enter()
        # 缓存 GRABBING 末尾的手腕 z (pelvis 系下), 作为站起插值的起点
        if self.context.left_handle is not None:
            self._initial_left_z = float(self.context.left_handle.position[2])
        else:
            print("WARNING: StandingUpPhase entered without left_handle, fallback z=0.0")
            self._initial_left_z = 0.0
        if self.context.right_handle is not None:
            self._initial_right_z = float(self.context.right_handle.position[2])
        else:
            print("WARNING: StandingUpPhase entered without right_handle, fallback z=0.0")
            self._initial_right_z = 0.0

    def update(self) -> Optional[TaskPhase]:
        progress = self.elapsed / self.duration

        self.controller._publish_status(
            f"Standing up... ({self.elapsed:.1f}s)",
            progress=min(progress, 1.0),
        )

        if progress >= 1.0:
            print("Task completed!")
            return TaskPhase.COMPLETED

        # 反向插值: 从目标姿态回到初始站立姿态
        current_height = self._lerp(
            self.context.target_height, self.context.initial_base_height, progress
        )
        current_rpy = self._lerp_array(
            self.context.target_torso_rpy, np.array([0.0, 0.0, 0.0]), progress
        )

        # 就地覆写 handle.position 的 z 分量 (xy 不动), 让 IK 跟随担架被抬起
        if self.context.left_handle is not None:
            self.context.left_handle.position[2] = self._lerp(
                self._initial_left_z, self.handle_z_target, progress
            )
        if self.context.right_handle is not None:
            self.context.right_handle.position[2] = self._lerp(
                self._initial_right_z, self.handle_z_target, progress
            )

        # IK 求解 (会自动刷新 waist_pitch 并用补偿后的世界系朝向求解)
        upper_body_pose = self.controller._solve_ik_for_handles()

        t_now = time.monotonic()
        self.controller._publish_command(
            nav_cmd=self.context.initial_nav_cmd,
            base_height=current_height,
            torso_rpy=current_rpy,
            target_upper_body_pose=upper_body_pose,
            # 给下游 InterpolationPolicy 一段过渡时间, 避免关节瞬跳
            target_time=t_now + self.move_duration,
        )

        return None

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        return a + (b - a) * min(max(t, 0.0), 1.0)

    @staticmethod
    def _lerp_array(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
        return a + (b - a) * min(max(t, 0.0), 1.0)


class StretcherTaskController:
    """
    State machine-based controller for the stretcher-grabbing task.
    """

    def __init__(
        self,
        target_height: float = 0.34,
        target_torso_rpy: List[float] = [0.0, 60.0, 0.0],
        default_left_wrist_position: List[float] = None,
        default_right_wrist_position: List[float] = None,
        grab_duration: float = 3.0,
        move_duration: float = 0.5,
        approach_duration: float = 2.0,
        standup_duration: float = 3.0,
        standup_handle_z_target: float = 0.0,
        publish_frequency: float = 50.0,
        grab_script: str = None,
        single_phase: bool = False,
    ):
        self.publish_frequency = publish_frequency
        self.single_phase = single_phase

        # Task context (shared state)
        self.context = TaskContext(
            target_height=target_height,
            target_torso_rpy=np.array(target_torso_rpy),
            default_left_wrist_position=np.array(default_left_wrist_position, dtype=float) if default_left_wrist_position else None,
            default_right_wrist_position=np.array(default_right_wrist_position, dtype=float) if default_right_wrist_position else None,
        )

        # Initialize ROS 2
        self.ros_manager = ROSManager(node_name="StretcherTaskController")
        self.node = self.ros_manager.node

        # Publishers
        self.cmd_publisher = ROSMsgPublisher(CONTROL_GOAL_TOPIC)
        self.status_publisher = ROSMsgPublisher(STRETCHER_TASK_STATUS_TOPIC)

        # Subscribers
        self.nav_cmd_subscriber = ROSMsgSubscriber(STRETCHER_NAV_CMD_TOPIC)
        self.pose_subscriber = ROSMsgSubscriber(STRETCHER_POSE_TOPIC)
        # 机器人状态订阅 (50Hz): 用于读取实测 waist_pitch 做 IK 目标朝向补偿
        self.state_subscriber = ROSMsgSubscriber(STATE_TOPIC_NAME)

        # Initialize robot model for IK
        self.robot_model = instantiate_g1_robot_model(
            waist_location="lower_body",
            high_elbow_pose=False,
        )

        # Initialize IK solvers
        left_hand_ik_solver, right_hand_ik_solver = instantiate_g1_hand_ik_solver()

        # Initialize retargeting IK
        self.retargeting_ik = TeleopRetargetingIK(
            robot_model=self.robot_model,
            left_hand_ik_solver=left_hand_ik_solver,
            right_hand_ik_solver=right_hand_ik_solver,
            enable_visualization=False,
            body_active_joint_groups=["upper_body"],
        )

        # Get joint group indices
        self.upper_body_indices = self.robot_model.get_joint_group_indices("upper_body")
        # waist 关节组顺序为 [yaw, roll, pitch] (见 g1_supplemental_info.py:60-62);
        # 取 pitch 在完整 q 向量中的索引, 用于 _update_robot_state 解析实测值
        self.waist_pitch_idx = self.robot_model.get_joint_group_indices("waist")[2]

        # Initialize phases
        self.phases: Dict[TaskPhase, BasePhase] = {
            TaskPhase.IDLE: IdlePhase(self.context, self),
            TaskPhase.NAVIGATING: NavigatingPhase(self.context, self),
            TaskPhase.FINE_TUNING: FineTuningPhase(self.context, self),
            TaskPhase.APPROACHING: ApproachingPhase(self.context, self, approach_duration),
            TaskPhase.GRABBING: GrabbingPhase(self.context, self, grab_duration, move_duration, grab_script),
            TaskPhase.STANDING_UP: StandingUpPhase(
                self.context,
                self,
                duration=standup_duration,
                handle_z_target=standup_handle_z_target,
                move_duration=move_duration,
            ),
            TaskPhase.COMPLETED: IdlePhase(self.context, self),  # Reuse idle for completed
        }

        # Current state
        self.current_phase: BasePhase = self.phases[TaskPhase.IDLE]
        self.current_phase_enum = TaskPhase.IDLE

        print(f"StretcherTaskController initialized")
        print(f"  Target height: {target_height}m, Target torso RPY: {target_torso_rpy}deg")
        if single_phase:
            print(f"  Mode: single-phase (will stop after one phase)")

    def _transition_to(self, new_phase_enum: TaskPhase):
        """Transition to a new phase."""
        if new_phase_enum not in self.phases:
            print(f"ERROR: Unknown phase {new_phase_enum}")
            return

        print(f"Transitioning: {self.current_phase_enum.value} -> {new_phase_enum.value}")

        # Exit current phase
        self.current_phase.exit()

        # Update state
        self.current_phase_enum = new_phase_enum
        self.current_phase = self.phases[new_phase_enum]

        # Enter new phase
        self.current_phase.enter()

    def set_start_phase(self, phase_name: str):
        """Set the starting phase for testing.

        Args:
            phase_name: Name of the phase to start from
        """
        try:
            phase_enum = TaskPhase(phase_name)
            print(f"Setting start phase to: {phase_name}")
            self._transition_to(phase_enum)
        except ValueError:
            valid_phases = [p.value for p in TaskPhase]
            print(f"ERROR: Invalid phase '{phase_name}'. Valid phases: {valid_phases}")
            sys.exit(1)

    def start_task(self):
        """Start the task from current phase."""
        if self.current_phase_enum == TaskPhase.IDLE:
            self._transition_to(TaskPhase.NAVIGATING)
        else:
            print(f"Task already in phase: {self.current_phase_enum.value}")

    def _publish_command(
        self,
        nav_cmd: np.ndarray = None,
        base_height: float = None,
        torso_rpy: np.ndarray = None,
        target_upper_body_pose: np.ndarray = None,
        target_time: float = None,
    ):
        """Publish a command to the robot control loop."""
        if nav_cmd is None:
            nav_cmd = self.context.nav_cmd
        if base_height is None:
            base_height = self.context.base_height
        if torso_rpy is None:
            torso_rpy = self.context.torso_rpy

        t_now = time.monotonic()

        msg = {
            "base_height_command": base_height,
            "navigate_cmd": nav_cmd,
            "torso_orientation_rpy": np.deg2rad(torso_rpy),
            "timestamp": time.time(),
            # target_time 是 InterpolationPolicy.set_goal() 的必需字段
            # 没有它 base_height_command 和 navigate_cmd 都会被忽略
            "target_time": target_time if target_time else t_now + (1.0 / self.publish_frequency),
            "interpolation_garbage_collection_time": t_now - 2 * (1.0 / self.publish_frequency),
        }

        if target_upper_body_pose is not None:
            msg["target_upper_body_pose"] = target_upper_body_pose

        self.cmd_publisher.publish(msg)

        # Log published command
        arm_str = "N/A"
        if target_upper_body_pose is not None:
            n = len(target_upper_body_pose)
            mid = n // 2
            left = np.rad2deg(target_upper_body_pose[:mid])
            right = np.rad2deg(target_upper_body_pose[mid:])
            arm_str = f"L=[{', '.join(f'{v:.1f}' for v in left)}] R=[{', '.join(f'{v:.1f}' for v in right)}]"
        # torso_rpy 本身是度，直接打印
        print(
            f"[CMD] phase={self.current_phase_enum.value:12s} "
            f"nav=[{nav_cmd[0]:+.3f}, {nav_cmd[1]:+.3f}, {nav_cmd[2]:+.3f}] "
            f"h={base_height:.3f}m "
            f"waist_rpy=[{torso_rpy[0]:+.1f}, {torso_rpy[1]:+.1f}, {torso_rpy[2]:+.1f}]deg "
            f"arms={arm_str}"
        )

    def _publish_status(self, status: str, progress: float = 0.0):
        """Publish task status."""
        msg = {
            "phase": self.current_phase_enum.value,
            "status": status,
            "progress": progress,
            "timestamp": time.time(),
        }
        self.status_publisher.publish(msg)

    def _update_handles(self):
        """Update stretcher handle poses from subscriber."""
        pose_msg = self.pose_subscriber.get_msg()
        if pose_msg:
            if "left_handle" in pose_msg:
                self.context.left_handle = StretcherHandle.from_msg(pose_msg["left_handle"])
            if "right_handle" in pose_msg:
                self.context.right_handle = StretcherHandle.from_msg(pose_msg["right_handle"])

    def _update_robot_state(self):
        """从 G1Env/env_state_act 读取最新 q, 更新 context.current_waist_pitch.

        消息缺失时保留旧值 (隐式 fallback), 不会导致 IK 求解中断.
        """
        state_msg = self.state_subscriber.get_msg()
        if state_msg is None:
            return
        q = state_msg.get("q")
        if q is None:
            return
        self.context.current_waist_pitch = float(q[self.waist_pitch_idx])

    def _compute_wrist_orientation(self, handle: StretcherHandle, side: str) -> np.ndarray:
        """计算 IK 用的手腕 4x4 目标位姿 (pelvis 系下).

        语义:
        - 目标朝向在 *世界系* 下固定为 "绕世界 Y +90°" (手腕垂直向下),
          左右手共用同一矩阵 (零位时左右腕朝向相同, IK 会自然解出对称关节角).
        - IK 求解参考系是 pelvis (见 body_ik_solver.py: pin.SE3 喂给 Pink FrameTask).
          pelvis 自身随 waist_pitch 旋转, 所以需要把世界系朝向左乘 R_y(-waist_pitch)
          补偿回 pelvis 系: R_pelvis_target = R_y(-waist_pitch) · R_y(+90°)
        - position 不补偿, 直接用 handle.position (调用方保证它已经在 pelvis 系下).
        - handle.orientation 不再被读取 (FoundationPose++ 给的朝向估计当前不可靠).
        - side 参数保留作占位, 当前未使用.
        """
        del side  # 当前左右手用同一朝向矩阵, IK 靠 frame name 区分左右臂

        R_world_target = R.from_euler('y', 90, degrees=True)
        R_compensate = R.from_euler('y', -self.context.current_waist_pitch, degrees=False)
        R_pelvis_target = R_compensate * R_world_target

        T = np.eye(4)
        T[:3, :3] = R_pelvis_target.as_matrix()
        T[:3, 3] = handle.position
        return T

    def _solve_ik_for_handles(self) -> Optional[np.ndarray]:
        """Solve IK for both hands to reach stretcher handles.

        左右手独立处理: 优先用订阅到的实测 handle.position;
        某侧 handle 缺失时, 退回该侧 default wrist position.
        orientation 不分实测/fallback —— 统一走 _compute_wrist_orientation,
        按当前 waist_pitch 补偿, 保证世界系下双手腕始终垂直向下.
        """
        # 先刷新 waist_pitch 实测值, _compute_wrist_orientation 依赖它做补偿
        self._update_robot_state()

        left_handle = self.context.left_handle
        if left_handle is None:
            if self.context.default_left_wrist_position is not None:
                print("WARNING: left handle not available, using default position")
                left_handle = StretcherHandle(
                    position=np.array(self.context.default_left_wrist_position, dtype=float)
                )
            else:
                print("WARNING: left handle and default position both unavailable")
                return None

        right_handle = self.context.right_handle
        if right_handle is None:
            if self.context.default_right_wrist_position is not None:
                print("WARNING: right handle not available, using default position")
                right_handle = StretcherHandle(
                    position=np.array(self.context.default_right_wrist_position, dtype=float)
                )
            else:
                print("WARNING: right handle and default position both unavailable")
                return None

        left_wrist = self._compute_wrist_orientation(left_handle, "left")
        right_wrist = self._compute_wrist_orientation(right_handle, "right")

        body_data = {
            "left_wrist_yaw_link": left_wrist,
            "right_wrist_yaw_link": right_wrist,
        }
        print("========================================")
        print(body_data)
        print("========================================")
        self.retargeting_ik.set_goal({
            "body_data": body_data,
            "left_hand_data": {"position": np.zeros((25, 4, 4))},
            "right_hand_data": {"position": np.zeros((25, 4, 4))},
        })

        try:
            return self.retargeting_ik.get_action()
        except Exception as e:
            print(f"IK failed: {e}")
            return None

    def run(self, duration: float = None):
        """Run the task controller."""
        rate = self.node.create_rate(self.publish_frequency)
        start_time = time.monotonic()

        try:
            while self.ros_manager.ok():
                if duration and (time.monotonic() - start_time) >= duration:
                    print(f"Task duration limit reached ({duration}s)")
                    break

                # Update current phase
                next_phase = self.current_phase.update()

                # Transition if needed
                if next_phase is not None:
                    if self.single_phase:
                        print(f"[SINGLE-PHASE] Phase '{self.current_phase_enum.value}' done, "
                              f"would transition to '{next_phase.value}' — stopping.")
                        self._publish_status(f"Single phase '{self.current_phase_enum.value}' completed")
                        break
                    self._transition_to(next_phase)

                # Check for completion
                if self.current_phase_enum == TaskPhase.COMPLETED:
                    self._publish_status("Task completed")
                    break

                rate.sleep()

        except self.ros_manager.exceptions() as e:
            print(f"Task interrupted: {e}")
        finally:
            self.shutdown()

    def shutdown(self):
        """Clean up resources."""
        self.ros_manager.shutdown()


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Stretcher task controller for G1 robot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run full task from beginning
  python run_stretcher_task.py --auto-start

  # Start from specific phase (for testing)
  python run_stretcher_task.py --start-phase grabbing

  # Run only one phase
  python run_stretcher_task.py --start-phase approaching --single-phase

  # Run with custom parameters
  python run_stretcher_task.py --target-height 0.34 --target-torso-rpy 0 60 0

Valid phases: idle, navigating, fine_tuning, approaching, grabbing, standing_up, completed
        """,
    )

    parser.add_argument(
        "--start-phase",
        type=str,
        default="idle",
        help="Phase to start from (default: idle)",
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Automatically start the task (transition from idle to navigating)",
    )
    parser.add_argument(
        "--target-height",
        type=float,
        default=0.34,
        help="Target base height for grabbing in meters (default: 0.34)",
    )
    parser.add_argument(
        "--target-torso-rpy",
        type=float,
        nargs=3,
        default=[0.0, 60.0, 0.0],
        metavar=("ROLL", "PITCH", "YAW"),
        help="Target torso [roll, pitch, yaw] in degrees (default: 0 60 0)",
    )
    parser.add_argument(
        "--grab-duration",
        type=float,
        default=3.0,
        help="Duration for grab phase in seconds (default: 3.0)",
    )
    parser.add_argument(
        "--move-duration",
        type=float,
        default=0.5,
        help="Duration for IK target motion in grab phase (seconds, default: 0.5). "
             "Controls how fast the arms move to the IK target position.",
    )
    parser.add_argument(
        "--approach-duration",
        type=float,
        default=2.0,
        help="Duration for approach phase in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--standup-duration",
        type=float,
        default=3.0,
        help="Duration for stand-up phase in seconds (default: 3.0)",
    )
    parser.add_argument(
        "--standup-handle-z-target",
        type=float,
        default=0.0,
        help="Stand-up 阶段手腕在 pelvis 系下 z 的最终目标 (米, 左右共用). "
             "GRABBING 末尾的 z 会沿直线插值到该值, xy 冻结. 默认 0.0",
    )
    parser.add_argument(
        "--freq",
        type=float,
        default=50.0,
        help="Publishing frequency in Hz (default: 50.0)",
    )
    parser.add_argument(
        "--grab-script",
        type=str,
        default=None,
        help="Path to grab script to run via subprocess",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Maximum task duration in seconds (None for indefinite)",
    )
    parser.add_argument(
        "--default-left-wrist-position",
        type=float,
        nargs=3,
        default=[0.25, 0.3, 0.1],
        metavar=("X", "Y", "Z"),
        help="Default left wrist position [x y z] (m, pelvis frame) for IK when left handle is unavailable; "
             "orientation is always waist_pitch-compensated, this only supplies position",
    )
    parser.add_argument(
        "--default-right-wrist-position",
        type=float,
        nargs=3,
        default=[0.25, -0.3, 0.1],
        metavar=("X", "Y", "Z"),
        help="Default right wrist position [x y z] (m, pelvis frame) for IK when right handle is unavailable; "
             "orientation is always waist_pitch-compensated, this only supplies position",
    )
    parser.add_argument(
        "--single-phase",
        action="store_true",
        help="Run only the specified --start-phase, stop when it completes instead of transitioning",
    )

    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_args()

    # Create controller
    controller = StretcherTaskController(
        target_height=args.target_height,
        target_torso_rpy=args.target_torso_rpy,
        default_left_wrist_position=args.default_left_wrist_position,
        default_right_wrist_position=args.default_right_wrist_position,
        grab_duration=args.grab_duration,
        move_duration=args.move_duration,
        approach_duration=args.approach_duration,
        standup_duration=args.standup_duration,
        standup_handle_z_target=args.standup_handle_z_target,
        publish_frequency=args.freq,
        grab_script=args.grab_script,
        single_phase=args.single_phase,
    )

    try:
        # Set starting phase
        if args.start_phase != "idle":
            controller.set_start_phase(args.start_phase)

        # Auto-start if requested
        if args.auto_start and args.start_phase == "idle":
            controller.start_task()

        # Run controller
        controller.run(duration=args.duration)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
