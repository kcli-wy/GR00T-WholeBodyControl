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
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Type

import numpy as np
from scipy.spatial.transform import Rotation as R

from decoupled_wbc.control.main.constants import (
    CONTROL_GOAL_TOPIC,
    DEFAULT_BASE_HEIGHT,
    DEFAULT_NAV_CMD,
    DEFAULT_STRETCHER_POSE_TOPIC_PREFIX,
    STATE_TOPIC_NAME,
    STRETCHER_LEFT_HANDLE_ID,
    STRETCHER_NAV_CMD_TOPIC,
    STRETCHER_RIGHT_HANDLE_ID,
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
    """Represents a stretcher handle pose.

    只保留 position (pelvis 系, 米). orientation 故意不存 —— 手腕目标朝向统一由
    _compute_wrist_orientation 按 waist_pitch 补偿算出 (世界系下 R_y(90°) 垂直向下),
    不使用 FoundationPose++ 给的朝向估计.
    """
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @classmethod
    def from_msg(cls, msg: dict) -> "StretcherHandle":
        """从 FoundationPose++ 单条消息解析 handle position.

        msg["pose_robot_matrix"] 是 4x4 齐次变换矩阵 (机器人坐标系, 已由
        camera_to_robot 从相机系转换). position 已落到 IK 求解参考系 (pelvis 系),
        直接取平移列. 不做防御: 缺字段即认为是真 bug, 由调用方判 None 走 fallback.
        """
        pose = np.array(msg["pose_robot_matrix"])
        return cls(position=pose[:3, 3].copy())


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
    # handle 锁定标志: GRABBING settle 结束后置 True, 之后 _update_handles 不再刷新
    # context 里的 handle (STANDING_UP 期间就地改 position.z 不受此限制 —— 锁的是
    # "不从外部 pose 重新读取", 不是 "不许改 position")
    handle_locked: bool = False

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
    """Phase 2: 基于 handle 位姿的近场微调 (PD 闭环).

    - 输入: 两个 FoundationPose++ handle pose topic (持续刷新 context)
    - 输出: navigate_cmd → 底盘 PD 微调
    - 闭环: 用左右 handle 均值算 x/y 误差, 用左右 handle x 差算 θ 误差,
            PD (P + D) 输出 [vx, vy, vθ], 各维限幅.
            D 项对 EWMA 低通后的误差求差分, 抑制 pose 抖动放大.
    - 跳转条件: 误差连续 N 帧 < tol → APPROACHING
    - 不再订阅 navigate_cmd / ready_to_grab (旧 VLN/Pose 桥接的字段, 已剥离).

    目标抓取窗 (pelvis 系, 硬编码):
      - x = target_handle_x  (CLI, 默认 0.45m, 实机调)
      - y = 0                (左右 handle y 均值对到机器人中线)
      - z 不在本阶段管       (交给后续 APPROACHING 弯腰)
    """

    def __init__(
        self,
        context: TaskContext,
        controller: "StretcherTaskController",
        target_handle_x: float = 0.45,
        kp_x: float = 1.0,
        kp_y: float = 1.0,
        kp_theta: float = 1.0,
        kd_x: float = 0.0,
        kd_y: float = 0.0,
        kd_theta: float = 0.0,
        d_alpha: float = 0.5,
        max_nav_speed: List[float] = None,
        tol: float = 0.03,
        tol_theta: float = 0.05,
        converge_frames: int = 10,
    ):
        super().__init__(context, controller)
        self.target_handle_x = target_handle_x
        self.kp = np.array([kp_x, kp_y, kp_theta], dtype=float)
        self.kd = np.array([kd_x, kd_y, kd_theta], dtype=float)
        self.d_alpha = d_alpha  # EWMA 低通系数: filt = alpha*filt + (1-alpha)*err
        self.max_nav_speed = np.array(max_nav_speed if max_nav_speed else [0.1, 0.1, 0.1], dtype=float)
        self.tol = tol
        self.tol_theta = tol_theta
        self.converge_frames = converge_frames
        # 运行态: 在 enter() 重置
        self._filt_err = np.zeros(3)
        self._last_update_time = 0.0
        self._converge_count = 0

    def enter(self):
        super().enter()
        self._filt_err = np.zeros(3)
        self._last_update_time = time.monotonic()
        self._converge_count = 0

    def _compute_error(self) -> np.ndarray:
        """计算 [err_x, err_y, err_theta].

        err_x = target_x - mean_x       (mean_x = (x_left + x_right)/2)
        err_y = -mean_y                 (mean_y = (y_left + y_right)/2, 对到中线)
        err_theta = x_right - x_left    (担架偏航: 正对时 ~0; 符号方向实机翻 Kp_theta)
        """
        lx, ly, _ = self.context.left_handle.position
        rx, ry, _ = self.context.right_handle.position
        mean_x = (lx + rx) / 2.0
        mean_y = (ly + ry) / 2.0
        return np.array([
            self.target_handle_x - mean_x,
            -mean_y,
            rx - lx,
        ])

    def update(self) -> Optional[TaskPhase]:
        self.controller._publish_status(f"Fine-tuning position... ({self.elapsed:.1f}s)")

        # 刷新最新 handle (FineTuning 用单帧最新值做实时反馈, 锁定用 buffer 是 GRABBING 的事)
        self.controller._update_handles()

        # 任一侧还没收到 pose → 停车等待, 不解算
        if self.context.left_handle is None or self.context.right_handle is None:
            self.controller._publish_command(nav_cmd=np.array([0.0, 0.0, 0.0]))
            return None

        err = self._compute_error()

        # D 项: 对误差做 EWMA 低通后再求差分, 避免把 pose 抖动直接放大进 nav_cmd
        prev_filt_err = self._filt_err.copy()
        self._filt_err = self.d_alpha * self._filt_err + (1.0 - self.d_alpha) * err
        t_now = time.monotonic()
        dt = t_now - self._last_update_time
        if dt <= 0.0:
            dt = 1e-3  # 防除零 (同 tick 或时钟回退)
        d_err = (self._filt_err - prev_filt_err) / dt
        self._last_update_time = t_now

        nav_cmd = self.kp * err + self.kd * d_err
        # 各维独立限幅
        nav_cmd = np.clip(nav_cmd, -self.max_nav_speed, self.max_nav_speed)

        self.controller._publish_command(nav_cmd=nav_cmd)

        # 收敛判定: 三维误差同时小于阈值, 连续 N 帧才退出 (防 pose 抖动假退出)
        converged = (abs(err[0]) < self.tol
                     and abs(err[1]) < self.tol
                     and abs(err[2]) < self.tol_theta)
        if converged:
            self._converge_count += 1
            if self._converge_count >= self.converge_frames:
                print(f"Fine-tuning converged: err={err}, holding for APPROACHING")
                return TaskPhase.APPROACHING
        else:
            self._converge_count = 0

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
    """Phase 3b: 锁定 handle + IK 求解 + 抓取担架把手.

    分两段 (非阻塞, 不卡主循环):

    1) settle 段 (enter 后 lock_settle_time 秒):
       - 持续 _update_handles() 把最新 handle push 进锁定 buffer (此时 handle_locked=False,
         _update_handles 正常刷新 context)
       - 不发任何 command —— 机器人靠 InterpolationPolicy 对超出 target_time 的查询做
         夹断 (返回终点姿态), 维持 APPROACHING 末态的弯腰姿态不松
       - 到点 _lock_handles(): 对 buffer 最近 N 帧取均值写 context, 置 handle_locked=True
    2) 正常抓取段 (settle 后 duration 秒):
       - handle_locked=True, _update_handles 不再刷新 (锁定值不变)
       - _solve_ik_for_handles() 用锁定 handle 解 IK
       - 发 target_upper_body_pose + 维持 base_height/torso_rpy, 50% 触发 grab_script
       - 跳转条件: progress>=1.0 → STANDING_UP

    progress 从锁定时刻起算 (不把 settle 算进抓取时间).
    """

    def __init__(
        self,
        context: TaskContext,
        controller: "StretcherTaskController",
        duration: float = 3.0,
        move_duration: float = 0.5,
        grab_script: str = None,
        lock_settle_time: float = 1.5,
        lock_window: int = 10,
    ):
        super().__init__(context, controller)
        self.duration = duration
        self.move_duration = move_duration  # IK 目标运动的持续时间 (秒)
        self.grab_script = grab_script
        self.lock_settle_time = lock_settle_time
        self.lock_window = lock_window
        self._grab_script_triggered = False
        # settle 倒计时剩余 (秒); >0 表示还在 settle 段
        self._lock_remaining = lock_settle_time
        # 正常抓取段起始时刻 (锁定完成时设置)
        self._grab_start_time = 0.0
        # settle 倒计时用实测 dt (受系统负载影响, 不假设 50Hz)
        self._last_update_time = 0.0

    def enter(self):
        super().enter()
        self._grab_script_triggered = False
        self._lock_remaining = self.lock_settle_time
        self._last_update_time = time.monotonic()
        # 清空锁定 buffer, settle 期间全新采集 (APPROACHING 不刷 handle, 无残留)
        self.controller._left_handle_buffer.clear()
        self.controller._right_handle_buffer.clear()

    def update(self) -> Optional[TaskPhase]:
        # ---- settle 段: 采 handle, 不发 command ----
        if self._lock_remaining > 0.0:
            self.controller._publish_status(
                f"Grabbing: settling pose ({self.lock_settle_time - self._lock_remaining:.1f}/{self.lock_settle_time}s)",
            )
            self.controller._update_handles()  # 采最新 handle 进 buffer + context
            t_now = time.monotonic()
            self._lock_remaining -= (t_now - self._last_update_time)
            self._last_update_time = t_now
            if self._lock_remaining <= 0.0:
                self._lock_remaining = 0.0
                self.controller._lock_handles()
                self._grab_start_time = time.monotonic()
            return None

        # ---- 正常抓取段 ----
        grab_elapsed = time.monotonic() - self._grab_start_time
        progress = grab_elapsed / self.duration

        self.controller._publish_status(
            f"Grabbing stretcher... ({grab_elapsed:.1f}s)",
            progress=min(progress, 1.0),
        )

        if progress >= 1.0:
            print("Grab phase complete")
            return TaskPhase.STANDING_UP

        # handle 已锁定, _update_handles 直接 return (不刷新); 直接解 IK
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

    注: 站起期间相机看不到 handle, FoundationPose++ 不再发布 handle pose,
    所以本阶段直接就地覆写 context.{left,right}_handle.position[2], 不依赖外部位姿估计.
    此时 handle 已在 GRABBING settle 末锁定 (context.handle_locked=True), 就地改 z 不受锁定影响.
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
        pose_topic_prefix: str = DEFAULT_STRETCHER_POSE_TOPIC_PREFIX,
        # GRABBING 阶段: settle 时长 + 锁定窗口
        grab_lock_settle_time: float = 1.5,
        grab_lock_window: int = 10,
        grab_duration: float = 3.0,
        move_duration: float = 0.5,
        approach_duration: float = 2.0,
        standup_duration: float = 3.0,
        standup_handle_z_target: float = 0.0,
        publish_frequency: float = 50.0,
        grab_script: str = None,
        single_phase: bool = False,
        # FINE_TUNING 阶段 PD 控制器参数 (实机调默认占位值)
        finetune_target_handle_x: float = 0.45,
        finetune_kp_x: float = 1.0,
        finetune_kp_y: float = 1.0,
        finetune_kp_theta: float = 1.0,
        finetune_kd_x: float = 0.0,
        finetune_kd_y: float = 0.0,
        finetune_kd_theta: float = 0.0,
        finetune_d_alpha: float = 0.5,
        finetune_max_nav_speed: List[float] = None,
        finetune_tol: float = 0.03,
        finetune_tol_theta: float = 0.05,
        finetune_converge_frames: int = 10,
    ):
        self.publish_frequency = publish_frequency
        self.single_phase = single_phase
        # GRABBING settle 期间采 handle 的窗口大小 (传给 GrabbingPhase)
        self.grab_lock_settle_time = grab_lock_settle_time
        self.grab_lock_window = grab_lock_window
        # FINE_TUNING PD 参数
        self.finetune_target_handle_x = finetune_target_handle_x
        self.finetune_kp_x = finetune_kp_x
        self.finetune_kp_y = finetune_kp_y
        self.finetune_kp_theta = finetune_kp_theta
        self.finetune_kd_x = finetune_kd_x
        self.finetune_kd_y = finetune_kd_y
        self.finetune_kd_theta = finetune_kd_theta
        self.finetune_d_alpha = finetune_d_alpha
        self.finetune_max_nav_speed = finetune_max_nav_speed
        self.finetune_tol = finetune_tol
        self.finetune_tol_theta = finetune_tol_theta
        self.finetune_converge_frames = finetune_converge_frames

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
        # VLN 导航指令 (NAVIGATING 阶段用, FineTuning 起改用内置 PID, 不再读它)
        self.nav_cmd_subscriber = ROSMsgSubscriber(STRETCHER_NAV_CMD_TOPIC)
        # 机器人状态订阅 (50Hz): 用于读取实测 waist_pitch 做 IK 目标朝向补偿
        self.state_subscriber = ROSMsgSubscriber(STATE_TOPIC_NAME)
        # 两个独立的 FoundationPose++ handle pose topic (各自一物体一 topic, 不同步).
        # 真实 topic 名 = f"{prefix}/{object_id}", 多机时通过 --pose-topic-namespace 覆盖 prefix.
        self.left_pose_subscriber = ROSMsgSubscriber(f"{pose_topic_prefix}/{STRETCHER_LEFT_HANDLE_ID}")
        self.right_pose_subscriber = ROSMsgSubscriber(f"{pose_topic_prefix}/{STRETCHER_RIGHT_HANDLE_ID}")
        # GRABBING settle 期间累积最近若干帧 handle position, 锁定时取均值平滑 pose 抖动.
        # maxlen 在 GrabbingPhase.enter() 里按 grab_lock_window 重新设置.
        self._left_handle_buffer: deque = deque(maxlen=grab_lock_window)
        self._right_handle_buffer: deque = deque(maxlen=grab_lock_window)

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
            TaskPhase.FINE_TUNING: FineTuningPhase(
                self.context,
                self,
                target_handle_x=self.finetune_target_handle_x,
                kp_x=self.finetune_kp_x,
                kp_y=self.finetune_kp_y,
                kp_theta=self.finetune_kp_theta,
                kd_x=self.finetune_kd_x,
                kd_y=self.finetune_kd_y,
                kd_theta=self.finetune_kd_theta,
                d_alpha=self.finetune_d_alpha,
                max_nav_speed=self.finetune_max_nav_speed,
                tol=self.finetune_tol,
                tol_theta=self.finetune_tol_theta,
                converge_frames=self.finetune_converge_frames,
            ),
            TaskPhase.APPROACHING: ApproachingPhase(self.context, self, approach_duration),
            TaskPhase.GRABBING: GrabbingPhase(
                self.context,
                self,
                duration=grab_duration,
                move_duration=move_duration,
                grab_script=grab_script,
                lock_settle_time=self.grab_lock_settle_time,
                lock_window=self.grab_lock_window,
            ),
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
        """从两个独立 FoundationPose++ topic 各自读取最新 handle pose.

        左右独立刷新: 某侧这一 tick 没有新消息 (get_msg 返回 None) 就保持上次的值,
        不会因一侧没更新而卡住另一侧 (担架静止, 几毫秒错位可忽略).
        新消息进来时同时 push 进锁定 buffer (GRABBING settle 期间用).

        handle 锁定后 (context.handle_locked=True) 直接 return —— GRABBING settle
        结束已用均值锁定 handle, STANDING_UP 期间就地改 position.z 不受此限制
        (本函数锁的是 "不从外部 pose 重新读取", 不是 "不许改 position").
        """
        if self.context.handle_locked:
            return
        for side in ("left", "right"):
            sub = getattr(self, f"{side}_pose_subscriber")
            buf = getattr(self, f"_{side}_handle_buffer")
            msg = sub.get_msg()
            if msg is None:
                continue
            handle = StretcherHandle.from_msg(msg)
            setattr(self.context, f"{side}_handle", handle)
            buf.append(handle.position)

    def _lock_handles(self):
        """对最近若干帧 handle position 取均值, 锁定到 context 并禁止后续刷新.

        GRABBING settle 结束时调用: 此时机器人已停下, 对 settle 期间累积的
        最近 N 帧取均值, 平滑掉 pose 估计的高频抖动, 之后整段 GRABBING/STANDING_UP
        都用这一组锁定值求解 IK. buffer 空时打印警告并放弃锁定 (后续仍可 fallback).
        """
        if len(self._left_handle_buffer) == 0 or len(self._right_handle_buffer) == 0:
            print("WARNING: cannot lock handles, lock buffer empty (FP++ not publishing?)")
            return
        self.context.left_handle = StretcherHandle(
            position=np.mean(self._left_handle_buffer, axis=0)
        )
        self.context.right_handle = StretcherHandle(
            position=np.mean(self._right_handle_buffer, axis=0)
        )
        self.context.handle_locked = True
        print(
            f"[LOCK] handles locked — "
            f"L={self.context.left_handle.position} "
            f"R={self.context.right_handle.position}"
        )

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
    """Parse command line arguments.

    参数按阶段 (stage) 分组、按阶段顺序排列 (Global → NAVIGATING → FINE_TUNING
    → APPROACHING → GRABBING → STANDING_UP). 每个参数 help 里也标了所属阶段.
    """
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

    # ==================== Global / Runtime (跨阶段, 控制循环本身) ====================
    parser.add_argument(
        "--start-phase",
        type=str,
        default="idle",
        help="[Global] Phase to start from (default: idle)",
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="[Global] Automatically start the task (transition from idle to navigating)",
    )
    parser.add_argument(
        "--single-phase",
        action="store_true",
        help="[Global] Run only the specified --start-phase, stop when it completes instead of transitioning",
    )
    parser.add_argument(
        "--freq",
        type=float,
        default=50.0,
        help="[Global] Publishing frequency in Hz (default: 50.0)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="[Global] Maximum task duration in seconds (None for indefinite)",
    )
    parser.add_argument(
        "--pose-topic-namespace",
        type=str,
        default=DEFAULT_STRETCHER_POSE_TOPIC_PREFIX,
        help="[Global] FoundationPose++ handle pose topic 前缀 (多机区分用). "
             "真实 topic = f'{prefix}/{left,right}_handle' (default: FoundationPose/pose)",
    )

    # ==================== NAVIGATING (Phase 1: VLN 远场导航) ====================
    # NAVIGATING 只订阅 StretcherTask/nav_cmd (VLN 发), 无本阶段专有参数.

    # ==================== FINE_TUNING (Phase 2: handle 位姿 PD 微调) ====================
    parser.add_argument(
        "--finetune-target-handle-x",
        type=float,
        default=0.45,
        help="[FineTuning] 目标抓取窗 x (pelvis 系, 米), 两 handle 均值收敛到此. "
             "默认 0.45, 实机调. (y 固定收敛到 0, z 不在本阶段管)",
    )
    parser.add_argument(
        "--finetune-kp-x",
        type=float,
        default=1.0,
        help="[FineTuning] x 方向 P 增益 (default: 1.0, 实机调)",
    )
    parser.add_argument(
        "--finetune-kp-y",
        type=float,
        default=1.0,
        help="[FineTuning] y 方向 P 增益 (default: 1.0, 实机调)",
    )
    parser.add_argument(
        "--finetune-kp-theta",
        type=float,
        default=1.0,
        help="[FineTuning] 偏航 θ 方向 P 增益 (default: 1.0, 实机调). "
             "符号方向实机验证: 故意歪担架看 PID 输出对不对, 不对翻此参数符号",
    )
    parser.add_argument(
        "--finetune-kd-x",
        type=float,
        default=0.0,
        help="[FineTuning] x 方向 D 增益 (default: 0.0, 实机调). "
             "D 项对 EWMA 低通后的误差求差分, 抑制 pose 抖动放大",
    )
    parser.add_argument(
        "--finetune-kd-y",
        type=float,
        default=0.0,
        help="[FineTuning] y 方向 D 增益 (default: 0.0, 实机调)",
    )
    parser.add_argument(
        "--finetune-kd-theta",
        type=float,
        default=0.0,
        help="[FineTuning] 偏航 θ 方向 D 增益 (default: 0.0, 实机调)",
    )
    parser.add_argument(
        "--finetune-d-alpha",
        type=float,
        default=0.5,
        help="[FineTuning] D 项误差 EWMA 低通系数 (0~1, default: 0.5). "
             "越小越平滑但越滞后; 仅当对应 D 增益>0 时生效",
    )
    parser.add_argument(
        "--finetune-max-nav-speed",
        type=float,
        nargs=3,
        default=[0.1, 0.1, 0.1],
        metavar=("VX", "VY", "VTHETA"),
        help="[FineTuning] 输出 nav_cmd [vx vy vtheta] 各维限幅 (m/s, rad/s, default: 0.1 0.1 0.1)",
    )
    parser.add_argument(
        "--finetune-tol",
        type=float,
        default=0.03,
        help="[FineTuning] x/y 收敛阈值 (米, default: 0.03)",
    )
    parser.add_argument(
        "--finetune-tol-theta",
        type=float,
        default=0.05,
        help="[FineTuning] θ 收敛阈值 (用左右 handle x 差, 米, default: 0.05, 约 3°)",
    )
    parser.add_argument(
        "--finetune-converge-frames",
        type=int,
        default=10,
        help="[FineTuning] 误差连续低于阈值多少帧才退出 (default: 10, @50Hz≈0.2s, 防 pose 抖动假退出)",
    )

    # ==================== APPROACHING (Phase 3a: 下蹲 + 弯腰) ====================
    parser.add_argument(
        "--approach-duration",
        type=float,
        default=2.0,
        help="[Approaching] 下蹲+弯腰插值时长 (秒, default: 2.0)",
    )
    parser.add_argument(
        "--target-height",
        type=float,
        default=0.34,
        help="[Approaching] 弯腰下蹲目标 base_height (米, default: 0.34), GRABBING/STANDING_UP 维持此值",
    )
    parser.add_argument(
        "--target-torso-rpy",
        type=float,
        nargs=3,
        default=[0.0, 60.0, 0.0],
        metavar=("ROLL", "PITCH", "YAW"),
        help="[Approaching] 目标 torso [roll, pitch, yaw] (度, default: 0 60 0)",
    )

    # ==================== GRABBING (Phase 3b: 锁定 handle + IK 抓取) ====================
    parser.add_argument(
        "--grab-lock-settle-time",
        type=float,
        default=1.5,
        help="[Grabbing] 进入 GRABBING 后等待 pose 稳定的 settle 时长 (秒, default: 1.5). "
             "期间订阅照常但发 command, 到点对最近 N 帧取均值锁定 handle",
    )
    parser.add_argument(
        "--grab-lock-window",
        type=int,
        default=10,
        help="[Grabbing] 锁定时取均值的最近帧数 (default: 10, @50Hz≈0.2s)",
    )
    parser.add_argument(
        "--grab-duration",
        type=float,
        default=3.0,
        help="[Grabbing] settle 结束后 IK 抓取时长 (秒, default: 3.0, 不含 settle)",
    )
    parser.add_argument(
        "--move-duration",
        type=float,
        default=0.5,
        help="[Grabbing] IK 目标运动持续时间 (秒, default: 0.5), 控制 InterpolationPolicy 平滑过渡",
    )
    parser.add_argument(
        "--grab-script",
        type=str,
        default=None,
        help="[Grabbing] 抓取脚本路径, GRABBING 50% 进度时 subprocess 启动",
    )
    # handle 缺失时的 IK fallback position (左右独立)
    parser.add_argument(
        "--default-left-wrist-position",
        type=float,
        nargs=3,
        default=[0.25, 0.3, 0.1],
        metavar=("X", "Y", "Z"),
        help="[Grabbing] 左 handle 缺失时的 fallback position [x y z] (pelvis 系, 米, default: 0.25 0.3 0.1). "
             "朝向始终走 waist_pitch 补偿, 此处只给 position",
    )
    parser.add_argument(
        "--default-right-wrist-position",
        type=float,
        nargs=3,
        default=[0.25, -0.3, 0.1],
        metavar=("X", "Y", "Z"),
        help="[Grabbing] 右 handle 缺失时的 fallback position [x y z] (pelvis 系, 米, default: 0.25 -0.3 0.1). "
             "朝向始终走 waist_pitch 补偿, 此处只给 position",
    )

    # ==================== STANDING_UP (Phase 4: 抬起担架) ====================
    parser.add_argument(
        "--standup-duration",
        type=float,
        default=3.0,
        help="[StandingUp] 站起反向插值时长 (秒, default: 3.0)",
    )
    parser.add_argument(
        "--standup-handle-z-target",
        type=float,
        default=0.0,
        help="[StandingUp] pelvis 系下手腕 z 的最终目标 (米, 左右共用, default: 0.0). "
             "GRABBING 末尾的 z 沿直线插值到该值, xy 冻结",
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
        pose_topic_prefix=args.pose_topic_namespace,
        grab_lock_settle_time=args.grab_lock_settle_time,
        grab_lock_window=args.grab_lock_window,
        grab_duration=args.grab_duration,
        move_duration=args.move_duration,
        approach_duration=args.approach_duration,
        standup_duration=args.standup_duration,
        standup_handle_z_target=args.standup_handle_z_target,
        publish_frequency=args.freq,
        grab_script=args.grab_script,
        single_phase=args.single_phase,
        # FINE_TUNING PD 参数
        finetune_target_handle_x=args.finetune_target_handle_x,
        finetune_kp_x=args.finetune_kp_x,
        finetune_kp_y=args.finetune_kp_y,
        finetune_kp_theta=args.finetune_kp_theta,
        finetune_kd_x=args.finetune_kd_x,
        finetune_kd_y=args.finetune_kd_y,
        finetune_kd_theta=args.finetune_kd_theta,
        finetune_d_alpha=args.finetune_d_alpha,
        finetune_max_nav_speed=args.finetune_max_nav_speed,
        finetune_tol=args.finetune_tol,
        finetune_tol_theta=args.finetune_tol_theta,
        finetune_converge_frames=args.finetune_converge_frames,
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
