"""Unit tests for StretcherTaskController._compute_wrist_orientation.

只验数学:目标朝向在世界系下固定为 R_y(+90°),通过 R_y(-(pelvis_pitch+waist_pitch))
补偿到 pelvis 系。position 不做补偿,直接透传 handle.position (待实机确认 IK 参考系后再定).

补偿源是两个相加的 pitch:
- pelvis_pitch: pelvis 世界系倾斜 (floating_base_pose 四元数解出, 腿关节造成);
- waist_pitch: torso 相对 pelvis 的旋转 (obs['q'][waist_pitch_idx], 改变末端 rpy).
两者相加取负进 orientation 补偿. 符号实机验.

测试不拉起 ROS / IK solver:在 import 真模块前 stub 掉重依赖,
用 SimpleNamespace 构造一个最小的"假 controller",
把真函数当成普通函数调用即可。
"""

import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

# --- 在 import run_stretcher_task 之前 stub 掉重依赖 ---
# _compute_wrist_orientation 只用 scipy.Rotation + numpy + self.context.current_pelvis_pitch,
# 不需要 ROS / IK solver / 可视化. 把它们 stub 掉以让测试能在最小依赖下运行.
for _modname in [
    "meshcat_shapes",
    "pink",
    "pink.barriers",
    "pink.solvers",
    "pink.tasks",
    "pink.utils",
    "pink.configuration",
    "pinocchio",
    "decoupled_wbc.control.visualization.humanoid_visualizer",
    "decoupled_wbc.control.teleop.teleop_retargeting_ik",
    "decoupled_wbc.control.teleop.solver.hand.instantiation.g1_hand_ik_instantiation",
    "decoupled_wbc.control.robot_model.instantiation.g1",
    "decoupled_wbc.control.utils.ros_utils",
    "decoupled_wbc.control.utils.telemetry",
]:
    sys.modules.setdefault(_modname, MagicMock())

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

from decoupled_wbc.control.main.teleop.run_stretcher_task import (  # noqa: E402
    StretcherHandle,
    StretcherTaskController,
)


def _make_fake_controller(
    pelvis_pitch_rad: float = 0.0,
    waist_pitch_rad: float = 0.0,
) -> SimpleNamespace:
    """构造一个只够 _compute_wrist_orientation 用的最小 stub controller.

    orientation 补偿用 (pelvis_pitch + waist_pitch) 相加, 两个都默认 0.
    """
    ctx = SimpleNamespace(
        current_pelvis_pitch=pelvis_pitch_rad,
        current_waist_pitch=waist_pitch_rad,
    )
    return SimpleNamespace(context=ctx)


def _call(controller_stub, handle: StretcherHandle, side: str = "left") -> np.ndarray:
    """以 unbound 方式调真函数, self 用 stub 替换."""
    return StretcherTaskController._compute_wrist_orientation(controller_stub, handle, side)


# ---------- case 1: pelvis_pitch = 0, 补偿应该是 identity ----------

def test_no_pelvis_pitch_is_world_rotation_only():
    handle = StretcherHandle(position=np.array([0.4, 0.2, -0.5]))
    ctrl = _make_fake_controller(pelvis_pitch_rad=0.0)

    T = _call(ctrl, handle, "left")

    expected_rot = R.from_euler("y", 90, degrees=True).as_matrix()
    np.testing.assert_allclose(T[:3, :3], expected_rot, atol=1e-12)
    np.testing.assert_array_equal(T[:3, 3], handle.position)
    np.testing.assert_array_equal(T[3], [0, 0, 0, 1])


# ---------- case 2: pelvis_pitch = 60°, 补偿后乘以 pelvis 在世界的旋转应该恢复世界系朝向 ----------

def test_compensation_recovers_world_orientation_at_60deg():
    """物理含义: 机器人弯腰 60° → pelvis 在世界系下绕 Y 转 60°.
    IK 看到的目标 R_pelvis_target 在世界系下表达 = R_pelvis_world · R_pelvis_target,
    应当恢复出 R_world_target = R_y(+90°).
    """
    pelvis_pitch = np.deg2rad(60.0)
    handle = StretcherHandle(position=np.array([0.4, 0.2, -0.5]))
    ctrl = _make_fake_controller(pelvis_pitch_rad=pelvis_pitch)

    T = _call(ctrl, handle, "left")

    # 直接的代数验证: R_pelvis_target 应等于 R_y(-60°) · R_y(90°) = R_y(30°)
    expected_pelvis = R.from_euler("y", -60.0 + 90.0, degrees=True).as_matrix()
    np.testing.assert_allclose(T[:3, :3], expected_pelvis, atol=1e-12)

    # 物理验证: 把 pelvis 系朝向左乘 R_pelvis_world (= R_y(+60°)) 应还原成世界系下的 R_y(+90°)
    R_pelvis_world = R.from_euler("y", 60.0, degrees=True).as_matrix()
    recovered_world = R_pelvis_world @ T[:3, :3]
    expected_world = R.from_euler("y", 90.0, degrees=True).as_matrix()
    np.testing.assert_allclose(recovered_world, expected_world, atol=1e-12)


# ---------- case 3: 端到端 — 弧度输入, position 严格透传 ----------

def test_end_to_end_position_unchanged_and_orientation_matches():
    pelvis_pitch = np.pi / 3  # 60° 用弧度直接给, 验证函数不假设单位
    pos = np.array([0.4, 0.2, -0.5])
    handle = StretcherHandle(position=pos)
    ctrl = _make_fake_controller(pelvis_pitch_rad=pelvis_pitch)

    T = _call(ctrl, handle, "right")

    # position 严格相等 (不是 allclose)
    np.testing.assert_array_equal(T[:3, 3], pos)

    # orientation: R_y(-π/3) · R_y(π/2)
    R_compensate = R.from_euler("y", -pelvis_pitch, degrees=False)
    R_world = R.from_euler("y", np.pi / 2, degrees=False)
    expected = (R_compensate * R_world).as_matrix()
    np.testing.assert_allclose(T[:3, :3], expected, atol=1e-12)


# ---------- 额外验: 左右手共用同一矩阵 (side 不影响结果) ----------

def test_left_and_right_produce_same_matrix():
    handle = StretcherHandle(position=np.array([0.0, 0.0, 0.0]))
    ctrl = _make_fake_controller(pelvis_pitch_rad=0.5)

    T_left = _call(ctrl, handle, "left")
    T_right = _call(ctrl, handle, "right")

    np.testing.assert_array_equal(T_left, T_right)


# ---------- case 4: 双 pitch 相加 — pelvis_pitch + waist_pitch 进补偿 ----------

def test_dual_pitch_compensation_uses_sum():
    """orientation 补偿用 (pelvis_pitch + waist_pitch) 相加.

    pelvis_pitch=40° + waist_pitch=20° → 总 60° → 补偿矩阵应等于 R_y(-60°)·R_y(90°),
    与单 pelvis_pitch=60°(waist=0) 的结果完全一致 (相加等价).
    """
    pelvis_pitch = np.deg2rad(40.0)
    waist_pitch = np.deg2rad(20.0)
    handle = StretcherHandle(position=np.array([0.4, 0.2, -0.5]))
    ctrl = _make_fake_controller(pelvis_pitch_rad=pelvis_pitch, waist_pitch_rad=waist_pitch)

    T = _call(ctrl, handle, "left")

    # 总 60° → R_y(-60°) · R_y(90°) = R_y(30°)
    expected = R.from_euler("y", -60.0 + 90.0, degrees=True).as_matrix()
    np.testing.assert_allclose(T[:3, :3], expected, atol=1e-12)
    # position 透传
    np.testing.assert_array_equal(T[:3, 3], handle.position)

    # 与单 pelvis_pitch=60°(waist=0) 等价
    ctrl_single = _make_fake_controller(pelvis_pitch_rad=np.deg2rad(60.0))
    T_single = _call(ctrl_single, handle, "left")
    np.testing.assert_allclose(T[:3, :3], T_single[:3, :3], atol=1e-12)


# ---------- StretcherHandle.from_msg: pose_robot_matrix → position ----------

def test_from_msg_extracts_position_from_pose_robot_matrix():
    """from_msg 只取 pose_robot_matrix[:3,3] (平移列), 丢弃旋转/其它字段."""
    pos = [0.45, -0.30, 0.12]
    # 造一个 pose_robot_matrix: 非单位旋转 + 平移, 证明旋转被丢弃
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", [0.1, -0.2, 0.3]).as_matrix()
    T[:3, 3] = pos

    msg = {
        "object_id": "left_handle",
        "pose_robot_matrix": T.tolist(),
        "position": [9.9, 9.9, 9.9],          # 相机系 position, 不应被用
        "pose_6d": [9.9, 9.9, 9.9, 0, 0, 0],  # 也不应被用
    }
    handle = StretcherHandle.from_msg(msg)

    np.testing.assert_allclose(handle.position, pos, atol=1e-12)
    # handle 不应再带 orientation 字段
    assert not hasattr(handle, "orientation")


def test_from_msg_handles_negative_position():
    msg = {"pose_robot_matrix": np.eye(4).tolist()}
    msg["pose_robot_matrix"][0][3] = -0.5
    msg["pose_robot_matrix"][1][3] = 0.25
    msg["pose_robot_matrix"][2][3] = -0.8
    handle = StretcherHandle.from_msg(msg)
    np.testing.assert_allclose(handle.position, [-0.5, 0.25, -0.8], atol=1e-12)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
