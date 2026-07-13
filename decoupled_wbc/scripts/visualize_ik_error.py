#!/usr/bin/env python3
"""手动给定双臂 IK target,求解后用 FK + wrist_offset 复算 displaced-frame 位姿,
与 target 对比 error,并用 meshcat 可视化三组 frame(target / FK wrist / FK displaced)。

target 语义为 displaced-frame 目标(抓握点),即 wrist_offset=(0.13, 0, 0) 偏移点
想去的位置;TeleopRetargetingIK 内部会自动把 displaced target 反推为 wrist target 再求解。
"""
import time

import meshcat_shapes
import numpy as np
from scipy.spatial.transform import Rotation

from decoupled_wbc.control.robot_model.instantiation.g1 import instantiate_g1_robot_model
from decoupled_wbc.control.teleop.solver.hand.instantiation.g1_hand_ik_instantiation import (
    instantiate_g1_hand_ik_solver,
)
from decoupled_wbc.control.teleop.teleop_retargeting_ik import TeleopRetargetingIK

# ---------------------------------------------------------------------------
# 手动给定的双臂 IK target (displaced-frame 目标)
#   - target 是 wrist 前方 wrist_offset=(0.13, 0, 0) 处虚拟 frame 的目标位姿,
#     不是 wrist frame 本身。
#   - position: 米; rpy: 度 (xyz 固定角)。
#   - 以下为占位值,请改成实际 target;可参考脚本运行时打印的默认 wrist 位姿。
# ---------------------------------------------------------------------------
LEFT_TARGET_POS = [+0.421, +0.255, +0.316]
LEFT_TARGET_RPY_DEG = [0.0, 40.0, 0.0]
RIGHT_TARGET_POS = [+0.453, -0.195, +0.281]
RIGHT_TARGET_RPY_DEG = [0.0, 40.0, 0.0]

# wrist_offset: target 是 wrist 沿自身局部 x/y/z 偏移该向量处的虚拟 frame 目标。
# (0.13, 0, 0) = wrist 前方 0.13m 抓握点;设为 (0, 0, 0) 则 target 退化为 wrist frame 目标。
WRIST_OFFSET = (0.13, 0.0, 0.0)

# error 阈值 (仅用于 print pass/fail,不 assert)
POS_TOL = 0.01  # m
ROT_TOL_DEG = 1.0


def pose_to_matrix(position, rpy_deg):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    T[:3, 3] = position
    return T


def add_frame(viewer, name, T, origin_color, axis_length=0.12, origin_radius=0.025):
    """在 meshcat viewer 画一个三色轴 frame,原点 sphere 用 origin_color 着色以区分。"""
    handle = viewer[name]
    meshcat_shapes.frame(
        handle,
        axis_length=axis_length,
        opacity=1.0,
        origin_color=origin_color,
        origin_radius=origin_radius,
    )
    handle.set_transform(T)


def main():
    robot_model = instantiate_g1_robot_model(
        waist_location="lower_body", high_elbow_pose=False
    )
    left_hand_ik, right_hand_ik = instantiate_g1_hand_ik_solver()
    retargeting_ik = TeleopRetargetingIK(
        robot_model=robot_model,
        left_hand_ik_solver=left_hand_ik,
        right_hand_ik_solver=right_hand_ik,
        enable_visualization=True,
        body_active_joint_groups=["upper_body"],
        wrist_offset=WRIST_OFFSET,
    )
    full_robot = retargeting_ik.full_robot
    wrist_offset = retargeting_ik.wrist_offset
    left_wrist = full_robot.supplemental_info.hand_frame_names["left"]
    right_wrist = full_robot.supplemental_info.hand_frame_names["right"]

    # 打印默认 wrist 位姿,作为 target 设值参考
    full_robot.cache_forward_kinematics(full_robot.q_zero)
    print(f"[ref] default {left_wrist}:\n{full_robot.frame_placement(left_wrist).np}")
    print(f"[ref] default {right_wrist}:\n{full_robot.frame_placement(right_wrist).np}")

    # 组装 displaced-frame target
    T_target = {
        left_wrist: pose_to_matrix(LEFT_TARGET_POS, LEFT_TARGET_RPY_DEG),
        right_wrist: pose_to_matrix(RIGHT_TARGET_POS, RIGHT_TARGET_RPY_DEG),
    }
    body_data = {
        left_wrist: T_target[left_wrist],
        right_wrist: T_target[right_wrist],
    }

    # IK (内部 _apply_wrist_offset 把 displaced target 反推为 wrist target 再解)
    q = retargeting_ik.compute_joint_positions(body_data, None, None)

    # FK 复算
    full_robot.cache_forward_kinematics(q, auto_clip=False)
    T_offset = np.eye(4)
    T_offset[:3, 3] = wrist_offset

    robot_viz = retargeting_ik.visualizer
    viewer = robot_viz.viz.viewer
    for side, wrist_link in [("left", left_wrist), ("right", right_wrist)]:
        T_fk_wrist = full_robot.frame_placement(wrist_link).np
        T_fk_displaced = T_fk_wrist @ T_offset
        T_tgt = T_target[wrist_link]

        # position error: xyz 分量 + norm
        dp = T_fk_displaced[:3, 3] - T_tgt[:3, 3]
        pos_norm = float(np.linalg.norm(dp))
        # rotation error: arccos((trace(R_fk @ R_target.T) - 1) / 2)
        R_rel = T_fk_displaced[:3, :3] @ T_tgt[:3, :3].T
        rot_deg = float(
            np.rad2deg(np.arccos(np.clip((np.trace(R_rel) - 1) / 2, -1, 1)))
        )

        pos_pass = pos_norm < POS_TOL
        rot_pass = rot_deg < ROT_TOL_DEG
        print(f"\n[{side}] {wrist_link}")
        print(
            f"  pos error: x={dp[0]:+.5f}  y={dp[1]:+.5f}  z={dp[2]:+.5f}  "
            f"norm={pos_norm:.5f} m  ({'PASS' if pos_pass else 'FAIL'} < {POS_TOL})"
        )
        print(
            f"  rot error: {rot_deg:.4f} deg  "
            f"({'PASS' if rot_pass else 'FAIL'} < {ROT_TOL_DEG})"
        )

        # 可视化: target (绿) / FK displaced (红);
        # wrist 由 RobotVisualizer 自带 (三色轴 + 黑色原点)
        add_frame(viewer, f"target_{side}", T_tgt, origin_color=0x00FF00)
        add_frame(
            viewer, f"fk_displaced_{side}", T_fk_displaced, origin_color=0xFF0000
        )

    print(
        "\n[meshcat] 可视化已打开: target=绿, fk_displaced=红, wrist=自带三色轴。"
        "绿红球重合即 offset 闭环正确。Ctrl+C 退出。"
    )
    try:
        while True:
            robot_viz.visualize(q)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n退出。")


if __name__ == "__main__":
    main()
