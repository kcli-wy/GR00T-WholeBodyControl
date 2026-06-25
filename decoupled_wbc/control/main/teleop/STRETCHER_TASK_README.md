# Stretcher Task Architecture

## Overview

This document describes the architecture for the G1 robot stretcher-grabbing task.

## Components

### 1. Camera Publisher

#### RealSense Publisher (`run_realsense_publisher.py`)
Publishes real camera images from Intel RealSense cameras.

**Features:**
- Auto-calibration intrinsics from RealSense SDK
- Depth aligned to RGB
- Post-processing filters (decimation, spatial, temporal, hole-filling)

**Usage:**
```bash
# Publish both cameras
python run_realsense_publisher.py

# Publish only chest camera
python run_realsense_publisher.py --camera chest

# Publish only head camera
python run_realsense_publisher.py --camera head
```

**Configuration:**
Edit `run_realsense_publisher.py` to set camera serial numbers:
```python
CHEST_CAMERA_SERIAL = "1234567890"  # TODO: Fill in
HEAD_CAMERA_SERIAL = "9876543210"   # TODO: Fill in
```

#### Camera Subscriber Test (`test_camera_sub.py`)
Subscribes to camera topics for testing reception.

**Usage:**
```bash
# Subscribe to all cameras
python test_camera_sub.py --verbose

# Subscribe with display (requires OpenCV)
python test_camera_sub.py --display --verbose

# Save images to directory
python test_camera_sub.py --save-dir ./images --verbose
```

#### Camera Communication Test (`test_camera_publisher.py`)
Tests camera ROS2 communication.

**Usage:**
```bash
# Test all cameras
python test_camera_publisher.py --verbose

# Test specific camera
python test_camera_publisher.py --camera chest --verbose
```

**Camera Configuration:**
| Camera | Location | Purpose | Topic | Content |
|--------|----------|---------|-------|---------|
| Chest | Robot chest | Navigation/VLN | `camera/chest` | RGB only |
| Head | Robot head | Pose estimation | `camera/head` | RGB + Depth |

### 2. Stretcher Task Controller (`run_stretcher_task.py`)

State machine-based controller that orchestrates the entire task.

**Phases:**
- `IDLE`: Waiting for start command
- `NAVIGATING`: Phase 1 - VLN 远场导航到担架附近 (订阅 `StretcherTask/nav_cmd`)
- `FINE_TUNING`: Phase 2 - 基于 handle 位姿的近场 PD 微调 (订阅两个 FoundationPose++ topic, 自算 `nav_cmd`, 收敛后转下一阶段)
- `APPROACHING`: Phase 3a - 下蹲 + 弯腰 (同步插值 `base_height` / `torso_rpy`)
- `GRABBING`: Phase 3b - settle 等位姿稳定 → 对最近 N 帧取均值锁定 handle → IK 求解抓取; 50% 进度非阻塞启动 `scripts/grab.sh` (手部抓取)
- `STANDING_UP`: Phase 4 - 抬起担架 (反向插值, 双手保持抓握, handle z 线性插值上升;
  orientation 仍用 `pelvis_pitch + waist_pitch` 补偿保持手腕垂直向下)
- `COMPLETED`: Task finished

**Usage:**
```bash
# Run full task
python run_stretcher_task.py --interface sim --auto-start

# Start from specific phase (for testing)
python run_stretcher_task.py --interface sim --start-phase grabbing

# With custom parameters
python run_stretcher_task.py --interface sim \
    --target-height 0.34 \
    --target-torso-rpy 0 60 0 \
    --grab-script scripts/grab.sh
```

### 2. Communication Test (`test_stretcher_comm.py`)

Mock publishers for testing ROS2 communication.

**Usage:**
```bash
# Run all tests
python test_stretcher_comm.py

# Test specific component
python test_stretcher_comm.py --test nav
python test_stretcher_comm.py --test pose
python test_stretcher_comm.py --test full
```

## ROS2 Topics

### Camera Topics (Constants in `constants.py`)
| Constant | Topic | Description |
|----------|-------|-------------|
| `CAMERA_CHEST_TOPIC` | `camera/chest` | Chest camera RGB |
| `CAMERA_HEAD_TOPIC` | `camera/head` | Head camera RGB + Depth |

### All Topics
| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `camera/chest` | ByteMultiArray (msgpack) | Camera Publisher | VLN Model |
| `camera/head` | ByteMultiArray (msgpack) | Camera Publisher | FoundationPose++ |
| `FoundationPose/pose/left_handle` | ByteMultiArray (msgpack) | FoundationPose++ | Task Controller |
| `FoundationPose/pose/right_handle` | ByteMultiArray (msgpack) | FoundationPose++ | Task Controller |
| `StretcherTask/nav_cmd` | ByteMultiArray (msgpack) | VLN Model | Task Controller (NAVIGATING) |
| `G1Env/env_state_act` | ByteMultiArray (msgpack) | Robot Control Loop | Task Controller (读 pelvis_pitch + waist_pitch) |
| `StretcherTask/status` | ByteMultiArray (msgpack) | Task Controller | External |
| `ControlPolicy/upper_body_pose` | ByteMultiArray (msgpack) | Task Controller | Robot Control Loop |

> 所有 topic 均使用 `ROSMsgPublisher` / `ROSMsgSubscriber` 格式 (ByteMultiArray + msgpack)。
>
> **handle pose topic 前缀可配**: 默认 `FoundationPose/pose`, 多机区分时用
> `--pose-topic-namespace <prefix>` 覆盖 (真实 topic = `f"{prefix}/{left,right}_handle"`)。
> 不再有 `StretcherTask/pose` 聚合 topic, controller 直接订阅两个独立 FP++ topic。

### 启动流程

```bash
# 1. 启动相机发布
python run_realsense_publisher.py --camera head

# 2. 启动 FoundationPose++ (conda activate foundationpose)
#    发两个独立 topic: FoundationPose/pose/{left,right}_handle
python /path/to/FoundationPose-plus-plus/src/obj_pose_track_ros2.py \
    --objects_json test_data/stretcher_handle.json

# 3. 启动机器人控制循环 (提供 G1Env/env_state_act 给 controller 读 pelvis_pitch + waist_pitch)
python run_g1_control_loop.py

# 4. 启动任务控制器
python run_stretcher_task.py --start-phase approaching --single-phase
# 多机时: --pose-topic-namespace robot1/FoundationPose/pose
```

### 调试命令

```bash
# 查看 topic 列表
ros2 topic list | grep -E "camera|FoundationPose|StretcherTask|ControlPolicy"

# 查看相机消息 (不显示数组)
ros2 topic echo camera/head --no-arr

# 查看 FoundationPose 位姿
ros2 topic echo FoundationPose/pose/right_handle --no-arr

# 查看任务状态
ros2 topic echo StretcherTask/status

# 查看发布频率
ros2 topic hz camera/head
ros2 topic hz FoundationPose/pose/right_handle
```

## Message Formats

### Camera Chest Image (`camera/chest`)
```python
{
    "rgb": np.ndarray,              # RGB image (H, W, 3) uint8
    "camera_info": {
        "width": int,
        "height": int,
        "fx": float,                # Focal length x
        "fy": float,                # Focal length y
        "cx": float,                # Principal point x
        "cy": float,                # Principal point y
        "distortion": [k1, k2, p1, p2, k3],
    },
    "timestamp": float,
    "frame_id": str,                # "chest_camera"
}
```

### Camera Head Image (`camera/head`)
```python
{
    "rgb": np.ndarray,              # RGB image (H, W, 3) uint8
    "depth": np.ndarray,            # Depth image (H, W) uint16, RealSense raw values (multiply by depth_scale to get meters), aligned to RGB
    "camera_info": {
        "width": int,
        "height": int,
        "fx": float,                # Focal length x
        "fy": float,                # Focal length y
        "cx": float,                # Principal point x
        "cy": float,                # Principal point y
        "distortion": [k1, k2, p1, p2, k3],
        "depth_scale": float,       # Depth scale (meters/raw_unit), e.g. 0.001; depth_m = depth_raw * depth_scale
    },
    "timestamp": float,
    "frame_id": str,                # "head_camera"
}
```

### Navigation Command (`StretcherTask/nav_cmd`)
```python
{
    "navigate_cmd": [vx, vy, wz],  # Navigation velocities
    "arrived": bool,                # True when arrived at destination
    "timestamp": float,
}
```

### FoundationPose++ 位姿输出 (`FoundationPose/pose/<object_id>`)

每个跟踪物体一个独立 topic，`<object_id>` 来自 `objects.json` 的 `"id"` 字段
(`left_handle` / `right_handle`)。**controller 直接订阅这两个独立 topic**,
不再有 pose bridge 聚合。

```python
# Topic: FoundationPose/pose/right_handle
# Topic: FoundationPose/pose/left_handle
# 消息类型: std_msgs/msg/ByteMultiArray (msgpack 序列化)
{
    "object_id": str,                    # e.g. "right_handle"
    "object_name": str,                  # 中文名称
    "frame_id": "head_camera",
    "stamp": int,                        # ROS2 时间戳 (纳秒)
    "pose_camera_matrix": [[float]*4]*4, # 4x4 齐次变换矩阵 (相机坐标系)
    "pose_robot_matrix": [[float]*4]*4,  # 4x4 齐次变换矩阵 (机器人坐标系)
    "pose_6d": [x, y, z, rx, ry, rz],   # 位置(米) + 欧拉角(弧度)
    "position": [x, y, z],              # 相机坐标系下位置 (米)
    "orientation_euler_rad": [rx, ry, rz],  # 欧拉角 (弧度)
}
```

> **controller 只读 `pose_robot_matrix`**: `StretcherHandle.from_msg()` 取该矩阵的
> 平移列 `[:3, 3]` 作为 handle position。
> `pose_camera_matrix` / `position` / `pose_6d` / `orientation_euler_rad` 均不读 ——
> 手腕目标朝向不走 FP++ 估计, 而是固定世界系 `R_y(90°)` + 实测
> `(pelvis_pitch + waist_pitch)` 相加补偿 (见下方 "IK 目标朝向补偿" 一节,
> 及 `run_stretcher_task.py:_compute_wrist_orientation`)。
>
> **position 不补偿**: `camera_to_robot` 是**静态全零位标定** (机器人所有关节角=0 时标定),
> 故 `pose_robot_matrix` 的 position 永远落在"全零位 pelvis 系" (站立、水平、waist=0),
> 不随蹲下/waist 改变。而 IK 模型 `set_floating_base=False` 把 pelvis 钉在原点、reduced
> model 把 waist/legs 锁死在 q0 (全零位) → **IK pelvis 系 = 全零位 pelvis 系**, 两系相同,
> position 直接透传即可, 无需任何补偿。

### Task Status (`StretcherTask/status`)
```python
{
    "phase": str,        # Current phase name
    "status": str,       # Human-readable status
    "progress": float,   # 0.0 to 1.0
    "timestamp": float,
}
```

## Data Flow

```
┌─────────────────┐     ┌─────────────┐     ┌─────────────┐
│ Camera Publisher │────▶│ camera/chest│────▶│  VLN Model  │
│  (run_camera)   │     └─────────────┘     └──────┬──────┘
│                 │     ┌─────────────┐            │
│                 │────▶│ camera/head │     ┌──────▼──────┐
│                 │     │ (RGB+Depth) │────▶│FoundationPose│
└─────────────────┘     └─────────────┘     │     ++        │
                                            └──────┬──────┘
                                                   │ pose_robot_matrix
                                                   ▼
                       ┌──────────────────────┐ left/right_handle
                       │   Task Controller    │◀─────┐
┌─────────────┐ nav_cmd│                      │      │
│  VLN Model  │───────▶│  (FineTuning: PD     │      │ G1Env/env_state_act
└─────────────┘        │   自算 nav_cmd)       │────▶ Robot Control Loop
                       │  GRABBING: lock+IK   │◀─────┘ (pelvis_pitch + waist_pitch 补偿)
                       └──────────┬───────────┘
                                  │
                                  ▼
                         ┌─────────────────┐
                         │  Task Status    │
                         └─────────────────┘
```

> 注: 不再有 pose bridge。FoundationPose++ 直接发两个独立 topic 给 controller;
> controller 还订阅 `G1Env/env_state_act` 读实测 `pelvis_pitch` (floating_base_pose 四元数,
> 腿关节造成) 与 `waist_pitch` (q[waist_pitch_idx], torso 相对 pelvis), 两者相加用于 IK
> 目标朝向 (世界系 R_y(90°)) 补偿到 pelvis 系 (见下方 "IK 目标朝向补偿" 一节)。

## RealSense Camera Configuration

### Default Settings
| Parameter | Value |
|-----------|-------|
| Resolution | 640x480 |
| FPS | 30 Hz |
| Color Format | RGB8 |
| Depth Format | Z16 (raw values, multiply by depth_scale ≈ 0.001 for meters) |
| Depth Alignment | Aligned to RGB |

### Post-Processing Filters
| Filter | Parameters | Default |
|--------|------------|---------|
| Decimation | magnitude | 2 |
| Spatial | alpha, delta, iterations | 0.5, 20, 1 |
| Temporal | alpha, delta, persistence | 0.4, 20, 3 |
| Hole-filling | mode | 1 (left) |

### Intrinsics
Auto-calibrated from RealSense SDK:
```python
{
    "width": 640,
    "height": 480,
    "fx": float,  # Focal length x
    "fy": float,  # Focal length y
    "cx": float,  # Principal point x
    "cy": float,  # Principal point y
    "distortion": [k1, k2, p1, p2, k3],
    "depth_scale": float,  # Depth scale (meters/raw_unit), e.g. 0.001
}
```

## Implementation Notes

### VLN Model Integration
- Subscribe to `StretcherTask/nav_cmd` topic
- Publish navigation commands at 10-50 Hz
- Set `arrived=True` when reaching the stretcher
- 仅 `NAVIGATING` 阶段使用; `FINE_TUNING` 起改用内置 PD 自算 `nav_cmd`

### Pose Estimation Integration (FoundationPose++)
- Controller 直接订阅两个独立 topic `FoundationPose/pose/{left,right}_handle`
- 每个 handle 一条消息, controller 读 `pose_robot_matrix[:3,3]` 作 position
- 朝向不读 FP++ 估计 —— 固定世界系 `R_y(90°)` + `G1Env/env_state_act` 的实测
  `(pelvis_pitch + waist_pitch)` 相加补偿到 pelvis 系 (见下方 "IK 目标朝向补偿")
- 多机区分: `--pose-topic-namespace <prefix>` 覆盖 topic 前缀
- `GRABBING` 阶段对最近 N 帧取均值锁定 handle, 之后全程不刷新 (担架静止 + pose 抖动)

### IK 目标朝向补偿 (`_compute_wrist_orientation`)

手腕目标朝向在世界系下固定为 `R_y(90°)` (手腕垂直向下), 但 IK 在 pelvis 系求解
(robot_model `set_floating_base=False`, pelvis 是固定根, "Pink 的 world" = pelvis 系)。
弯腰时两个量改变末端朝向, 需**相加**补偿:

| pitch | 来源 | 物理含义 |
|-------|------|----------|
| `pelvis_pitch` | `obs["floating_base_pose"][3:7]` 四元数 (`[w,x,y,z]`) 解 Y 轴欧拉角 | pelvis 在世界系的倾斜, 由**腿关节**造成 (不在 IK 模型内) |
| `waist_pitch` | `obs["q"][waist_pitch_idx]` (waist 关节组 `[yaw, roll, pitch]` 第 3 个) | torso 相对 pelvis 的旋转, 改变末端坐标系 rpy |

补偿公式 (`_compute_wrist_orientation`):

```python
total_pitch = current_pelvis_pitch + current_waist_pitch
R_pelvis_target = R_y(-total_pitch) · R_y(90°)   # 先按负号, 实机验
T[:3, :3] = R_pelvis_target
T[:3, 3] = handle.position                       # position 不补偿 (见下)
```

**注:**

- **符号**: 先按 `R_y(-total_pitch)` (相加取负) 实现, **实机验证**。弯腰时若手腕方向反,
  翻 `_compute_wrist_orientation` 里 `R.from_euler('y', -total_pitch)` 的负号为正 (一行改动)。
- **position 不补偿**: `pose_robot_matrix` 的 `camera_to_robot` 是**静态全零位标定** →
  position 永远在"全零位 pelvis 系"; IK 模型 `set_floating_base=False` + reduced model 锁
  waist/legs 在 q0 → IK pelvis 系 = 全零位 pelvis 系。两系相同, position 直接透传。
- **左右手朝向相同**: 补偿矩阵不依赖 handle, 只依赖 `total_pitch`, 左右手共用同一矩阵
  (IK 靠 frame name 区分左右臂)。

### 坐标系梳理 (5 个系)

| # | 坐标系 | 说明 |
|---|--------|------|
| 1 | 世界系 (MuJoCo base) | 机器人运动的绝对参考; `floating_base_pose` 表达在此系 |
| 2 | 真实 pelvis 系 | 真实机器人 pelvis 刚体, 随腿关节相对世界系倾斜 (`pelvis_pitch`) |
| 3 | IK pelvis 系 | IK 模型把 pelvis 钉在原点 (`set_floating_base=False`); **= 全零位 pelvis 系**; IK 在此系求解 |
| 4 | 全零位 pelvis 系 (静态标定) | `camera_to_robot` 全零位标定的目标系; `pose_robot_matrix` 的 position 落在此系; **= 系 (3)**, 故 position 不补 |
| 5 | torso 系 | torso 刚体, 随 waist 相对 pelvis 旋转 (`waist_pitch`); 相机的父系 |

**关键关系**:
- **站立时**:(1)≈(2)≈(3)=(4), torso(5)=pelvis(waist=0) → 全部重合, 验证 IK offset 通过。
- **蹲下时**:(2) 相对 (1) 倾斜 `pelvis_pitch`, (5) 相对 (2) 转 `waist_pitch`; 但 (3)=(4)
  始终相同 (IK 模型 pelvis 钉原点 + waist 锁 q0)。

**为何 orientation 补两层、position 不补**:
- **orientation** 目标对齐**世界系 (1)** (手腕垂直向下), 蹲下时真实末端朝向 = (1) 下表达,
  需经 (2) `pelvis_pitch` + (5) `waist_pitch` 两层旋转换到 IK pelvis 系 (3) → 补两层。
- **position** 来自 `pose_robot_matrix` 的 (4) 系, 而 (4)=(3), 直接用, 不补。

### IK 模型与求解

- robot_model: `instantiate_g1_robot_model(waist_location="lower_body", high_elbow_pose=False)`,
  `set_floating_base=False` (pelvis 固定根)。
- `TeleopRetargetingIK(body_active_joint_groups=["upper_body"])` → `ReducedRobotModel` 把
  waist/legs 锁死在 q0 (全零位), **IK 只解双臂 14 关节 + 双手**。
- 目标 frame: `left_wrist_yaw_link` / `right_wrist_yaw_link` (`hand_frame_names`)。
- **wrist offset**: `TeleopRetargetingIK` 默认 `wrist_x_offset=0.13`, IK 内部把"延伸 frame
  目标"换算回 wrist frame 目标 (`_apply_wrist_offset`), 即 IK 实际让"手腕末端往外 0.13m
  的点"落到给定的 handle position 上。offset 方向已实机验证。

**debug 输出** (GRABBING/STANDING_UP 调 `_solve_ik_for_handles` 时打印):

```
[IK-target] left  = [+0.450, +0.300, +0.120]  (locked)
[IK-target] right = [+0.450, -0.300, +0.120]  (locked)
[IK-target] pelvis pitch = +35.0° (measured, from floating_base_pose)
[IK-target] waist pitch  = +25.0° (measured, from q[waist_pitch_idx])
[IK-target] total pitch  = +60.0° (sum, used for orientation compensation)
```

`left/right` 行末尾 `(src)` 标来源: `locked` (GRABBING/STANDING_UP 锁定值) /
`live` (FineTuning/settle 实时值) / `default` (handle 缺失 fallback)。

### Grab Script
GRABBING 阶段 50% 进度时, `subprocess.Popen` **非阻塞**启动抓取脚本 (后台运行, 不卡 50Hz 主循环):
- 默认 `scripts/grab.sh` (相对 `run_stretcher_task.py` 所在目录, 用 `__file__` 解析, 不依赖 cwd)。
- **按扩展名选执行器**: `.sh` → `bash <script>`, `.py` → `python <script>`, 其它 → 直接 exec (依赖 shebang)。
- 脚本要求:
  1. 独立可执行 (`.sh` 需 shebang, 或靠上面的 bash 调用)。
  2. 控制手部执行器完成实际抓取动作。
  3. 可用 `--grab-script <path>` 覆盖路径。

## CLI 参数 (`run_stretcher_task.py`)

参数按阶段分组、按阶段顺序排列 (Global → NAVIGATING → FINE_TUNING → APPROACHING → GRABBING → STANDING_UP)。每个参数 `--help` 里也标了所属阶段。实机调的占位默认值见各参数说明。

### Global / Runtime
| 参数 | 默认 | 说明 |
|------|------|------|
| `--start-phase` | `idle` | 起始阶段 |
| `--auto-start` | off | 自动从 idle 进入 navigating |
| `--single-phase` | off | 只跑 `--start-phase`, 完成后停 |
| `--freq` | `50.0` | 发布频率 Hz |
| `--duration` | None | 任务最长时长 (秒) |
| `--pose-topic-namespace` | `FoundationPose/pose` | FP++ handle pose topic 前缀 (多机区分) |

### NAVIGATING
无本阶段专有参数 (订阅 `StretcherTask/nav_cmd`)。

### FINE_TUNING (PD 微调)
| 参数 | 默认 | 说明 |
|------|------|------|
| `--finetune-target-handle-x` | `0.45` | 目标抓取窗 x (pelvis 系, 米, 实机调); y 收敛到 0 |
| `--finetune-kp-x` / `-kp-y` / `-kp-theta` | `1.0` | x/y/θ 的 P 增益 (实机调; θ 符号实机验证) |
| `--finetune-kd-x` / `-kd-y` / `-kd-theta` | `0.0` | x/y/θ 的 D 增益 (实机调; D 项对低通后误差求差分) |
| `--finetune-d-alpha` | `0.5` | D 项 EWMA 低通系数 (0~1, 仅 D>0 时生效) |
| `--finetune-max-nav-speed` | `0.1 0.1 0.1` | 输出 nav_cmd [vx vy vθ] 各维限幅 |
| `--finetune-tol` | `0.03` | x/y 收敛阈值 (米) |
| `--finetune-tol-theta` | `0.05` | θ 收敛阈值 (左右 handle x 差, 米, ~3°) |
| `--finetune-converge-frames` | `10` | 误差连续低于阈值多少帧才退出 (防 pose 抖动假退出) |

### APPROACHING (下蹲 + 弯腰)
| 参数 | 默认 | 说明 |
|------|------|------|
| `--approach-duration` | `2.0` | 下蹲+弯腰插值时长 (秒) |
| `--target-height` | `0.34` | 弯腰目标 base_height (米) |
| `--target-torso-rpy` | `0 60 0` | 目标 torso RPY (度) |

### GRABBING (锁定 handle + IK 抓取)
| 参数 | 默认 | 说明 |
|------|------|------|
| `--grab-lock-settle-time` | `1.5` | settle 等位姿稳定时长 (秒, 期间订阅照常不发 command) |
| `--grab-lock-window` | `10` | 锁定时取均值的最近帧数 |
| `--grab-duration` | `3.0` | settle 后 IK 抓取时长 (秒, 不含 settle) |
| `--move-duration` | `0.5` | IK 目标运动持续时间 (InterpolationPolicy 平滑过渡) |
| `--grab-script` | `scripts/grab.sh` | 抓取脚本路径, 50% 进度时 subprocess 启动 (默认 `scripts/grab.sh`, 相对本脚本所在目录; `.sh` 用 bash 执行, `.py` 用 python 执行) |
| `--default-left-wrist-position` | `0.25 0.3 0.1` | 左 handle 缺失 fallback position (pelvis 系, 米) |
| `--default-right-wrist-position` | `0.25 -0.3 0.1` | 右 handle 缺失 fallback position |

### STANDING_UP (抬起担架)
| 参数 | 默认 | 说明 |
|------|------|------|
| `--standup-duration` | `3.0` | 站起反向插值时长 (秒) |
| `--standup-handle-z-target` | `0.0` | pelvis 系下手腕 z 最终目标 (米, 左右共用); xy 冻结 |

## File Structure

```
control/main/teleop/
├── run_stretcher_task.py        # Task controller (state machine)
├── run_realsense_publisher.py   # RealSense camera publisher
├── test_stretcher_comm.py       # Test VLN/Pose communication with mock publishers
├── test_camera_publisher.py     # Test camera topic communication
├── test_camera_sub.py           # Camera subscriber for testing reception
├── scripts/
│   └── grab.sh                  # 手部抓取脚本 (GRABBING 50% 时 subprocess 启动)
└── STRETCHER_TASK_README.md     # This file

control/main/
└── constants.py                 # Topic constants (CAMERA_CHEST_TOPIC, CAMERA_HEAD_TOPIC, etc.)
```

### Test Files说明

| 文件 | 用途 |
|------|------|
| `test_stretcher_comm.py` | 模拟 VLN 和位姿估计模型，测试与任务控制器的通信 |
| `test_camera_publisher.py` | 测试相机 topic 通信是否正常 |
| `test_camera_sub.py` | 订阅相机 topic，验证图像接收，支持显示/保存 |

## Quick Start

### 1. Test with Real Cameras
```bash
# Set serial numbers in run_realsense_publisher.py first

# Terminal 1: Start RealSense publisher
python run_realsense_publisher.py

# Terminal 2: Verify reception
python test_camera_sub.py --verbose --display
```

### 2. Test Full Task
```bash
# Terminal 1: Start camera publisher
python run_realsense_publisher.py

# Terminal 2: Start task controller (from grabbing phase)
python run_stretcher_task.py --start-phase grabbing

# Terminal 3: Monitor status
python test_stretcher_comm.py --test full
```

## Testing

1. **Unit Test**: Test individual phase transitions
2. **Integration Test**: Test ROS2 communication with mock publishers
3. **Simulation Test**: Run full task in MuJoCo simulator
4. **Hardware Test**: Run on real robot (with caution)

## Safety Considerations

- Emergency stop: Press `Ctrl+C` at any time
- Joint safety monitor: Automatically limits dangerous velocities
- Timeout safety: Returns to safe state if communication lost
- Phase transitions: Each phase validates prerequisites before proceeding
