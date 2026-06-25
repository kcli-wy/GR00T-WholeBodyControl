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
- `GRABBING`: Phase 3b - settle 等位姿稳定 → 对最近 N 帧取均值锁定 handle → IK 求解抓取
- `STANDING_UP`: Phase 4 - 抬起担架 (反向插值, 双手保持抓握, handle z 线性插值上升)
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
    --grab-script grab_stretcher.py
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
| `G1Env/env_state_act` | ByteMultiArray (msgpack) | Robot Control Loop | Task Controller (读 waist_pitch) |
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

# 3. 启动机器人控制循环 (提供 G1Env/env_state_act 给 controller 读 waist_pitch)
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
> 平移列 `[:3, 3]` 作为 handle position (机器人坐标系 = IK 求解参考系 pelvis 系,
> 由 `camera_to_robot` 静态变换完成, 已处理)。
> `pose_camera_matrix` / `position` / `pose_6d` / `orientation_euler_rad` 均不读 ——
> 手腕目标朝向不走 FP++ 估计, 而是固定世界系 `R_y(90°)` + 实测 `waist_pitch` 补偿
> (见 `run_stretcher_task.py:_compute_wrist_orientation`)。

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
                       │  GRABBING: lock+IK   │◀─────┘ (waist_pitch 补偿)
                       └──────────┬───────────┘
                                  │
                                  ▼
                         ┌─────────────────┐
                         │  Task Status    │
                         └─────────────────┘
```

> 注: 不再有 pose bridge。FoundationPose++ 直接发两个独立 topic 给 controller;
> controller 还订阅 `G1Env/env_state_act` 读实测 `waist_pitch`, 用于 IK 目标朝向
> (世界系 R_y(90°)) 补偿到 pelvis 系。

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
- 朝向不读 FP++ 估计 —— 固定世界系 `R_y(90°)` + `G1Env/env_state_act` 的实测 `waist_pitch` 补偿到 pelvis 系
- 多机区分: `--pose-topic-namespace <prefix>` 覆盖 topic 前缀
- `GRABBING` 阶段对最近 N 帧取均值锁定 handle, 之后全程不刷新 (担架静止 + pose 抖动)

### Grab Script
The grab script should:
1. Be a standalone Python script
2. Control the robot's hand actuators
3. Handle the actual grasping motion
4. Can be specified via `--grab-script` argument

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
| `--grab-script` | None | 抓取脚本路径, 50% 进度时 subprocess 启动 |
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
