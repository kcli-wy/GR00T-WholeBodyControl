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
- `NAVIGATING`: Phase 1 - VLN navigation to stretcher
- `FINE_TUNING`: Phase 2 - Pose-based position adjustment
- `APPROACHING`: Phase 3a - Lower body and adjust torso
- `GRABBING`: Phase 3b - IK solve and grab handles
- `STANDING_UP`: Phase 4 - Stand up with stretcher
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
| `FoundationPose/pose/<id>` | ByteMultiArray (msgpack) | FoundationPose++ | Pose Bridge |
| `StretcherTask/nav_cmd` | ByteMultiArray (msgpack) | VLN Model | Task Controller |
| `StretcherTask/pose` | ByteMultiArray (msgpack) | Pose Bridge | Task Controller |
| `StretcherTask/status` | ByteMultiArray (msgpack) | Task Controller | External |
| `ControlPolicy/upper_body_pose` | ByteMultiArray (msgpack) | Task Controller | Robot Control Loop |

> 所有 topic 均使用 `ROSMsgPublisher` / `ROSMsgSubscriber` 格式 (ByteMultiArray + msgpack)。

### 启动流程

```bash
# 1. 启动相机发布
python run_realsense_publisher.py --camera head

# 2. 启动 FoundationPose++ (conda activate foundationpose)
python /path/to/FoundationPose-plus-plus/src/run_foundationpose_ros2.py \
    --objects_json test_data/stretcher_handle.json

# 3. 启动位姿桥接 (FoundationPose++ → StretcherTask/pose)
# (待实现)

# 4. 启动任务控制器
python run_stretcher_task.py --start-phase approaching --single-phase
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

### Pose Estimation (`StretcherTask/pose`)
```python
{
    "left_handle": {
        "position": [x, y, z],
        "orientation": [w, x, y, z],  # Quaternion (scalar-first)
    },
    "right_handle": {
        "position": [x, y, z],
        "orientation": [w, x, y, z],
    },
    "ready_to_grab": bool,           # True when positioned correctly
    "navigate_cmd": [vx, vy, wz],    # Optional fine-tuning command
    "timestamp": float,
}
```

### FoundationPose++ 位姿输出 (`FoundationPose/pose/<object_id>`)

每个跟踪物体一个独立 topic，`<object_id>` 来自 `objects.json` 的 `"id"` 字段。
与 `StretcherTask/pose` 不同，这是 FoundationPose++ 原始输出，由 pose bridge 转换为 `StretcherTask/pose` 格式。

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
│                 │     │ (RGB+Depth) │────▶│    Pose     │
└─────────────────┘     └─────────────┘     │  Estimation │
                                            └──────┬──────┘
                                                   │
┌─────────────┐     ┌─────────────┐            │
│  VLN Model  │────▶│  nav_cmd    │────▶│                     │
└─────────────┘     └─────────────┘     │                     │
                                        │  Task Controller    │────▶ Robot Control Loop
┌─────────────┐     ┌─────────────┐     │                     │
│    Pose     │────▶│  pose       │────▶│                     │
│  Estimation │     └─────────────┘     └─────────────────────┘
└─────────────┘                                    │
                                                   ▼
                                          ┌─────────────────┐
                                          │  Task Status    │
                                          └─────────────────┘
```

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

### Pose Estimation Integration
- Subscribe to `StretcherTask/pose` topic
- Publish handle poses at 10-50 Hz
- Optionally include `navigate_cmd` for fine-tuning
- Set `ready_to_grab=True` when positioned correctly

### Grab Script
The grab script should:
1. Be a standalone Python script
2. Control the robot's hand actuators
3. Handle the actual grasping motion
4. Can be specified via `--grab-script` argument

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
