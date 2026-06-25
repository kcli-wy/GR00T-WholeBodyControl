IMAGE_TOPIC_NAME = "realsense/color/image_raw"
STATE_TOPIC_NAME = "G1Env/env_state_act"
CONTROL_GOAL_TOPIC = "ControlPolicy/upper_body_pose"
ROBOT_CONFIG_TOPIC = "WBCPolicy/robot_config"
KEYBOARD_INPUT_TOPIC = "/keyboard_input"
LOCO_MANIP_TASK_STATUS_TOPIC = "LocoManipPolicy/task_status"
LOCO_NAV_TASK_STATUS_TOPIC = "NavigationPolicy/task_status"
LOWER_BODY_POLICY_STATUS_TOPIC = "ControlPolicy/lower_body_policy_status"
JOINT_SAFETY_STATUS_TOPIC = "ControlPolicy/joint_safety_status"

# Stretcher task topics
STRETCHER_NAV_CMD_TOPIC = "StretcherTask/nav_cmd"  # VLN navigation command
STRETCHER_TASK_STATUS_TOPIC = "StretcherTask/status"  # Task status feedback

# Stretcher handle pose topics (FoundationPose++ 原始输出, 每个物体一个独立 topic)
# 真实 topic 名 = f"{DEFAULT_STRETCHER_POSE_TOPIC_PREFIX}/{object_id}", 多机时通过
# --pose-topic-namespace 覆盖 prefix.
DEFAULT_STRETCHER_POSE_TOPIC_PREFIX = "FoundationPose/pose"
STRETCHER_LEFT_HANDLE_ID = "left_handle"
STRETCHER_RIGHT_HANDLE_ID = "right_handle"

# Camera topics
CAMERA_CHEST_TOPIC = "camera/chest"  # Chest camera for navigation (RGB only)
CAMERA_HEAD_TOPIC = "camera/head"   # Head camera for pose estimation (RGB + Depth)


DEFAULT_NAV_CMD = [0.0, 0.0, 0.0]
DEFAULT_BASE_HEIGHT = 0.74
DEFAULT_WRIST_POSE = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] * 2  # x, y, z + w, x, y, z

DEFAULT_MODEL_SERVER_PORT = 5555  # port used to host the model server
