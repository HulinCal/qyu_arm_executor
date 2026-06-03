# QYU Arm Executor

基于 ROS2 和 MoveIt 2 的四轴机械臂仿真与控制项目。

                    结果展示：

![仿真显示](https://github.com/HulinCal/qyu_arm_executor/blob/main/assets/result.gif){target="_blank"}



## 项目结构

```
qyu_arm_executor/
├── qyu_ros2_simple_arm/          # 机械臂核心包
│   ├── config/                   # 控制器和桥接配置
│   ├── launch/                   # 启动文件
│   ├── meshes/                   # 3D模型文件
│   ├── rviz/                     # RViz配置
│   ├── urdf/                     # URDF模型
│   └── worlds/                   # Gazebo世界文件
├── qyu_ros2_simple_arm_moveit_config/  # MoveIt 2配置
│   ├── config/                   # MoveIt配置文件
│   └── launch/                   # MoveIt启动文件
├── pose_control_cpp_pkg/         # 姿态控制包
│   ├── models/                   # YOLO模型
│   ├── src/                      # C++节点源码
│   └── CMakeLists.txt
└── qyu_ros2_simple_arm_py/       # Python工具包
    └── qyu_ros2_simple_arm_py/   # 逆运动学测试脚本
```

## 功能特性

- ✅ 四轴机械臂 Gazebo 仿真
- ✅ MoveIt 2 运动规划
- ✅ 夹爪控制
- ✅ 目标姿态控制
- ✅ 物体检测与定位（YOLOv8）

## 依赖安装

```bash
# 基础依赖
sudo apt install ros-jazzy-controller-manager
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-joint-trajectory-controller
sudo apt install ros-jazzy-rqt-joint-trajectory-controller

# MoveIt 2 依赖
sudo apt install ros-jazzy-moveit
sudo apt install ros-jazzy-moveit-ros-move-group
sudo apt install ros-jazzy-moveit-simple-controller-manager
```

## 构建项目

```bash
cd ~/ros-arm/qyu_arm_executor
colcon build --symlink-install
source install/setup.bash
```

## 使用说明

### 1. 启动机械臂仿真

```bash
ros2 launch qyu_ros2_simple_arm spawn_robot.launch.py
```

### 2. 启动 MoveIt 2

```bash
ros2 launch qyu_ros2_simple_arm_moveit_config demo.launch.py
```

### 3. 启动姿态控制节点

```bash
ros2 run pose_control_cpp_pkg pose_control_node
```

### 4. 启动物体检测节点

```bash
ros2 run pose_control_cpp_pkg object_detect_locate_node
```

### 5. 启动目标点移动节点

```bash
ros2 run pose_control_cpp_pkg move_to_pose_node
```

## 节点说明

### pose_control_node
机械臂姿态控制主节点，实现：
- 夹爪张开/闭合
- 机械臂目标姿态控制
- MoveIt 2 运动规划执行

### object_detect_locate_node
物体检测与定位节点：
- 使用 YOLOv8 进行物体检测
- 发布检测到的物体位置信息

### move_to_pose_node
目标点移动节点：
- 根据检测结果移动到目标位置
- 执行抓取操作

## 话题列表

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 关节轨迹指令 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 关节状态 |
| `/detected_objects` | `geometry_msgs/msg/PoseArray` | 检测到的物体位置 |

## 服务列表

| 服务名 | 服务类型 | 说明 |
|--------|----------|------|
| `/move_to_pose` | `geometry_msgs/srv/PoseStamped` | 移动到指定姿态 |

## 参考资料

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/main/)
- [Gazebo Documentation](https://gazebosim.org/docs)

## 许可证

MIT License
