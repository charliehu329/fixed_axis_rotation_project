
# Franka FR3 固定空间轴旋转控制

## 1. 功能说明

本项目控制 Franka FR3 的 `fr3_link8` 绕一条固定在机器人基座坐标系中的空间轴旋转。

固定轴首先在初始 `fr3_link8` 坐标系中定义：

```yaml
axis_point_link8:
  - 0.0
  - 0.0
  - 0.0

axis_direction_link8:
  - 0.0
  - 0.0
  - 1.0
```

节点第一次收到有效关节状态后，会将轴点和轴方向转换到世界坐标系，并将该空间轴固定。

控制过程中同时计算：

* 末端绕轴旋转产生的线速度；
* 末端绕轴旋转产生的角速度；
* 位置跟踪误差反馈；
* 姿态跟踪误差反馈；
* 七维关节速度指令。

---

## 2. 固定轴参数说明

### 2.1 轴方向

```yaml
axis_direction_link8:
  - 0.0
  - 0.0
  - 1.0
```

表示固定轴方向沿初始 `fr3_link8` 坐标系的正 Z 方向。

节点启动时会自动归一化该向量。

### 2.2 轴上一点

```yaml
axis_point_link8:
  - 0.0
  - 0.0
  - 0.0
```

表示固定轴经过初始 `fr3_link8` 原点。

此时末端位置基本不发生圆周移动，主要表现为绕自身原点旋转。

若希望末端沿圆周移动，轴点必须与 `fr3_link8` 原点存在偏移。例如：

```yaml
axis_point_link8:
  - 0.10
  - 0.0
  - 0.0
```

表示固定轴经过 `fr3_link8` 正 X 方向 0.10 m 处，末端旋转半径约为 0.10 m。

真实运行前必须根据实际机构尺寸确认轴点和轴方向。

---

## 3. 软件环境

当前测试环境：

```text
Ubuntu 24.04
ROS 2 Jazzy
franka_ros2 3.0.0
libfranka 0.17.0
Python 3
Pinocchio
```

主要 ROS 2 依赖：

```text
rclpy
sensor_msgs
std_msgs
launch
launch_ros
fixed_axis_rotation_interfaces
```

---

## 4. 项目结构

```text
franka_ros2_ws/src/
├── fixed_axis_rotation/
│   ├── config/
│   │   └── fixed_axis_rotation.yaml
│   ├── fixed_axis_rotation/
│   │   ├── __init__.py
│   │   ├── fixed_axis_keyboard_node.py
│   │   ├── fixed_axis_rotation_node.py
│   │   ├── robot_kinematics.py
│   │   ├── safety.py
│   │   └── velocity_mapper.py
│   ├── launch/
│   │   └── fixed_axis_rotation.launch.py
│   ├── resource/
│   │   └── fixed_axis_rotation
│   ├── package.xml
│   ├── readme.md
│   └── setup.py
│
└── fixed_axis_rotation_interfaces/
    ├── msg/
    │   └── FixedAxisCommand.msg
    ├── CMakeLists.txt
    └── package.xml
```

---

## 5. 主要 ROS 2 节点

### 5.1 fixed_axis_rotation_node

功能：

1. 订阅当前关节状态；
2. 记录固定空间轴；
3. 生成固定轴旋转期望位姿；
4. 计算六维末端速度；
5. 通过雅可比矩阵计算七维关节速度；
6. 执行速度限制和安全检查；
7. 发布到底层关节速度控制器。

输入：

```text
/franka/joint_states
/fixed_axis_rotation/command
```

输出：

```text
/joint_velocity_example_controller/commands
```

### 5.2 fixed_axis_keyboard_node

功能：

1. 从终端读取键盘；
2. 调整目标角速度；
3. 发送运行、停止、急停和重新记录固定轴指令；
4. 持续发布命令心跳。

输出：

```text
/fixed_axis_rotation/command
```

键盘节点需要在独立交互式终端中启动，不由 launch 文件自动启动。

---

## 6. 构建项目

进入工作空间：

```bash
cd ~/franka_ros2_ws

source /opt/ros/jazzy/setup.bash
```

构建接口包和主控制包：

```bash
colcon build \
  --packages-select \
  fixed_axis_rotation_interfaces \
  fixed_axis_rotation \
  --symlink-install
```

加载工作空间：

```bash
source ~/franka_ros2_ws/install/setup.bash
```

检查可执行节点：

```bash
ros2 pkg executables fixed_axis_rotation
```

正常应显示：

```text
fixed_axis_rotation fixed_axis_keyboard_node
fixed_axis_rotation fixed_axis_rotation_node
```

检查自定义消息：

```bash
ros2 interface show \
  fixed_axis_rotation_interfaces/msg/FixedAxisCommand
```

---

## 7. URDF 检查

参数文件默认使用：

```yaml
urdf_path: "/tmp/fr3.urdf"
```

启动节点前检查文件是否存在：

```bash
ls -l /tmp/fr3.urdf
```

也可以执行：

```bash
test -f /tmp/fr3.urdf && echo "URDF OK" || echo "URDF missing"
```

URDF 中必须包含：

```text
fr3_joint1
fr3_joint2
fr3_joint3
fr3_joint4
fr3_joint5
fr3_joint6
fr3_joint7
fr3_link8
```

如果 URDF 路径不同，需要修改：

```text
config/fixed_axis_rotation.yaml
```

中的：

```yaml
urdf_path:
```

---

## 8. 第一次 dry-run 测试

第一次测试必须保持：

```yaml
dry_run: true
```

也可以在启动命令中强制覆盖：

```bash
ros2 launch fixed_axis_rotation \
  fixed_axis_rotation.launch.py \
  dry_run:=true
```

此时节点会：

* 读取关节状态；
* 计算固定轴；
* 计算末端速度；
* 计算关节速度；
* 执行安全检查；
* 输出控制日志；
* 不向真实机器人发布关节速度。

---

## 9. 启动顺序

### 终端 1：启动 Franka 底层速度控制器

```bash
cd ~/franka_ros2_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch franka_velocity_ctrl \
  fr3_velocity.launch.py \
  robot_ip:=172.16.0.2 \
  mode:=topic \
  use_rviz:=false
```

### 终端 2：启动固定轴主控制节点

```bash
cd ~/franka_ros2_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch fixed_axis_rotation \
  fixed_axis_rotation.launch.py \
  dry_run:=true
```

### 终端 3：启动键盘控制节点

```bash
cd ~/franka_ros2_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run fixed_axis_rotation \
  fixed_axis_keyboard_node \
  --ros-args \
  --params-file \
  ~/franka_ros2_ws/src/fixed_axis_rotation/config/fixed_axis_rotation.yaml
```

### 启动指令详解

#### 固定轴主控制节点启动

全部可传入参数：

```bash
ros2 launch fixed_axis_rotation \
  fixed_axis_rotation.launch.py \
  params_file:=~/franka_ros2_ws/src/fixed_axis_rotation/config/fixed_axis_rotation.yaml \
  dry_run:=true
```

参数说明：

`params_file`

```text
固定轴控制参数文件路径。
```

参数文件中包含 URDF 路径、固定轴定义、Topic、速度限制和安全参数。建议使用绝对路径。

`dry_run`

```text
是否只计算关节速度而不向机器人发布。
```

可选值：

```text
true     只计算和打印日志，不发布关节速度
false    向底层控制器发布关节速度
```

第一次测试必须使用：

```bash
dry_run:=true
```

查看 launch 支持的全部参数：

```bash
ros2 launch fixed_axis_rotation \
  fixed_axis_rotation.launch.py \
  --show-args
```

---

#### 键盘控制节点启动

全部可传入参数：

```bash
ros2 run fixed_axis_rotation \
  fixed_axis_keyboard_node \
  --ros-args \
  --params-file \
  ~/franka_ros2_ws/src/fixed_axis_rotation/config/fixed_axis_rotation.yaml \
  -p command_topic:=/fixed_axis_rotation/command \
  -p publish_rate_hz:=20.0 \
  -p default_angular_speed:=0.02 \
  -p angular_speed_step:=0.005 \
  -p max_angular_speed:=0.05
```

参数说明：

`--params-file`

```text
加载键盘节点参数文件。
```

`command_topic`

```text
键盘控制命令的发布 Topic。
```

默认值：

```text
/fixed_axis_rotation/command
```

`publish_rate_hz`

```text
控制命令心跳发布频率，单位 Hz。
```

默认值：

```text
20.0
```

`default_angular_speed`

```text
按下 W 且当前速度为零时使用的初始角速度。
单位：rad/s。
```

默认值：

```text
0.02
```

`angular_speed_step`

```text
每次按下 D、A、+ 或 - 时的角速度变化量。
单位：rad/s。
```

默认值：

```text
0.005
```

`max_angular_speed`

```text
键盘能够设置的最大角速度绝对值。
单位：rad/s。
```

默认值：

```text
0.05
```

命令行中的 `-p` 参数会覆盖 YAML 文件中的同名参数。

正常使用时，参数已经写在 YAML 文件中，因此可以简化为：

```bash
ros2 run fixed_axis_rotation \
  fixed_axis_keyboard_node \
  --ros-args \
  --params-file \
  ~/franka_ros2_ws/src/fixed_axis_rotation/config/fixed_axis_rotation.yaml
```



---

## 10. 键盘操作

```text
W       开始或继续旋转

D / +   增大目标角速度绝对值

A / -   减小目标角速度绝对值

R       反转旋转方向

Space   正常停止，按照角加速度限制平滑减速

E       紧急停止，立即发送零关节速度

C       停止后重新记录当前固定空间轴

H       显示键盘帮助

Q       紧急停止并退出键盘节点
```

角速度正方向按照固定轴方向的右手定则定义。

---

## 11. 检查 ROS 2 通信

查看节点：

```bash
ros2 node list
```

正常应包含：

```text
/fixed_axis_rotation_node
/fixed_axis_keyboard_node
```

查看命令 Topic：

```bash
ros2 topic echo \
  /fixed_axis_rotation/command
```

查看命令频率：

```bash
ros2 topic hz \
  /fixed_axis_rotation/command
```

预期约为：

```text
20 Hz
```

查看关节状态频率：

```bash
ros2 topic hz \
  /franka/joint_states
```

查看主节点参数：

```bash
ros2 param dump \
  /fixed_axis_rotation_node
```

---

## 12. dry-run 日志检查

主控制节点正常运行时，应周期打印：

```text
theta
omega
target
position_error
orientation_error
sigma_min
condition
q_dot
dry_run
```

重点确认：

```text
dry_run=True
```

按下 `W` 后：

```text
target
```

应变为非零值。

```text
omega
```

应按照角加速度限制逐渐接近目标值。

```text
q_dot
```

应出现有限的非零关节速度。

按下空格后：

```text
omega
```

应逐渐减小到零。

按下 `E` 后应立即进入紧急停止。

---

## 13. 安全机制

当前控制系统包含：

1. 关节状态超时停止；
2. 键盘命令超时停止；
3. 非有限数值检查；
4. 关节位置限位检查；
5. 雅可比最小奇异值检查；
6. 雅可比条件数检查；
7. 末端线速度限制；
8. 末端角速度限制；
9. 单关节速度限制；
10. 角加速度限制；
11. 末端位置跟踪误差限制；
12. 末端姿态跟踪误差限制；
13. 紧急停止；
14. 节点退出前发送零关节速度；
15. `dry_run` 安全调试模式。

---

## 14. 真机运行前检查

必须确认以下项目：

```text
[ ] /tmp/fr3.urdf 存在

[ ] URDF 中存在 fr3_link8

[ ] /franka/joint_states 正常发布

[ ] 关节名称和顺序正确

[ ] 固定轴点 axis_point_link8 正确

[ ] 固定轴方向 axis_direction_link8 正确

[ ] 固定轴旋转半径正确

[ ] dry_run 中关节速度方向正确

[ ] W、Space、E、C 和 Q 按键正常

[ ] 命令超时后立即停止

[ ] 关节速度不超过 0.05 rad/s

[ ] 末端运动范围内没有人员和障碍物

[ ] 可以随时触发 Franka 急停
```

只有上述检查全部通过后，才可以运行：

```bash
ros2 launch fixed_axis_rotation \
  fixed_axis_rotation.launch.py \
  dry_run:=false
```

第一次真机运行应使用：

```yaml
default_angular_speed: 0.02
max_angular_speed: 0.05
max_joint_velocity: 0.05
```

并保持随时可以急停。

---

## 15. 常见问题

### 15.1 末端只旋转但不沿圆周移动

原因：

```yaml
axis_point_link8:
  - 0.0
  - 0.0
  - 0.0
```

固定轴经过末端原点，因此旋转半径为零。

需要将轴点设置为与末端原点存在偏移的位置。

### 15.2 提示等待关节状态

检查：

```bash
ros2 topic echo \
  /franka/joint_states \
  --once
```

并确认 YAML 中：

```yaml
joint_state_topic: "/franka/joint_states"
```

### 15.3 提示等待控制命令

确认键盘节点已在独立终端启动：

```bash
ros2 node list | grep keyboard
```

检查命令 Topic：

```bash
ros2 topic hz \
  /fixed_axis_rotation/command
```

### 15.4 提示 Jacobian is near singular

表示当前机械臂姿态接近运动学奇异位置。

控制节点会停止运动。不要直接降低安全阈值，应先改变机器人初始姿态。

### 15.5 dry-run 中没有机器人运动

这是正常现象。

```yaml
dry_run: true
```

表示仅计算，不发布真实关节速度。

