# FixedAxisCommand.msg
#
# 功能：
#     定义固定轴旋转控制节点使用的控制指令。
#
# 输入来源：
#     fixed_axis_keyboard_node
#
# 输出目标：
#     fixed_axis_rotation_node
#
# 说明：
#     target_angular_speed 为带符号角速度。
#
#     正值：
#         沿固定轴方向，按照右手定则旋转。
#
#     负值：
#         沿固定轴反方向旋转。
#
#     零值：
#         目标角速度为零。
#
#     单位：
#         rad/s。


# ============================================================
# 控制指令常量
# ============================================================

# 保持停止状态。
uint8 COMMAND_STOP=0

# 按 target_angular_speed 运行。
uint8 COMMAND_RUN=1

# 立即急停，不经过速度缓降。
uint8 COMMAND_EMERGENCY_STOP=2

# 根据当前 fr3_link8 位姿，
# 重新记录空间固定轴的位置和方向。
uint8 COMMAND_CAPTURE_AXIS=3


# ============================================================
# 消息数据
# ============================================================

# 控制指令类型。
uint8 command

# 目标旋转角速度。
# 正负号决定旋转方向。
float64 target_angular_speed