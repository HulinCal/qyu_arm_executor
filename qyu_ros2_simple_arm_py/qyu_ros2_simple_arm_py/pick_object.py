import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import Pose
import subprocess
import re

# ==============================================
# 【1】从 Gazebo 获取 coke_can_steel 真实坐标
# ==============================================
def get_coke_pose():
    try:
        res = subprocess.run(
            ["gz", "model", "-m", "coke_can_steel", "--pose"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        match = re.search(r"\[\s*([\d\-.]+)\s+([\d\-.]+)\s+([\d\-.]+)", res.stdout)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            z = float(match.group(3))
            return x, y, z
    except:
        pass
    return 0.4, 0.2, 1.0  # 默认 fallback 坐标


# ==============================================
# 【2】MoveIt2 控制机械臂到达目标位置
# ==============================================
def main():
    rclpy.init()
    node = Node("moveit2_arm_controller")

    # 初始化 MoveItPy
    moveit = MoveItPy(node, "robot_description")
    arm_group = moveit.get_planning_component("arm")  # 你的规划组名，通常是 arm

    # 获取可乐罐位置
    cx, cy, cz = get_coke_pose()
    node.get_logger().info(f"🎯 目标位置：x={cx:.2f}, y={cy:.2f}, z={cz:.2f}")

    # 设置目标位姿
    target_pose = Pose()
    target_pose.position.x = cx
    target_pose.position.y = cy
    target_pose.position.z = cz + 0.15  # 抬高 15cm，防止撞地
    target_pose.orientation.w = 1.0     # 默认姿态

    # 设置目标
    arm_group.set_goal_position(target_pose)

    # 规划 + 运动
    node.get_logger().info("🔍 开始规划路径...")
    success = arm_group.plan()

    if success:
        node.get_logger().info("✅ 规划成功，开始执行！")
        arm_group.execute()
    else:
        node.get_logger().error("❌ 规划失败！")

    rclpy.shutdown()

if __name__ == "__main__":
    main()