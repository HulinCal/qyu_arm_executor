#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <thread>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("arm_gripper_node");
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  // ==========================
  // 1. 机械臂控制
  // ==========================
  MoveGroupInterface move_arm(node, "arm");
  move_arm.setPlanningTime(5.0);
  move_arm.setMaxVelocityScalingFactor(0.6);
  move_arm.setMaxAccelerationScalingFactor(0.6);
  move_arm.setGoalPositionTolerance(0.005);
  move_arm.setGoalOrientationTolerance(0.005);

  // ==========================
  // 2. 夹抓控制
  // ==========================
  MoveGroupInterface move_gripper(node, "gripper");
  move_gripper.setPlanningTime(5.0);
  move_gripper.setMaxVelocityScalingFactor(0.6);
  move_gripper.setMaxAccelerationScalingFactor(0.6);
  move_gripper.setGoalPositionTolerance(0.005);
  move_gripper.setGoalOrientationTolerance(0.005);


  //----------------------------------------------------
  // 张开夹抓---setJointValueTarget方式，不同同时控制两个夹抓
  //----------------------------------------------------
  std::map<std::string, double> gripper_open = {
    {"left_finger_joint", 0.035},
    {"right_finger_joint", 0.035}
  };

  // 发送目标
  move_gripper.setJointValueTarget(gripper_open);

  // 规划 + 执行
  MoveGroupInterface::Plan open_plan;
  if (move_gripper.plan(open_plan)) {
    move_gripper.execute(open_plan);

    // 等待动作完成--再次执行，因为调用一次只能移动一个夹抓，这个问题还没解决
    rclcpp::sleep_for(std::chrono::seconds(2));
    move_gripper.execute(open_plan);
    RCLCPP_INFO(node->get_logger(), "✅ 双夹爪同步打开！");
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  // up
  // RCLCPP_INFO(node->get_logger(), "✅ 机械臂 目标-up...");
  // move_arm.setNamedTarget("up");
  // MoveGroupInterface::Plan up_plan;
  // if (move_arm.plan(up_plan)) move_arm.execute(up_plan);
  // rclcpp::sleep_for(std::chrono::seconds(1));

  // 去目标点--完成
  RCLCPP_INFO(node->get_logger(), "✅ 机械臂 去目标点...");
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = node->get_clock()->now();
  target_pose.pose.position.x = 0.5854;   // 已验证：x = 0.15
  target_pose.pose.position.y = 0.0012;   // 已验证：y = 0.18 
  target_pose.pose.position.z = 0.005;   // 已验证：z = 0.41

  target_pose.pose.orientation.x = 0.0000; // 已验证的安全姿态：x = -0.59375
  target_pose.pose.orientation.y = 0.7634; // 已验证的安全姿态：y = -0.60903
  target_pose.pose.orientation.z = 0.0000; // 已验证的安全姿态：z = -0.13927
  target_pose.pose.orientation.w = 0.6459; // 已验证的安全姿态：w = 0.5071

  move_arm.setPoseTarget(target_pose,"end_effector_link");
  MoveGroupInterface::Plan arm_plan;
  if (move_arm.plan(arm_plan)) {
    move_arm.execute(arm_plan);
    RCLCPP_INFO(node->get_logger(), "✅ 机械臂到达目标！");
  }
  rclcpp::sleep_for(std::chrono::seconds(2));

  //----------------------------------------------------
  // 张开夹抓---setNamedTarget方式，不同同时控制两个夹抓
  //----------------------------------------------------
  //张开夹抓
  // RCLCPP_INFO(node->get_logger(), "✅ 机械臂 张开夹抓...");
  // move_gripper.setNamedTarget("open");
  // MoveGroupInterface::Plan open_plan;
  // if (move_gripper.plan(open_plan)) {
  //   move_gripper.execute(open_plan);
  //   RCLCPP_INFO(node->get_logger(), "✅ 夹抓到达目标！");
  // } 
  // rclcpp::sleep_for(std::chrono::seconds(2));


  rclcpp::shutdown();
  return 0;
}
