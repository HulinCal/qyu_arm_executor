#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <thread>
#include <mutex>

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class ObjectPoseListener : public rclcpp::Node
{
public:
    ObjectPoseListener() : Node("object_pose_listener"), pose_received_(false)
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/detected_object_position", 10,
            std::bind(&ObjectPoseListener::poseCallback, this, std::placeholders::_1));
    }

    bool isPoseReceived()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return pose_received_;
    }

    geometry_msgs::msg::PointStamped getPose()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_pose_;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_received_ = false;
    }

private:
    std::mutex mutex_;
    bool pose_received_;
    geometry_msgs::msg::PointStamped latest_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pose_sub_;

    void poseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_pose_ = *msg;
        pose_received_ = true;
        RCLCPP_INFO(this->get_logger(), "📥 收到物体位置: (%.4f, %.4f, %.4f)",
            msg->point.x, msg->point.y, msg->point.z);
    }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_to_pose_node");
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  // 创建位置监听器
  auto pose_listener = std::make_shared<ObjectPoseListener>();

  // ==============================================
  // 【等待检测到物体位置】
  // ==============================================
  RCLCPP_INFO(node->get_logger(), "⏳ 等待检测物体位置...");
  rclcpp::Rate loop_rate(10);
  while (!pose_listener->isPoseReceived() && rclcpp::ok()) {
    rclcpp::spin_some(pose_listener);
    loop_rate.sleep();
  }

  if (!rclcpp::ok()) {
    RCLCPP_ERROR(node->get_logger(), "❌ 等待检测位置时节点已退出");
    rclcpp::shutdown();
    return 1;
  }

  // 获取检测到的位置
  auto detected_pose = pose_listener->getPose();
  RCLCPP_INFO(node->get_logger(), "✅ 使用检测到的位置: (%.4f, %.4f, %.4f)",
    detected_pose.point.x, detected_pose.point.y, detected_pose.point.z);

  // 等待数据
  rclcpp::sleep_for(1s);
  rclcpp::spin_some(node);

  // 1. 初始化两个规划组（你的配置）
  MoveGroupInterface arm_group(node, "arm");       // 机械臂
  MoveGroupInterface gripper_group(node, "gripper");// 夹爪

  // 通用设置
  arm_group.setPlanningTime(60.0);
  arm_group.setMaxVelocityScalingFactor(0.2);
  arm_group.setMaxAccelerationScalingFactor(0.1);

  // ==============================================
  // 【第一步：张开夹爪（双关节同时控制）】
  // ==============================================
  RCLCPP_INFO(node->get_logger(), "✅ 开始张开夹爪");

  // 直接设置两个关节值（适配你的夹爪）
  gripper_group.setJointValueTarget("left_finger_joint", 0.035);
  gripper_group.setJointValueTarget("right_finger_joint", 0.035);

  MoveGroupInterface::Plan gripper_plan;
  bool gripper_success = (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (gripper_success) {
    gripper_group.execute(gripper_plan);

    // 等待机械臂稳定--不能同时打开，只能执行两次，还需要解决这个问题
    rclcpp::sleep_for(1s);
    gripper_group.execute(gripper_plan);

    RCLCPP_INFO(node->get_logger(), "✅ 夹爪已张开！");
  } else {
    RCLCPP_ERROR(node->get_logger(), "❌ 夹爪规划失败");
  }

  // 等待机械臂稳定
  rclcpp::sleep_for(1s);

  // ==============================================
  // 【第二步：设置机械臂目标位姿 - 使用检测到的位置】
  // ==============================================
  RCLCPP_INFO(node->get_logger(), "✅ 机械臂 去目标点...");
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = node->get_clock()->now();
  
  // 使用检测到的位置
  target_pose.pose.position.x = detected_pose.point.x;
  target_pose.pose.position.y = detected_pose.point.y;
  target_pose.pose.position.z = detected_pose.point.z;

  target_pose.pose.orientation.x = 0.0000; // 已验证的安全姿态：x = -0.59375
  target_pose.pose.orientation.y = 0.7634; // 已验证的安全姿态：y = -0.60903
  target_pose.pose.orientation.z = 0.0000; // 已验证的安全姿态：z = -0.13927
  target_pose.pose.orientation.w = 0.6459; // 已验证的安全姿态：w = 0.5071

  arm_group.setPoseTarget(target_pose,"end_effector_link");
  MoveGroupInterface::Plan arm_plan;
  if (arm_group.plan(arm_plan)) {
    RCLCPP_INFO(node->get_logger(), "🚀 开始执行机械臂运动...");
    arm_group.execute(arm_plan);
    
    // 额外等待确保机械臂完全停止
    RCLCPP_INFO(node->get_logger(), "⏳ 等待机械臂稳定...");
    rclcpp::sleep_for(2s);
    
    // 再次确认夹爪保持张开状态
    RCLCPP_INFO(node->get_logger(), "🔧 保持夹爪张开...");
    gripper_group.setJointValueTarget("left_finger_joint", 0.035);
    gripper_group.setJointValueTarget("right_finger_joint", 0.035);
    bool grip_ok = (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (grip_ok) {
      gripper_group.execute(gripper_plan);
    }
    
    RCLCPP_INFO(node->get_logger(), "✅ 机械臂到达目标！");
  }

  // ==============================================
  // 使用etJointValueTarget
  // ==============================================
  //   std::map<std::string, double> target_joints = {
  //   {"elbow_joint",         0.92502}, //53度
  //   {"shoulder_lift_joint", 1.117}, //64度
  //   {"shoulder_pan_joint",   0.0},
  //   {"wrist_joint",         -0.31416} //-18度
  // };
  
  // // 设置目标
  // arm_group.setJointValueTarget(target_joints);
  
  // MoveGroupInterface::Plan arm_plan;
  // bool plan_ok = (arm_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  // if (plan_ok) {
  //   RCLCPP_INFO(node->get_logger(), "✅ 规划成功，机械臂运动到目标位置！");
  //   arm_group.execute(arm_plan);
    
  //   // 确保夹爪在机械臂运动后保持张开状态
  //   RCLCPP_INFO(node->get_logger(), "🔧 保持夹爪张开...");
  //   gripper_group.setJointValueTarget("left_finger_joint", 0.035);
  //   gripper_group.setJointValueTarget("right_finger_joint", 0.035);
  //   gripper_success = (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  //   if (gripper_success) {
  //     gripper_group.execute(gripper_plan);
  //   }
  // } else {
  //   RCLCPP_ERROR(node->get_logger(), "❌ 规划失败");
  //   rclcpp::shutdown();
  //   return 1;
  // }

  rclcpp::sleep_for(std::chrono::seconds(2));

  // ==============================================
  // 【第三步：关闭夹爪】
  // 直接设置两个关节值（适配你的夹爪）
  gripper_group.setJointValueTarget("left_finger_joint", 0.02);
  gripper_group.setJointValueTarget("right_finger_joint", 0.02);

  gripper_success = (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (gripper_success) {
    gripper_group.execute(gripper_plan);

    // 等待机械臂稳定--不能同时打开，只能执行两次，还需要解决这个问题
    rclcpp::sleep_for(1s);

    gripper_group.execute(gripper_plan);

    RCLCPP_INFO(node->get_logger(), "✅ 夹爪已关闭！");
  }

  rclcpp::sleep_for(1s);


  // ==============================================
  // 第四步，抓住，并离开地面并转动
  // ==============================================
  std::map<std::string, double> target_joints = {
    {"elbow_joint",         0.92502}, //53度
    {"shoulder_lift_joint", 0.8727}, //64度
    {"shoulder_pan_joint",   0.5},
    {"wrist_joint",         -0.31416} //-18度
  };

  // 设置目标
  arm_group.setJointValueTarget(target_joints);

  bool plan_ok = (arm_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (plan_ok) {
    RCLCPP_INFO(node->get_logger(), "✅ 第四步--规划成功，机械臂运动到目标位置！");
    arm_group.execute(arm_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "❌ 第四步--规划失败");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::sleep_for(1s);

  // ==============================================
  // 第五步，放回地面
  // ==============================================
  target_joints = {
    {"elbow_joint",         0.92502}, //53度
    {"shoulder_lift_joint", 1.117}, //64度
    {"shoulder_pan_joint",   0.5},
    {"wrist_joint",         -0.31416} //-18度
  };

  // 设置目标
  arm_group.setJointValueTarget(target_joints);

  plan_ok = (arm_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (plan_ok) {
    RCLCPP_INFO(node->get_logger(), "✅ 第五步--规划成功，机械臂运动到目标位置！");
    arm_group.execute(arm_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "❌ 第五步--规划失败");
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::sleep_for(1s);

  // ==============================================
  // 【第六步：打开夹爪】
  // 直接设置两个关节值（适配你的夹爪）
  gripper_group.setJointValueTarget("left_finger_joint", 0.035);
  gripper_group.setJointValueTarget("right_finger_joint", 0.035);

  gripper_success = (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (gripper_success) {
    gripper_group.execute(gripper_plan);

    // 等待机械臂稳定--不能同时打开，只能执行两次，还需要解决这个问题
    rclcpp::sleep_for(1s);

    gripper_group.execute(gripper_plan);

    RCLCPP_INFO(node->get_logger(), "✅ 夹爪已打开！");
  }

  // ==============================================
  // 第七步，抬起夹抓
  // ==============================================
  target_joints = {
    {"elbow_joint",         0.92502}, //53度
    {"shoulder_lift_joint", 0.85}, //64度
    {"shoulder_pan_joint",   0.5},
    {"wrist_joint",         -0.31416} //-18度
  };

  // 设置目标
  arm_group.setJointValueTarget(target_joints);

  plan_ok = (arm_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (plan_ok) {
    RCLCPP_INFO(node->get_logger(), "✅ 第六步--规划成功，机械臂运动到目标位置！");
    arm_group.execute(arm_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "❌ 第六步--规划失败");
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::sleep_for(1s);

  // ==============================================
  // 【第八步：回到up
  // ==============================================
  RCLCPP_INFO(node->get_logger(), "✅ 机械臂 目标-up...");
  arm_group.setNamedTarget("up");
  MoveGroupInterface::Plan up_plan;
  if (arm_group.plan(up_plan)) arm_group.execute(up_plan);


  RCLCPP_INFO(node->get_logger(), "🎉 任务全部完成！");

  rclcpp::shutdown();
  return 0;
}
