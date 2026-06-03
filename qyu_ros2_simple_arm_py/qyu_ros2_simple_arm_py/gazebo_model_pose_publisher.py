import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import subprocess
import time
import re

class GazeboModelPosePublisher(Node):
    def __init__(self, model_name='coke_can_steel'):
        super().__init__('gazebo_model_pose_publisher')
        
        self.model_name = model_name
        
        # 创建 ROS 发布器
        self.pose_publisher = self.create_publisher(
            Pose,
            f'/model/{model_name}/pose',
            10
        )
        
        # 机械臂基座在世界坐标系中的位置（从 spawn_robot.launch.py 获取）
        self.arm_base_world_position = [0.0, 0.0, 1.04]
        
        # 机械臂工作空间限制
        self.min_z = 0.5  # 最低可达高度（相对基座）
        
        # 定时器：每0.1秒查询一次模型位置
        self.timer = self.create_timer(0.1, self.query_model_pose)
        
        self.get_logger().info(f'Gazebo Model Pose Publisher started for: {model_name}')
        self.get_logger().info(f'Arm base position (world): {self.arm_base_world_position}')
        self.get_logger().info(f'Minimum reachable z height: {self.min_z}m')
        self.get_logger().info(f'Publishing to: /model/{model_name}/pose (relative to arm base)')
    
    def get_model_pose(self):
        """使用 gz model 命令获取模型位置"""
        try:
            # 调用 gz model 命令获取模型信息
            result = subprocess.run(
                ['gz', 'model', '-m', self.model_name, '--pose'],
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                output = result.stdout
                # 解析位置信息
                # 输出格式:
                #   Pose [ XYZ (m) ] [ RPY (rad) ]:
                #     [0.400417 0.201180 1.014900]
                #     [-0.011212 0.004028 0.000236]
                match = re.search(r'\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]', output)
                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    z = float(match.group(3))
                    return [x, y, z]
                else:
                    self.get_logger().warn(f'Could not parse pose from output: {output}')
                    return None
            else:
                self.get_logger().warn(f'gz model command failed: {result.stderr}')
                return None
        except Exception as e:
            self.get_logger().error(f'Error getting model pose: {str(e)}')
            return None
    
    def world_to_base_frame(self, world_pos):
        """将世界坐标系转换为机械臂基座坐标系"""
        base_pos = [
            world_pos[0] - self.arm_base_world_position[0],
            world_pos[1] - self.arm_base_world_position[1],
            world_pos[2] - self.arm_base_world_position[2]
        ]
        
        # 确保 z 坐标在工作空间内（最低 0.1m）
        if base_pos[2] < self.min_z:
            self.get_logger().warn(
                f'Object z position {base_pos[2]:.4f}m is below minimum height {self.min_z}m. '
                f'Adjusting to {self.min_z}m'
            )
            base_pos[2] = self.min_z
        
        return base_pos
    
    def query_model_pose(self):
        """查询并发布模型位置"""
        world_pos = self.get_model_pose()
        
        if world_pos is not None:
            # 将世界坐标转换为机械臂基座坐标
            base_pos = self.world_to_base_frame(world_pos)
            
            # 创建并发布 Pose 消息
            pose = Pose()
            pose.position.x = world_pos[0]
            pose.position.y = world_pos[1]
            pose.position.z = world_pos[2]
            pose.orientation.w = 1.0  # 默认朝向
            
            self.pose_publisher.publish(pose)
            
            # 每5秒打印一次位置信息
            current_time = self.get_clock().now().nanoseconds
            if current_time % 5000000000 < 100000000:
                self.get_logger().info(
                    f'{self.model_name} position (world): [{world_pos[0]:.4f}, {world_pos[1]:.4f}, {world_pos[2]:.4f}]'
                )
                self.get_logger().info(
                    f'{self.model_name} position (relative to arm base): [{base_pos[0]:.4f}, {base_pos[1]:.4f}, {base_pos[2]:.4f}]'
                )
    
    def run(self):
        """运行节点"""
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    
    # 可以通过命令行参数指定模型名称
    model_name = 'coke_can_steel'
    
    node = GazeboModelPosePublisher(model_name)
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()