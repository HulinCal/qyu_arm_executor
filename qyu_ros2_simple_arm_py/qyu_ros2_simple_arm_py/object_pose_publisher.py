import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool
import time

class ObjectPosePublisher(Node):
    def __init__(self):
        super().__init__('object_pose_publisher')
        
        # 创建 ROS 发布器
        self.pose_publisher = self.create_publisher(
            Pose,
            '/model/coke_can/pose',
            10
        )
        
        # 创建服务来设置物体位置
        self.set_position_service = self.create_service(
            SetBool,
            '/set_object_position',
            self.set_position_callback
        )
        
        # 机械臂基座在世界坐标系中的位置（从 spawn_robot.launch.py 获取）
        self.arm_base_world_position = [0.0, 0.0, 1.04]
        
        # 默认物体位置（世界坐标系，根据 world.sdf 中的 coke_can_steel 位置）
        self.object_world_position = [0.4004, 0.2012, 1.0148]
        
        # 定时器发布位置
        self.timer = self.create_timer(0.1, self.publish_pose)
        
        self.get_logger().info('Object Pose Publisher started')
        self.get_logger().info(f'Arm base position (world): {self.arm_base_world_position}')
        self.get_logger().info(f'Object position (world): {self.object_world_position}')
        self.get_logger().info('Publishing to: /model/coke_can/pose (relative to arm base)')
    
    def world_to_base_frame(self, world_pos):
        """将世界坐标系转换为机械臂基座坐标系"""
        base_pos = [
            world_pos[0] - self.arm_base_world_position[0],
            world_pos[1] - self.arm_base_world_position[1],
            world_pos[2] - self.arm_base_world_position[2]
        ]
        return base_pos
    
    def publish_pose(self):
        """发布物体位置（相对于机械臂基座）"""
        # 将世界坐标转换为机械臂基座坐标
        object_base_position = self.world_to_base_frame(self.object_world_position)
        
        pose = Pose()
        pose.position.x = object_base_position[0]
        pose.position.y = object_base_position[1]
        pose.position.z = object_base_position[2]
        pose.orientation.w = 1.0  # 默认朝向
        
        self.pose_publisher.publish(pose)
        
        # 每5秒打印一次位置信息
        current_time = self.get_clock().now().nanoseconds
        if current_time % 5000000000 < 100000000:
            self.get_logger().info(
                f'Object position (relative to arm base): '
                f'[{object_base_position[0]:.4f}, {object_base_position[1]:.4f}, {object_base_position[2]:.4f}]'
            )
    
    def set_position_callback(self, request, response):
        """设置物体位置服务回调"""
        if request.data:
            # 重置为默认位置
            self.object_world_position = [0.4004, 0.2012, 1.0148]
        else:
            # 可以在这里添加自定义位置设置逻辑
            pass
        
        self.get_logger().info(f'Object world position updated to: {self.object_world_position}')
        response.success = True
        response.message = f'World position set to: {self.object_world_position}'
        return response
    
    def run(self):
        """运行节点"""
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPosePublisher()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()