import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState

class GazeboServiceChecker(Node):
    def __init__(self):
        super().__init__('gazebo_service_checker')
        
        self.get_logger().info('Checking available Gazebo services...')
        
        # 尝试不同的服务名称
        service_names = [
            '/gazebo/get_entity_state',
            '/gz/get_entity_state',
            '/world/empty/get_entity_state',
            '/world/default/get_entity_state',
        ]
        
        for service_name in service_names:
            self.get_logger().info(f'Trying service: {service_name}')
            client = self.create_client(GetEntityState, service_name)
            
            if client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'✓ Service found: {service_name}')
            else:
                self.get_logger().warn(f'✗ Service not available: {service_name}')
        
        self.get_logger().info('Service check completed')

def main(args=None):
    rclpy.init(args=args)
    node = GazeboServiceChecker()
    
    try:
        # 运行5秒后退出
        import time
        time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()