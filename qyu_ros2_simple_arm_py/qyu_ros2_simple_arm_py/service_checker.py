import rclpy
from rclpy.node import Node

class ServiceChecker(Node):
    def __init__(self):
        super().__init__('service_checker')
        
        self.get_logger().info('Checking available services...')
        
        # 获取所有服务
        services = self.get_service_names_and_types()
        
        # 过滤出与 gazebo/gz 相关的服务
        gazebo_services = [s for s in services if 'gz' in s[0].lower() or 'gazebo' in s[0].lower()]
        
        self.get_logger().info(f'Found {len(gazebo_services)} Gazebo-related services:')
        for name, types in gazebo_services:
            self.get_logger().info(f'  {name}: {types}')
        
        self.get_logger().info('Service check completed')

def main(args=None):
    rclpy.init(args=args)
    node = ServiceChecker()
    
    try:
        # 运行2秒后退出
        import time
        time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()