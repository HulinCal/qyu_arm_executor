from setuptools import find_packages, setup

package_name = 'qyu_ros2_simple_arm_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # 运行时依赖
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'std_msgs',
        'trajectory_msgs',
        'control_msgs',
        'moveit_msgs',
        'geometry_msgs',
    ],
    zip_safe=True,
    maintainer='David Dudas',
    maintainer_email='david.dudas@outlook.com',
    description='Python nodes for simulation of a robotic arm with Gazebo Harmonic and ROS Jazzy for BME MOGI ROS2 course',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_joint_angles = qyu_ros2_simple_arm_py.send_joint_angles:main',
            'inverse_kinematics = qyu_ros2_simple_arm_py.inverse_kinematics:main',
            'pick_object = qyu_ros2_simple_arm_py.pick_object:main',
            'gazebo_model_pose_publisher = qyu_ros2_simple_arm_py.gazebo_model_pose_publisher:main',
            'object_pose_publisher = qyu_ros2_simple_arm_py.object_pose_publisher:main',
        ],
    },
)