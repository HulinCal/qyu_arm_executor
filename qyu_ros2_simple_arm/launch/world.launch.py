import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_qyu_ros2_navigation = get_package_share_directory('qyu_ros2_simple_arm')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_qyu_ros2_navigation_share = os.path.join(pkg_qyu_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] = pkg_qyu_ros2_navigation_share + os.pathsep + os.environ.get("GZ_SIM_RESOURCE_PATH", "")


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_qyu_ros2_navigation,
            'worlds',
            LaunchConfiguration('world')
        ]),
        TextSubstitution(text=' -r -v -v1 --render-engine ogre2 --render-engine-gui-api-backend opengl')],
        'on_exit_shutdown': 'true'}.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject