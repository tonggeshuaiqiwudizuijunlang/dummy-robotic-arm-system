import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取 dummy_moveit_config 的 share 路径
    dummy_moveit_config_dir = get_package_share_directory('dummy_moveit_config')
    
    # 包含本包下的 demo.launch.py
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dummy_moveit_config_dir, 'launch', 'demo.launch.py')
        )
    )

    # 启动串口发送脚本 (ExecuteProcess)
    # 因为该脚本就在直接的工程目录下，我们指明它的绝对路径
    usb_sender_node = ExecuteProcess(
        cmd=['python3', '/home/fishros/jixiebi/dummy_moveit_ws/test_usb_sender.py'],
        output='screen'
    )

    return LaunchDescription([
        demo_launch,
        usb_sender_node,
    ])
