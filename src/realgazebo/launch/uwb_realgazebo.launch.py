# rosunreal.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def resolve_share_path(pkg: str, relpath: str) -> str:
    if os.path.isabs(relpath):
        return relpath
    return os.path.join(get_package_share_directory(pkg), relpath)

def launch_setup(context, *args, **kwargs):
    # LaunchConfigurations -> 실제 문자열로 치환
    ros_package = LaunchConfiguration('ros_package').perform(context)
    model_yaml  = LaunchConfiguration('model_yaml').perform(context)
    scenario_yaml = LaunchConfiguration('scenario_yaml').perform(context)

    px4_motor_output     = LaunchConfiguration('px4_motor_output').perform(context)
    drone_position_output = LaunchConfiguration('drone_position_output').perform(context)
    tag_position_output  = LaunchConfiguration('tag_position_output').perform(context)
    detect_position_output  = LaunchConfiguration('detect_position_output').perform(context)
    position_tolerance_text = LaunchConfiguration('position_tolerance').perform(context)
    unreal_ip            = LaunchConfiguration('unreal_ip').perform(context)
    unreal_port_text     = LaunchConfiguration('unreal_port').perform(context)

    try:
        unreal_port = int(unreal_port_text)
        position_tolerance = float(position_tolerance_text)
    except Exception:
        unreal_port = 5005
        position_tolerance = 3.0

    params = {
        'package_name': ros_package,
        'model_yaml': resolve_share_path(ros_package, model_yaml),
        'scenario_yaml': resolve_share_path(ros_package, scenario_yaml),
        'px4_motor_output': px4_motor_output,
        'drone_position_output': drone_position_output,
        'tag_position_output': tag_position_output,
        'detect_position_output': detect_position_output,
        'position_tolerance' : position_tolerance,
        'unreal_ip': unreal_ip,
        'unreal_port': unreal_port,
    }

    node = Node(
        package='realgazebo',      # 패키지명
        executable='rosunreal',    # 빌드된 실행파일명(CMake에서 설정한 이름)
        name='rosunreal',
        output='screen',
        parameters=[params],
    )
    return [node]

def generate_launch_description():
    # 코드의 declare_parameter 기본값과 동일하게
    args = [
        DeclareLaunchArgument('ros_package', default_value='realgazebo',
                              description='Package name used to resolve share paths'),
        DeclareLaunchArgument('model_yaml', default_value='yaml/models/x500.yaml'),
        DeclareLaunchArgument('scenario_yaml', default_value='yaml/scenario/uwb_drone.yaml'),
        DeclareLaunchArgument('px4_motor_output', default_value='/manager/out/actuator_motors'),
        DeclareLaunchArgument('drone_position_output', default_value='/fmu/out/monitoring'),
        DeclareLaunchArgument('tag_position_output', default_value='/jfi/in/target'),
        DeclareLaunchArgument('detect_position_output', default_value='/detections_posearray'),
        DeclareLaunchArgument('position_tolerance', default_value='2.0'),
        DeclareLaunchArgument('unreal_ip', default_value='10.255.70.224'),
        DeclareLaunchArgument('unreal_port', default_value='5005'),
        OpaqueFunction(function=launch_setup),
    ]
    return LaunchDescription(args)
