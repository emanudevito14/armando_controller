from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.conditions import LaunchConfigurationEquals
# Importiamo IfCondition per rendere condizionale l'EventHandler
from launch.conditions import IfCondition 
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    declared_arguments = []

    # 1. declared argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_type',
            default_value='position',
            description='Type of controller: "position" or "trajectory"'
        )
    )
    controller_type = LaunchConfiguration('controller_type')

    declared_arguments.append(
        DeclareLaunchArgument('gz_args', default_value='-r -v 1 empty.sdf', description='Arguments for gz_sim')
    )
    
    # 2. path defination
    armando_controller_path = get_package_share_directory('armando_controller')
    xacro_file = os.path.join(armando_controller_path, 'urdf', 'arm_old.urdf.xacro')
    
   
    controllers_config_file = os.path.join(armando_controller_path, 'config', 'arm_controllers.yaml') 

    robot_description = {"robot_description": Command(['xacro ', xacro_file])}

    # 3. Node Robot State Publisher (RSP)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description, 
            {"use_sim_time": True},
            controllers_config_file # <-- I parametri YAML VENGONO CARICATI QUI
        ],
    )

    # 4.  Gazebo (Ignition)
    gazebo_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    # 5. Spawn in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'armando', '-allow_renaming', 'true', '-z', '2'],
    )

    # 6.  Spawner  Controller
    
    # Spawner  Joint State Broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Spawner Position Controller 
  
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=LaunchConfigurationEquals('controller_type', 'position')
    )
 
    
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=LaunchConfigurationEquals('controller_type', 'trajectory')
    )

    # 7. controller C++
    arm_controller_node = Node(
        package="armando_controller",
        executable="arm_controller_node",
        name="arm_controller",
        output="screen",
        parameters=[{"publisher_type": controller_type}]
    )

    # 8. (Event Handlers)
    
   
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster],
        )
    )

    
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                position_controller_spawner,
                trajectory_controller_spawner,
            ],
        )
    )

    
    delay_arm_controller_position = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_controller_spawner,
            on_exit=[arm_controller_node], # Avvia il nodo C++
        ),
        condition=LaunchConfigurationEquals('controller_type', 'position')
    )

  
    delay_arm_controller_trajectory = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=trajectory_controller_spawner,
            on_exit=[arm_controller_node], 
        ),
        condition=LaunchConfigurationEquals('controller_type', 'trajectory')
    )

 
    nodes_to_start = [
        robot_state_publisher_node,
        gazebo_ignition,
        gz_spawn_entity,
        delay_joint_state_broadcaster,
        delay_controller,
        delay_arm_controller_position, 
        delay_arm_controller_trajectory,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
