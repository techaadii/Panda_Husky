import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    pkg_husky_panda = get_package_share_directory('husky_panda_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_panda_moveit = get_package_share_directory('panda_moveit')

    # 1. Paths
    world_file = os.path.join(pkg_husky_panda, 'worlds', 'pick_place.sdf')
    urdf_file = os.path.join(pkg_husky_panda, 'urdf', 'husky_panda.urdf')

    # 2. Load Robot Description (URDF)
    # CRITICAL: Even though Gazebo uses SDF, ROS/MoveIt NEEDS the URDF for kinematics.
    with open(urdf_file, 'r') as inf:
        robot_desc_content = inf.read()
    robot_description = {'robot_description': robot_desc_content}

    # 3. Load MoveIt Configs
    srdf_file = os.path.join(pkg_panda_moveit, 'config', 'panda.srdf')
    with open(srdf_file, 'r') as inf:
        robot_desc_sem_content = inf.read()
    robot_description_semantic = {'robot_description_semantic': robot_desc_sem_content}
    
    kinematics_yaml = load_yaml('panda_moveit', 'config/kinematics.yaml')
    moveit_controllers = load_yaml('panda_moveit', 'config/moveit_controllers.yaml')

    # 4. Environment Variables
    ign_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=['/opt/ros/humble/lib', ':', os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')]
    )
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(pkg_husky_panda, 'models'), ':', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
    )

    # 5. Nodes

    # Robot State Publisher (Publishes TFs based on the URDF)
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Move Group (Motion Planning)
    node_move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            moveit_controllers,
            {
                'use_sim_time': True,
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_manage_controllers': True,
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.execution_duration_monitoring': True
            }
        ]
    )

    # Gazebo Simulation
    # Note: We use 'gz_sim.launch.py' and 'gz_args' for Humble+
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    # Bridge (Connects Gazebo topics to ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Camera Bridges
            '/wrist_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/husky_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        ],
        output='screen'
    )

    # Controller Spawners
    # Note: We do NOT spawn the robot here (it's in the SDF). We only spawn controllers.
    spawn_jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    spawn_husky = Node(package="controller_manager", executable="spawner", arguments=["husky_velocity_controller"])
    spawn_arm = Node(package="controller_manager", executable="spawner", arguments=["panda_arm_controller"])
    spawn_hand = Node(package="controller_manager", executable="spawner", arguments=["panda_hand_controller"])

    return LaunchDescription([
        ign_plugin_path,
        ign_resource_path,
        node_rsp,
        node_move_group,
        ign_gazebo,
        bridge,
        spawn_jsb,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_jsb,
                on_exit=[spawn_husky, spawn_arm, spawn_hand],
            )
        )
    ])