# Copyright (C) 2023 Open Source Robotics Foundation
# Copyright (C) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.actions import TimerAction

from launch_ros.actions import Node

def  launch_nodes(context, *args, **kwargs):
    actions = []



    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time_bool = (str(use_sim_time).lower() == 'true')
    use_simulator = LaunchConfiguration('use_simulator').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    output = LaunchConfiguration('output').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)

    x = LaunchConfiguration('x_pose', default='-8.00').perform(context) 
    y = LaunchConfiguration('y_pose', default='-0.50').perform(context)
    z = LaunchConfiguration('z_pose', default='0.01').perform(context)
    R = LaunchConfiguration('roll', default='0.00').perform(context)
    P = LaunchConfiguration('pitch', default='0.00').perform(context)
    Y = LaunchConfiguration('yaw', default='0.00').perform(context)


    # Gazeboのbridge設定を各ロボット用にremapping
    # Gazebo→ROS 2のブリッジ設定
    # DiffDriveプラグインが配信するtf, odom, cmd_vel, joint_statesをブリッジ
    # センサーデータ(scan, imu)もブリッジ
    # 注意: parameter_bridgeは引数で渡されたトピック名をそのまま使用する
    # bridgeノードが名前空間で実行されていても、自動的に名前空間は付与されない
    bridge_topics = [
        f'/{namespace}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        f'/{namespace}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        f'/{namespace}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        f'/{namespace}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        f'/{namespace}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        f'/{namespace}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        # '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]

    # 1. Gazebo側の長いトピック名を構築
    # 注意: 'link/base_footprint' の部分はURDFの構造(Lump処理)に依存します。
    # ログで確認したパス: /world/square15/model/robot_1/link/base_footprint/sensor/sensor_contact/contact
    gz_contact_topic = f'/world/{world_name}/model/{robot_name}/link/base_footprint/sensor/sensor_contact/contact'

    # 2. ROS側の希望するトピック名 (例: /robot_1/contact)
    ros_contact_topic = f'/{namespace}/contact'

    # 3. ブリッジリストに追加 (Gazeboトピック名@ROS型[Gazebo型)
    # ここでは型定義だけを行い、名前の変更は下の Node の remappings で行います
    bridge_topics.append(
        f'{gz_contact_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
    )
    # ========================================================================

    args = bridge_topics + ['--ros-args', '--log-level', log_level]
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        namespace=namespace,
        arguments=args,
        parameters=[{'use_sim_time': use_sim_time_bool}],
        remappings=[(gz_contact_topic, ros_contact_topic)],
        output=output
    )

    # camera_bridge_image = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     name='bridge_gz_ros_camera_image',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time_bool,
    #     }],
    #     arguments=[f'/{namespace}/rgbd_camera/image' if namespace else '/rgbd_camera/image'])

    # camera_bridge_depth = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     name='bridge_gz_ros_camera_depth',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time_bool,
    #     }],
    #     arguments=[f'/{namespace}/rgbd_camera/depth_image' if namespace else '/rgbd_camera/depth_image'])

    # topic_path = f'/{namespace}/robot_description' if namespace else '/robot_description'



    # static_tf_publisher_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name=f'{namespace}_map_to_odom_tf',
    #     arguments=[
    #         '0', '0', '0',      # x y z
    #         '0', '0', '0',      # roll pitch yaw
    #         f'{namespace}/map',
    #         f'{namespace}/odom',
    #     ],
    #     output='screen',
    #     remappings=[
    #         ('/tf_static', f'/{namespace}/tf_static')
    #     ]
    # )

    # spawn_model = Node(
    #     condition=IfCondition(use_simulator),
    #     package='ros_gz_sim',
    #     executable='create',
    #     # namespace=namespace,  # 修正
    #     output='screen',
    #     arguments=[
    #         '-entity', robot_name,
    #         '-topic', topic_path,
    #         # '-file', Command(['xacro', ' ', robot_sdf]), # TODO SDF file is unhappy, not sure why
    #         '-robot_namespace', namespace,
    #     ],
        
    #     #パラメータとしてロボット名を渡すよう修正
    #     parameters=[
    #         {
    #             'use_sim_time': use_sim_time_bool, 
    #             'name': robot_name,
    #             'x': float(x), 'y': float(y), 'z': float(z),
    #             'R': float(R), 'P': float(P), 'Y': float(Y)   
    #         }    
    #     ]
    # )


    # twist2twist_stamped_node = Node(
    #     package='twist2twist_stamped',
    #     executable='twist2twist_stamped',
    #     namespace=namespace,
    #     name='twist2twist_stamped',
    #     output=output,
    #     parameters=[{'use_sim_time': use_sim_time_bool, 'namespace': robot_name}]
    # )

    # # 【追加】 Joint State Broadcaster の起動
    # # ロボットの関節状態（joint_states）を配信します
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     namespace=namespace, # 名前空間内で実行
    #     arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    #     parameters=[{'use_sim_time': use_sim_time_bool}],
    # )


    # # 【追加】 Diff Drive Controller の起動
    # # オドメトリ(odom/tf)配信と速度指令(cmd_vel)の受信を担当します
    # diff_drive_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     namespace=namespace, # 名前空間内で実行
    #     arguments=["diff_drive_controller", "--controller-manager", "controller_manager"],
    #     parameters=[{'use_sim_time': use_sim_time_bool}],
    # )


    actions.append(bridge)
    # actions.append(static_tf_publisher_node)

    # actions.append(camera_bridge_image)
    # actions.append(camera_bridge_depth)
    # actions.append(spawn_model)

    # actions.append(twist2twist_stamped_node)

    # # Gazeboハードウェアインターフェース（GazeboSystem）の初期化完了を待つため
    # # 待機時間を延長（5秒→7秒）。複数ロボット環境では初期化に時間がかかる。
    # actions.append(
    #     TimerAction(
    #         period=7.0,
    #         actions=[joint_state_broadcaster_spawner, diff_drive_controller_spawner]
    #     )
    # )

    return actions


def generate_launch_description():

    desc_dir = get_package_share_directory('nav2_minimal_tb4_description')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot4',
        description='name of the robot')
    
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='-8.00',
        description='Initial x position of the robot')

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.00',
        description='Initial y position of the robot')

    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.01',
        description='Initial z position of the robot')

    declare_roll_cmd = DeclareLaunchArgument(
        'roll',
        default_value='0.00',
        description='Initial roll of the robot')

    declare_pitch_cmd = DeclareLaunchArgument(
        'pitch',
        default_value='0.00',
        description='Initial pitch of the robot')

    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.00',
        description='Initial yaw of the robot')
    
    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='square15',
        description='Name of the world to spawn the robot in')


    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(desc_dir)).parent.resolve()))
    set_env_vars_tb4_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(get_package_share_directory('tb4_launcher')))
    declare_output_cmd = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Output type of nodes'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='error', description='log level'
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    # ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_world_name_cmd)

    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_tb4_resources)
    ld.add_action(declare_output_cmd)

    ld.add_action(declare_log_level_cmd)

    ld.add_action(OpaqueFunction(function=launch_nodes))
    return ld
