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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os
import tempfile
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription



# from test.src.gazebo.launch.launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def add_actions(context, *args, **kwargs):


    tb4_launcher_dir = get_package_share_directory('tb4_launcher')
    # This checks that tb4 exists needed for the URDF / simulation files.
    # If not using TB4, its safe to remove.
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    # namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    ###ロボットの数
    robot_count = LaunchConfiguration('robot_count')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    world_name = LaunchConfiguration('world_name').perform(context)
    world_pkg_dir = get_package_share_directory('world_xacro_creator')
    map_yaml_path = os.path.join(world_pkg_dir, 'maps', f'{world_name}.yaml')
    world_xacro_dir = os.path.join(world_pkg_dir, 'worlds', f'{world_name}.sdf.xacro')

    output = LaunchConfiguration('output')
    log_level = LaunchConfiguration('log_level')

    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    
    # robot_sdf launch arg points to the xacro/urdf source. We'll generate
    # a per-robot SDF (via xacro -> gz sdf -p) and pass the generated SDF path
    # to the spawn launch. Keep the original LaunchConfiguration available
    # for robot_state_publisher which expects a URDF/xacro input.
    robot_xacro = LaunchConfiguration('robot_xacro')
    # resolved xacro path (string) so we can run conversion here
    try:
        robot_xacro_path = LaunchConfiguration('robot_xacro').perform(context)
    except Exception:
        robot_xacro_path = None

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world_xacro_dir])
    
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '0', world_sdf],
        output=output,
        condition=IfCondition(use_simulator),
    )

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(sim_dir, 'worlds'))
    set_env_vars_tb4_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(tb4_launcher_dir))
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v 0 -g ']}.items(),
    )

    #クロックのブリッジはシミュレーション全体で一つのみでいいのでここでブリッジ起動
    bridge_topics = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]
    
    # サービスブリッジ（Gazebo → ROS 2）
    # set_poseサービスはワールド全体で1つなので、ここで1回だけブリッジ
    # サービスブリッジ（Gazebo <-> ROS 2）
    bridge_services = [
        # Gazebo側の "/world/<world>/create" を ROS側の "ros_gz_interfaces/srv/SpawnEntity" 型に変換
        f'/world/{world_name}/create@ros_gz_interfaces/srv/SpawnEntity',
        # Gazebo側の "/world/<world>/remove" を ROS側の "ros_gz_interfaces/srv/DeleteEntity" 型に変換
        f'/world/{world_name}/remove@ros_gz_interfaces/srv/DeleteEntity',
    ]
    
    # トピックとサービスを結合
    bridge_args = bridge_topics + bridge_services

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz_clock',
        arguments=bridge_args,
        parameters=[{'use_sim_time': use_sim_time}],
        output=output,
    )

    actions = []

    actions.append(set_env_vars_resources)
    actions.append(set_env_vars_tb4_resources)
    actions.append(world_sdf_xacro)
    actions.append(remove_temp_sdf_file)
    actions.append(gazebo_server)
    actions.append(gazebo_client)

    actions.append(bridge)

    robot_count = int(LaunchConfiguration('robot_count').perform(context))

    for i in range(robot_count):

        max_row_count = 5
        distance_between_robots = 1.0  # meters

        # 【最適化3】ロボット座標を3次元で分散配置
        # X軸のみでなく、Y軸も変化させて衝突を回避
        # グリッド配置: 1行に10台まで、次の行に進む
        row = i // max_row_count  # 行番号 (0, 1, 2, ...)
        col = i % max_row_count   # 列番号 (0-2)

        offset = {'x': 10, 'y': 0.0}
        
        x = col * distance_between_robots  # X方向に3m間隔（10ロボットごとに列を変える）
        y = row * distance_between_robots  # Y方向に3m間隔（10台ごとに行を変える）

        pose = {
            'x': str(offset['x'] + x),
            'y': str(offset['y'] + y),
            'z': '0.01',  # 全ロボット同じ高さから落下
            'R': '0.00',
            'P': '0.00',
            'Y': '0.00',
        }

        namespace_i = f'robot_{i+1}'
        # test_bitmask = f'0x{format(pow(2,i), "04x")}'  # 'ffff'
        test_bitmask = f'0xffff'  # 'ffff'


        urdf = Command(['xacro', ' ', robot_xacro, ' ', f'namespace:={namespace_i}', ' ', f'bitmask:={test_bitmask}', ' ', f'controllers_file:={controllers_file}'])

        # Evaluate the Command substitution now and dump the expanded URDF/xacro output
        # to a temporary file for inspection/debugging. This avoids having to run
        # the xacro command manually and makes the concrete robot_description
        # available as a file. Failure to evaluate is non-fatal — we fall back to
        # leaving `urdf` as a substitution for robot_state_publisher.
        try:
            urdf_expanded = urdf.perform(context)
            # Persist expanded URDF in workspace so it won't be removed automatically.
            dump_dir = os.path.join(os.getcwd(), 'urdf_dumps')
            try:
                os.makedirs(dump_dir, exist_ok=True)
            except Exception:
                # ignore mkdir failures, we'll try to write in cwd as fallback
                dump_dir = os.path.join(os.getcwd(), 'urdf_dumps')

            urdf_dump_path = os.path.join(dump_dir, f'{namespace_i}.urdf')
            try:
                with open(urdf_dump_path, 'w') as _f:
                    _f.write(urdf_expanded)
                # Inform where the file was written so the user can inspect it later
                # print(f"[multi_launch] Wrote expanded URDF for {namespace_i} -> {urdf_dump_path}")
            except Exception as e:
                print(f"[multi_launch] Failed to write expanded URDF for {namespace_i}: {e}")
        except Exception as e:
            # Most likely the substitution couldn't be evaluated in this context
            # (e.g., missing xacro binary, environment). That's OK — robot_state_publisher
            # can still receive the `Command` substitution and expand it at runtime.
            print(f"[multi_launch] Could not evaluate urdf Command for {namespace_i}: {e}")
        

        ###ロボット状態配信ノードの作成
        start_robot_state_publisher_cmd = Node(
            condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace_i,
            output=output,
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': urdf,
                    'frame_prefix': f'{namespace_i}/',
                    # 'publish_fixed_frames': True,
                }
            ],
            remappings=remappings,
        )

        ###各ロボット用のrviz起動コマンドの作成
        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb4_launcher_dir, "launch", 'rviz_launch.py')),
            condition=IfCondition(use_rviz),
            launch_arguments={
                'namespace': namespace_i,
                'use_namespace': use_namespace,
                'use_sim_time': use_sim_time,
                'rviz_config': rviz_config_file,
            }.items(),
        )

        ###各ロボット用のnav2スタックランチコマンドの作成
        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb4_launcher_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace_i,
                'use_namespace': use_namespace,
                'slam': slam,
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'log_level': log_level,
                'output': output,
            }.items(),
        )

        # static_tf = Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_basefootprint_to_baselink',
        #     namespace=namespace_i,   # optional: frames を絶対パスで指定するなら namespace は不要
        #     arguments=[
        #         '0', '0', '0', '0', '0', '0', '1',
        #         f'/{namespace_i}/base_footprint', f'/{namespace_i}/base_link'
        #     ],
        #     output='screen',
        # )
        
        # actions.append(static_tf)

        # Generate per-robot SDF from the xacro (so spawned model uses SDF)
        # We'll perform the conversion here (during launch generation) so the
        # spawn launch receives a concrete SDF file path.
        sdf_dump_dir = os.path.join(os.getcwd(), 'sdf_dumps')
        os.makedirs(sdf_dump_dir, exist_ok=True)
        sdf_dump_path = os.path.join(sdf_dump_dir, f'{namespace_i}.sdf')
        robot_sdf_for_spawn = None
        if robot_xacro_path:
            try:
                # Create a temporary URDF from xacro first to avoid piping issues
                tmp_robot_urdf = tempfile.mktemp(prefix=f'{namespace_i}_', suffix='.urdf')
                # xacro -> temporary URDF file with all necessary parameters
                cmd_xacro = f"xacro {robot_xacro_path} namespace:={namespace_i} bitmask:={test_bitmask} -o {tmp_robot_urdf}"
                subprocess.check_call(cmd_xacro, shell=True)
                # gz sdf reads the URDF file and converts to SDF, output directly to dump path
                cmd_gzsdf = f"gz sdf -p {tmp_robot_urdf} > {sdf_dump_path}"
                subprocess.check_call(cmd_gzsdf, shell=True)
                # print(f"[multi_launch] Wrote expanded SDF for {namespace_i} -> {sdf_dump_path}")
                
                # Use the dump path for spawning
                robot_sdf_for_spawn = sdf_dump_path
                
                # remove the intermediate URDF right away
                try:
                    os.remove(tmp_robot_urdf)
                except Exception:
                    pass
            except subprocess.CalledProcessError as e:
                # Fall back to passing the original xacro path so at least
                # robot_state_publisher continues to work; warn in logs.
                # print(f"[multi_launch] Failed to generate SDF for {namespace_i}: {e}")
                # attempt to remove any intermediate files
                try:
                    if 'tmp_robot_urdf' in locals() and os.path.exists(tmp_robot_urdf):
                        os.remove(tmp_robot_urdf)
                except Exception:
                    pass
                robot_sdf_for_spawn = None

        gz_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb4_launcher_dir, 'launch', 'spawn_tb4.launch.py')),
                launch_arguments={
                    'namespace': namespace_i,
                    'use_simulator': use_simulator,
                    'use_sim_time': use_sim_time,
                    'robot_name': namespace_i,
                    # if conversion succeeded, pass the generated SDF path,
                    # otherwise pass the original robot_sdf (xacro path)
                    'robot_sdf': robot_sdf_for_spawn if robot_sdf_for_spawn else robot_xacro,
                    'x_pose': pose['x'],
                    'y_pose': pose['y'],
                    'z_pose': pose['z'],
                    'roll': pose['R'],
                    'pitch': pose['P'],
                    'yaw': pose['Y']
                }.items()
        )
        
        # robot_state_publisherは即座に起動（軽量）
        actions.append(start_robot_state_publisher_cmd)

        
        delay_time = 0.5  # float型に変更（TimerActionはfloatを要求）
        
        # 【最適化1】RViz2とNav2スタックも段階的に起動
        # 全ロボットが同時にRViz2/Nav2を起動するとGPU/CPU/メモリが枯渇
        # ロボット番号に応じて起動を遅延させる
        component_delay = float(delay_time * i)  # 5秒間隔でRViz2/Nav2を起動（float型）
        
        actions.append(
            TimerAction(
                period=component_delay,
                actions=[rviz_cmd, bringup_cmd]
            )
        )
        
        # 【最適化2】Gazeboスポーンをさらに遅延（Nav2初期化を待つ）
        # Nav2の初期化完了を待ってからスポーンすることで、
        # 物理エンジンとNav2の競合を回避
        spawn_delay = float(delay_time * (i+1))  
        actions.append(
            TimerAction(
                period=spawn_delay,
                actions=[gz_robot]
            )
        )

    return actions


def generate_launch_description():

    tb4_launcher_dir = get_package_share_directory('tb4_launcher')

    ld = LaunchDescription()
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(tb4_launcher_dir, 'params', 'nav2_malti_params.yaml'),
        # default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_robot_count_cmd = DeclareLaunchArgument(
        'robot_count',
        default_value='2',
        description='Number of robots to spawn',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(tb4_launcher_dir, 'rviz', 'nav2_multi_view.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator',
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='False', description='Whether to start RVIZ'
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Whether to execute gzclient)'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='nav2_turtlebot4', description='name of the robot'
    )

    declare_robot_xacro_cmd = DeclareLaunchArgument(
        'robot_xacro',
        default_value=os.path.join(tb4_launcher_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro'),
        description='Full path to robot xacro file to spawn the robot in gazebo',
    )
    declare_output_cmd = DeclareLaunchArgument(
        'output',
        default_value='log',
        description='Output for nodes launched by this launch file. screen or log',
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='error', description='log level'
    )
    declare_controllers_file_cmd = DeclareLaunchArgument(
        'controllers_file',
        default_value=os.path.join(tb4_launcher_dir, 'config', 'diff_drive_controller.yaml'),
        description='Full path to the controller configuration file to use',
    )
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_robot_count_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_xacro_cmd)
    ld.add_action(declare_output_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_controllers_file_cmd)

    ld.add_action(OpaqueFunction(function=add_actions))

    return ld
