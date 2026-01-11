import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable, AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

HOME = os.path.expanduser('~')
ws   = os.path.join(HOME, 'navigation2_ws') #ディレクトリ構造に合わせて変更して

def launch_setup(context, *args, **kwargs):
   
    width = LaunchConfiguration('width').perform(context)
    height = LaunchConfiguration('height').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    resolution = LaunchConfiguration('resolution').perform(context)

    actions = []

    # Ensure GUI processes receive DISPLAY (useful when launching from ROS2 launch)
    actions.append(SetEnvironmentVariable('DISPLAY', os.environ.get('DISPLAY', '')))

    create_pgm_cmd = [
        'python', '-u', os.path.join(ws, 'scripts', 'interactive_pgm_editor.py'),
        '--width', width,
        '--height', height, 
        '--world_name', world_name,
        '--resolution', resolution,
    ]

    create_pgm_proc = ExecuteProcess(
        cmd=create_pgm_cmd,
        output='screen'
    )


    create_map_yaml_cmd = [
        'python', '-u', os.path.join(ws, 'scripts', 'create_map_yaml.py'),
        '--width', width,
        '--height', height,
        '--world_name', world_name,
        '--resolution', resolution,
    ]

    create_map_yaml_proc = ExecuteProcess(
        cmd=create_map_yaml_cmd,
        output='screen'
    )

    print(f"resolution for world conversion: {resolution} m/px")

    convert_world_cmd = [
        'python', '-u', os.path.join(ws, 'scripts', 'convert_pgm_to_world.py'),
        '--world_name', world_name,
        '--resolution', resolution,
        '--sim_freq', '5000',
        '--step_size', '0.01',
    ]

    convert_world_proc = ExecuteProcess(
        cmd=convert_world_cmd,
        output='screen'
    )

    # Start the interactive PGM editor immediately
    actions.append(create_pgm_proc)

    # After the editor exits, generate the map YAML then convert the PGM to a world
    reg = RegisterEventHandler(
        OnProcessExit(
            target_action=create_pgm_proc,
            on_exit=[
                create_map_yaml_proc,
                TimerAction(
                    period=1.0,
                    actions=[convert_world_proc],
                )
            ],
        )
    )

    actions.append(reg)

    return actions


def generate_launch_description():
    declare_width_cmd = DeclareLaunchArgument('width', default_value='100')
    declare_height_cmd = DeclareLaunchArgument('height', default_value='100')
    declare_world_name_cmd = DeclareLaunchArgument('world_name', default_value='square15')
    declare_resolution_cmd = DeclareLaunchArgument('resolution', default_value='0.05')

    ld = LaunchDescription()
    ld.add_action(declare_width_cmd)
    ld.add_action(declare_height_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_resolution_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
