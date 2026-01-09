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
from launch.event_handlers import OnProcessStart

HOME = os.path.expanduser('~')
ws   = os.path.join(HOME, 'nav2_ws') #ディレクトリ構造に合わせて変更して

def launch_setup(context, *args, **kwargs):
    robot_count = int(LaunchConfiguration('robot_count').perform(context))
    world_name = LaunchConfiguration('world_name').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    use_rl = LaunchConfiguration('use_rl').perform(context)
    rl_algorithm = LaunchConfiguration('rl_algorithm').perform(context)
    timesteps = LaunchConfiguration('timesteps').perform(context)
    output = LaunchConfiguration('output').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    sim_freq = LaunchConfiguration('sim_freq').perform(context)
    step_size = LaunchConfiguration('step_size').perform(context)
    load_model = LaunchConfiguration('load_model').perform(context)
    eval = LaunchConfiguration('eval').perform(context)
    eval_episodes = LaunchConfiguration('eval_episodes').perform(context)
    rebuild_world = LaunchConfiguration('rebuild_world').perform(context)

    # DDS/環境変数設定
    actions = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', f"file://{os.path.expanduser('~')}/nav2_ws/cyclonedds_config.xml"),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('GZ_SIM_LOG_LEVEL', 'error'),
        SetEnvironmentVariable('GZ_LOG_LEVEL', 'error'),
    ]

    actions.append(
        AppendEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH', 
            '/opt/ros/jazzy/lib'
        )
    )

    set_env_vars_sdf_log = AppendEnvironmentVariable(
            'SDF_VERBOSE_LEVEL', '1')

    actions.append(set_env_vars_sdf_log)

    # PGMファイルからワールドファイルを生成&ビルド
    world_create_cmd = [
        'python', '-u', os.path.join(ws, 'scripts', 'convert_pgm_to_world.py'),
        '--world_name', f'{world_name}',
        '--sim_freq', sim_freq,
        '--step_size', step_size,
        "&&",
        "colcon", "build", "--packages-select", "world_xacro_creator",
        "&&",
        "source", "install/setup.bash"
    ]
    world_create_cmd_str = ' '.join(world_create_cmd)
    world_create_proc = ExecuteProcess(cmd=['bash', '-lc', world_create_cmd_str], cwd=ws, output=output)
    if rebuild_world.lower() == 'true':
        actions.append(world_create_proc)

    # tb4_launcherのmulti_launch.pyを呼び出し
    tb4_launcher_path = os.path.join(get_package_share_directory('tb4_launcher'), 'launch', 'multi_launch.py')
    nav2_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb4_launcher_path),
            launch_arguments={
                'robot_count': str(robot_count),
                'world_name': world_name,
                'headless': headless,
                'use_rviz': use_rviz,
                'output': output,
                'log_level': log_level,
            }.items(),
        )
    
    cmd = [
        'python', '-u', os.path.join(ws, 'scripts', 'pettingZoo_test.py'),
        '--robot_count', str(robot_count),
        '--world_name', world_name,
        '--rl_algorithm', rl_algorithm,
        '--timesteps', timesteps,
        '--load_model', load_model,
        '--eval_episodes', eval_episodes,
    ]

    if eval.lower() == 'true':
        cmd.append('--eval')

    if use_rl.lower() == 'true':
        cmd.append('--use_rl')
    
    
    env_node = TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(cmd=cmd, output=output)
            ]
        )

    actions.append(nav2_launcher)
    actions.append(env_node)
    
    return actions


def generate_launch_description():
    declare_robot_count_cmd = DeclareLaunchArgument('robot_count', default_value='20')
    declare_world_name_cmd = DeclareLaunchArgument('world_name', default_value='square15')
    declare_headless_cmd = DeclareLaunchArgument('headless', default_value='False')
    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value='False')
    declare_use_rl_cmd = DeclareLaunchArgument('use_rl', default_value='False')
    declare_algorithm_cmd = DeclareLaunchArgument('rl_algorithm', default_value='SAC')
    declare_timesteps_cmd = DeclareLaunchArgument('timesteps', default_value='10000000')
    declare_output_cmd = DeclareLaunchArgument('output', default_value='log')
    declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='error', description='log level')
    declare_sim_freq_cmd = DeclareLaunchArgument('sim_freq', default_value='2000')
    declare_step_size_cmd = DeclareLaunchArgument('step_size', default_value='0.005')
    declare_load_model_cmd = DeclareLaunchArgument('load_model', default_value='')
    declare_eval_cmd = DeclareLaunchArgument('eval', default_value='False')
    declare_eval_episodes_cmd = DeclareLaunchArgument('eval_episodes', default_value='100')
    declare_rebuild_world_cmd = DeclareLaunchArgument('rebuild_world', default_value='False')

    ld = LaunchDescription()
    ld.add_action(declare_robot_count_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_rl_cmd)
    ld.add_action(declare_algorithm_cmd)
    ld.add_action(declare_timesteps_cmd)
    ld.add_action(declare_output_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_sim_freq_cmd)
    ld.add_action(declare_step_size_cmd)
    ld.add_action(declare_load_model_cmd)
    ld.add_action(declare_eval_cmd)
    ld.add_action(declare_eval_episodes_cmd)
    ld.add_action(declare_rebuild_world_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
