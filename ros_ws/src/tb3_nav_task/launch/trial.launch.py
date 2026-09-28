"""Complete Task 02 system: Gazebo world + TurtleBot3 Burger + bridges + navigator + monitor.

Demo (GUI):
    ros2 launch tb3_nav_task trial.launch.py world:=u_trap strategy:=bug2 gui:=true
Headless trial (used by scripts/run_experiments.py):
    ros2 launch tb3_nav_task trial.launch.py world:=cluttered strategy:=vfh gui:=false \
        trial_id:=A_cluttered_vfh_v0 nav_params:='{"v_max": 0.15}'

Arguments
  world        open_field | cluttered | u_trap        (worlds/<world>.sdf, tb3_nav_task/worlds.py)
  strategy     vfh | bug2
  gui          true | false   (false = gz server only, faster, for batch experiments)
  start_x/start_y/start_yaw   spawn pose (default: the world's start pose)
  nav_params   JSON dict of planner overrides, e.g. {"v_max":0.22,"safety":0.2}
  monitor      true | false   (evaluation node; ends the launch when the trial finishes)
  navigator    true | false   (false = spawn only, e.g. to inspect sensors or drive by teleop)
  trial_id, plan, variant, results_dir, timeout   bookkeeping for trial_monitor
  gl_mode      OpenGL workaround for the UTM VM (virgl driver reports only OpenGL 2.1, but
               Gazebo's ogre2 renderer - needed by the GPU LiDAR - requires >= 3.3):
                 software (default) Mesa software rasteriser (LIBGL_ALWAYS_SOFTWARE=1, kms_swrast).
                          Validated: LiDAR bias -0.4 mm, MAE 11 mm vs ray-cast ground truth.
                 virgl    hardware virgl + MESA_GL_VERSION_OVERRIDE=4.3: runs, but the LiDAR
                          returns wrong ranges (~0.13 m everywhere) -> NOT usable
                 native   no overrides (for a real Linux PC with a proper GPU driver)
               See scripts/gltest2.sh and scripts/probe_sensors.py.
  render_engine  GUI render engine (default ogre2)
"""
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    'world': 'open_field', 'strategy': 'vfh', 'gui': 'true',
    'start_x': 'nan', 'start_y': 'nan', 'start_yaw': 'nan', 'nav_params': '{}',
    'monitor': 'true', 'trial_id': 'demo', 'plan': 'demo', 'variant': '0',
    'results_dir': os.path.expanduser('~/vm-share/task02/results'), 'timeout': '200.0',
    'verbose': '1', 'gl_mode': 'software', 'render_engine': 'ogre2', 'navigator': 'true',
    'hide_lidar_visual': 'true',
}


GT_PLUGIN = """
<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
  <publish_link_pose>false</publish_link_pose>
  <publish_model_pose>true</publish_model_pose>
  <publish_nested_model_pose>false</publish_nested_model_pose>
  <use_pose_vector_msg>false</use_pose_vector_msg>
  <static_publisher>false</static_publisher>
  <update_frequency>50</update_frequency>
</plugin>"""


def _prepare_model(src, hide_lidar_visual=True):
    """Copy the stock TurtleBot3 Burger model.sdf with two simulation-only changes:
      1. add Gazebo's PosePublisher plugin -> /model/burger/pose (ground truth for evaluation;
         the dynamic_pose/info -> TFMessage bridge drops entity names, so it cannot identify the robot)
      2. remove the *visual* mesh of the base_scan (LiDAR housing) link. With the ogre1 renderer the
         GPU LiDAR rays start inside that mesh and every beam returns range_min (0.12 m). Collision
         geometry, sensor pose and all physical properties are unchanged.
    """
    import xml.etree.ElementTree as ET
    tree = ET.parse(src)
    model = tree.getroot().find('model')
    if hide_lidar_visual:
        for link in model.findall('link'):
            if link.get('name') == 'base_scan':
                for vis in link.findall('visual'):
                    link.remove(vis)
    model.append(ET.fromstring(GT_PLUGIN))
    out = '/tmp/tb3_nav_task_burger.sdf'
    tree.write(out)
    return out


def _setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    os.environ['TURTLEBOT3_MODEL'] = 'burger'  # read by turtlebot3 launch files at include time

    from tb3_nav_task.worlds import WORLDS
    world = WORLDS[a['world']]
    sx = world.start[0] if a['start_x'] == 'nan' else float(a['start_x'])
    sy = world.start[1] if a['start_y'] == 'nan' else float(a['start_y'])
    syaw = world.start[2] if a['start_yaw'] == 'nan' else float(a['start_yaw'])
    nav_params = json.loads(a['nav_params'] or '{}')
    gui = a['gui'].lower() == 'true'

    pkg = get_package_share_directory('tb3_nav_task')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_sdf = os.path.join(pkg, 'worlds', f'{world.name}.sdf')
    model_sdf = _prepare_model(os.path.join(tb3, 'models', 'turtlebot3_burger', 'model.sdf'),
                               a['hide_lidar_visual'].lower() == 'true')
    bridge_yaml = os.path.join(tb3, 'params', 'turtlebot3_burger_bridge.yaml')
    gz_launch = os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    v = a['verbose']

    gl_env = {
        'virgl': {'MESA_GL_VERSION_OVERRIDE': '4.3', 'MESA_GLSL_VERSION_OVERRIDE': '430'},
        'software': {'LIBGL_ALWAYS_SOFTWARE': '1', 'MESA_LOADER_DRIVER_OVERRIDE': 'kms_swrast'},
        'native': {},
    }[a['gl_mode']]
    actions = [SetEnvironmentVariable(k, val) for k, val in gl_env.items()]
    actions += [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               os.path.join(tb3, 'models') + os.pathsep +
                               os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        # Gazebo server (physics + sensors). -r: run immediately.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch), launch_arguments={
            'gz_args': f'-r -s -v{v} {world_sdf}', 'on_exit_shutdown': 'true'}.items()),
    ]
    if gui:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                       launch_arguments={'gz_args': f'-g -v{v} --render-engine-gui {a["render_engine"]}',
                                         'on_exit_shutdown': 'true'}.items()))
    actions += [
        # Spawn the Burger at the requested pose (-Y = yaw).
        Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
            '-name', 'burger', '-file', model_sdf, '-x', str(sx), '-y', str(sy), '-z', '0.01',
            '-Y', str(syaw)]),
        # Standard TurtleBot3 bridges: /clock /odom /tf /scan /imu /joint_states <- gz, /cmd_vel -> gz
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='tb3_bridge',
             output='screen', arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}']),
        # Ground-truth world pose of the robot, for EVALUATION ONLY (never used by the navigator).
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ground_truth_bridge',
             output='screen',
             arguments=['/model/burger/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
             remappings=[('/model/burger/pose', '/ground_truth/pose')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
    ]

    nav_ros_params = {'use_sim_time': True, 'strategy': a['strategy'],
                      'goal_x': float(world.goal[0]), 'goal_y': float(world.goal[1]),
                      'start_x': float(sx), 'start_y': float(sy), 'start_yaw': float(syaw)}
    nav_ros_params.update({k: (float(val) if isinstance(val, (int, float)) else str(val))
                           for k, val in nav_params.items()})
    navigator = Node(package='tb3_nav_task', executable='navigator', output='screen',
                     parameters=[nav_ros_params])
    if a['navigator'].lower() == 'true':
        actions.append(TimerAction(period=3.0, actions=[navigator]))

    if a['monitor'].lower() == 'true':
        monitor = Node(package='tb3_nav_task', executable='trial_monitor', output='screen',
                       parameters=[{
                           'use_sim_time': True, 'world': world.name, 'trial_id': a['trial_id'],
                           'plan': a['plan'], 'strategy': a['strategy'], 'variant': int(a['variant']),
                           'params': json.dumps(nav_params, sort_keys=True),
                           'results_dir': a['results_dir'], 'timeout': float(a['timeout']),
                           'start_x': float(sx), 'start_y': float(sy), 'start_yaw': float(syaw)}])
        actions.append(monitor)
        if not gui:  # batch mode: finishing the evaluation ends the whole launch
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=monitor, on_exit=[EmitEvent(event=Shutdown(reason='trial finished'))])))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] +
        [OpaqueFunction(function=_setup)])
