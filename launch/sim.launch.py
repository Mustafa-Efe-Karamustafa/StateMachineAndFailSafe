# =============================================================================
# DOSYA : launch/sim.launch.py
# AMAÇ  : Tek komutla her şeyi başlatır: (1) Gazebo + havuz dünyası, (2) ROS↔Gazebo köprüsü
#         (saat, odometri, 6 itici komutu), (3) motion_node (gains.yaml kazançlarıyla).
# AKIŞ  : motion_node → /model/auv/joint/<itici>/cmd_thrust → köprü → Gazebo itici eklentisi → araç hareketi
#         araç pozu → Gazebo odometri eklentisi → /model/auv/odometry → köprü → motion_node
# KULLANIM: ros2 launch sualti_sim_son sim.launch.py mission:=square
# =============================================================================
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# İtici eklem adları (pool.sdf içindeki <joint name=...> ile aynı).
THRUSTERS = ['thruster_fr', 'thruster_fl', 'thruster_rl', 'thruster_rr',
             'thruster_vl', 'thruster_vr']


# _setup: launch argümanları çözüldükten sonra çalışır (OpaqueFunction), süreçlerin listesini döner.
def _setup(context):
    pkg = get_package_share_directory('sualti_sim_son')
    arg = lambda n: LaunchConfiguration(n).perform(context)

    # Gazebo sürümü: 'gz' komutu varsa Harmonic, yoksa Fortress ('ign').
    flavor = arg('gz')
    if flavor == 'auto':
        flavor = 'harmonic' if shutil.which('gz') else 'fortress'

    world = os.path.join(pkg, 'worlds', 'pool.sdf')
    if flavor == 'fortress':
        # Fortress eklenti adları farklı (ignition-gazebo-*): dünya dosyasını çalışırken dönüştür.
        with open(world) as f:
            text = f.read()
        text = text.replace('gz-sim-', 'ignition-gazebo-').replace('gz::sim::', 'ignition::gazebo::')
        world = '/tmp/sualti_pool_fortress.sdf'
        with open(world, 'w') as f:
            f.write(text)
        gz_cmd, msgs = ['ign', 'gazebo'], 'ignition.msgs'
    else:
        gz_cmd, msgs = ['gz', 'sim'], 'gz.msgs'

    # -r: hemen çalıştır;  -s: arayüzsüz (headless).
    gz_cmd += ['-r'] + (['-s'] if arg('headless') == 'true' else []) + [world]

    # Köprü söz dizimi  konu@ROS_tipi<yön>GZ_tipi :  '[' Gazebo→ROS (saat, odometri),  ']' ROS→Gazebo (itki komutları).
    bridge_args = [f'/clock@rosgraph_msgs/msg/Clock[{msgs}.Clock',
                   f'/model/auv/odometry@nav_msgs/msg/Odometry[{msgs}.Odometry']
    bridge_args += [f'/model/auv/joint/{j}/cmd_thrust@std_msgs/msg/Float64]{msgs}.Double'
                    for j in THRUSTERS]

    return [
        ExecuteProcess(cmd=gz_cmd, output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=bridge_args, output='screen'),
        # use_sim_time: düğüm Gazebo saatini (/clock) kullanır → dt simülasyon süresiyle hesaplanır.
        Node(package='sualti_sim_son', executable='motion_node', output='screen',
             parameters=[os.path.join(pkg, 'config', 'gains.yaml'),
                         {'use_sim_time': True,
                          'mission': arg('mission'),
                          'thrust_sign': float(arg('thrust_sign'))}]),
    ]


# generate_launch_description: komut satırı argümanlarını tanımlar (mission, gz, headless, thrust_sign).
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='line',
                              description='line | square | circle | manual'),
        DeclareLaunchArgument('gz', default_value='auto',
                              description='auto | harmonic | fortress'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('thrust_sign', default_value='1.0',
                              description='Araç geri giderse -1.0 yapın'),
        OpaqueFunction(function=_setup),
    ])
