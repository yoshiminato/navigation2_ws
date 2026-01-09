import numpy as np
from PIL import Image
import argparse
from ament_index_python.packages import get_package_share_directory

# 長方形分割のためのアルゴリズムを実装した関数をインポート
from rec_partition import rec_partition

pkg_dir = get_package_share_directory('world_xacro_creator')

parser = argparse.ArgumentParser()
# parser.add_argument('--input', type=str, required=True, help='入力ファイル名')
# parser.add_argument('--output', type=str, default='', help='出力ファイル名')
parser.add_argument('--world_name', type=str, default='map', help='ワールド名（拡張子なし）')
parser.add_argument('--input_dir', type=str, default=pkg_dir + '/maps/', help='入力ディレクトリ')
parser.add_argument('--output_dir', type=str, default=pkg_dir + '/worlds/', help='出力ディレクトリ')
parser.add_argument('--sim_freq', type=float, default=3000, help='シミュレーション周波数 [Hz]')
parser.add_argument('--step_size', type=float, default=0.005, help='リアルタイム更新レート [Hz]')
parser.add_argument('--resolution', type=float, default=0.05, help='マップの解像度（メートル/ピクセル）')
args = parser.parse_args()

print(f"resolution: {args.resolution} m/px")

input_filepath  = args.input_dir + args.world_name + '.pgm'
output_filepath = args.output_dir + args.world_name + '.sdf.xacro'
image = Image.open(input_filepath).convert("L")
image = np.array(image, dtype=np.uint8)

# np.savetxt('output_image.txt', image, fmt='%d')

resolution = args.resolution  # [m/px]: x [m/px] means x meter for 1 px.
width, height = image.shape

# 各種プラグインの記述を追加&照明と地面のモデルをベタ書きに変更
world_template_list = [f"""<?xml version="1.0" ?>
<sdf version="1.6" xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:arg name="headless" default="true"/>
  <world name="{args.world_name}">
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <physics type="ode">
      <max_step_size>{args.step_size}</max_step_size>
      <real_time_update_rate>{args.sim_freq}</real_time_update_rate>
    </physics>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <xacro:unless value="$(arg headless)">
      <plugin
        filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster">
      </plugin>
    </xacro:unless>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="gz-sim-contact-system"
      name="gz::sim::systems::Contact">
    </plugin>
    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>

                                     
    <light name='sun' type='directional'>
        <cast_shadows>0</cast_shadows>
        <pose>0 0 10 0 0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.8 0.8 0.8 1</specular>
        <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
        </attenuation>
        <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='ground_plane'>
        <static>1</static>
        <link name='link'>
            <collision name='ground'>
                <geometry>
                    <plane>
                        <normal>0 0 1</normal>
                        <size>100 100</size>
                    </plane>
                </geometry>
                <max_contacts>10</max_contacts>
            </collision>
            <visual name='visual'>
                <geometry>
                    <plane>
                        <normal>0 0 1</normal>
                        <size>100 100</size>
                    </plane>
                </geometry>
                <material>
                    <ambient>0.8 0.8 0.8 1</ambient>
                    <diffuse>0.8 0.8 0.8 1</diffuse>
                    <specular>0.8 0.8 0.8 1</specular>
                </material>
            </visual>
        </link>
    </model>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>0 0 10 0 1.570796 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
"""]

wall_height = 1.0

# 長方形分割
rects = rec_partition(image, thr=128)

for idx, rect in enumerate(rects):
    x, y, w, h = rect
    wall_x = (((2 * y + h) / 2) - height / 2) * resolution
    wall_y = (width / 2 - ((2 * x + w) / 2)) * resolution
    wall_length_x = h * resolution
    wall_length_y = w * resolution

    world_template_list.append(
f"""
<model name="wall_{idx}">
  <static>true</static>
  <pose>{wall_x} {wall_y} {wall_height/2} 0 0 0</pose>
  <link name="link">
    <collision name="wall">
      <geometry>
        <box>
          <size>{wall_length_x} {wall_length_y} {wall_height}</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>{wall_length_x} {wall_length_y} {wall_height}</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
""")

# Footer part of .world file
world_template_list.append("""
  </world>
</sdf>
""")

# Save as .world file
with open(output_filepath, "w") as f:
    f.write("".join(world_template_list))

print(output_filepath + " has been created!!")