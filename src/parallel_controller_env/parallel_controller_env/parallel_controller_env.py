from pettingzoo import ParallelEnv
from gymnasium.spaces import Discrete, MultiDiscrete, Box, Tuple



from sensor_msgs.msg import Imu  # IMUメッセージ型をインポート
import numpy as np

from collections import deque

from builtin_interfaces.msg import Time

import rclpy
from rclpy.node import Node
import functools
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from rclpy.action import ActionClient
from ros_gz_interfaces.msg import Contacts
# from gazebo_msgs.srv import SetEntityState
# from gazebo_msgs.msg import EntityState
from ros_gz_interfaces.srv import SetEntityPose
from action_msgs.srv import CancelGoal
from std_srvs.srv import Empty
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalInfo
from unique_identifier_msgs.msg import UUID
import numpy as np
import math
import matplotlib
matplotlib.use('TkAgg')  # GUIバックエンドを明示的に指定
import matplotlib.pyplot as plt
plt.ion()  # インタラクティブモード有効化
import threading
import subprocess

import cv2

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from ros_gz_interfaces.msg import Entity
from rosgraph_msgs.msg import Clock

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from .utils import quaternion_from_yaw, quaternion_to_yaw, get_unique_log_file_path, sdf_dir, Pose

import time as _time

from tf2_ros.buffer import Buffer
from .my_transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs

from nav2_msgs.srv import ClearEntireCostmap

import random
import time

from enum import Enum

from rclpy.parameter import Parameter


"""タイムアウト時間"""
SIMULATION_TIMEOUT = 50  # [秒] エピソードの最大実行時間
SPAWN_TIMEOUT = 10       # [秒] スポーンの最大待機時間

CONTACT_DETECTION_START_DELAY = 0.4

STEP_FREQ = 10      # [Hz] step関数の実行周期

MAX_POINTS = 5     # globalPlanの点数
DUMMY_VAL  = 0     # ダミー値

"""位置推定誤差の許容値"""
POSITION_TOLERANCE = 0.005  # [m] ゴール到達判定用のAMCL位置誤差許容値
YAW_TOLERANCE = 0.002      # [rad] ゴール到達判定用のAMCL姿勢誤差許容値


"""報酬"""
REWARD_COLLISION           = -200.0   # 衝突時のペナルティ
REWARD_GOAL                =  200.0   # ゴール到達時の報酬
REWARD_STEP                =  -0.2       # ステップ毎のペナルティ(時間経過ペナルティ)
REWARD_SUBGOAL_COEF        =  40.0     # サブゴール距離差分報酬係数(自律移動の総経路帳に応じて修正スべき？)
REWARD_HIGH_SPEED_COEF     =  0.0    # 高速移動報酬係数
REWARD_BACKWARD            =  -0.1     # 後退ペナルティ

HIGH_SPEED_DOMAIN          = {'min': 0.35, 'max': 0.5}  # 高速移動とみなす速度域 [m/s]

TARGET_PLAN_POINT_IDX = 2            # 目標とするglobalPlanの点のインデックス


MAX_LINEAR_VEL = 0.5  # 最大並進速度 [m/s]
MIN_LINEAR_VEL = -0.35  # 最小並進速度 [m/s]
MAX_ANGULAR_VEL = 1.9  # 最大回転速度 [rad/s]

COSTMAP_SIZE = 30  # コストマップの一辺のサイズ
COSTMAP_BUFFER_SIZE = 4  # コストマップのバッファサイズ（過去何フレーム分保持するか）


"""コントローラの状態定義"""
class ControllerState(Enum):
    COLLIDED = 1 # 衝突
    TIMEOUT  = 2 # タイムアウト
    ACHIEVED = 3 # ゴール到達
    RUNNING  = 4 # 実行中
    WAITING  = 5 # 待機中
    NONE     = 6 # 未定義

class ERRORCODE:
    NONE=0
    UNKNOWN=200
    INVALID_PLANNER=201
    TF_ERROR=202
    START_OUTSIDE_MAP=203
    GOAL_OUTSIDE_MAP=204
    START_OCCUPIED=205
    GOAL_OCCUPIED=206
    TIMEOUT=207
    NO_VALID_PATH=208


"""Twist(ロボットの制御入力)コマンドの作成"""
#前進(高速)
cmd_forward = Twist()
cmd_forward.linear.x = 0.5
cmd_forward.angular.z = 0.0
#後退
cmd_backward = Twist()
cmd_backward.linear.x = -0.35
cmd_backward.angular.z = 0.0
#左回転(高速)
cmd_left_fast = Twist()
cmd_left_fast.linear.x = 0.4
cmd_left_fast.angular.z = 0.95
#左回転(低速)
cmd_left_slow = Twist()
cmd_left_slow.linear.x = 0.2
cmd_left_slow.angular.z = 1.9
#右回転(高速)
cmd_right_fast = Twist()
cmd_right_fast.linear.x = 0.4
cmd_right_fast.angular.z = -1.9
#右回転(低速)
cmd_right_slow = Twist()
cmd_right_slow.linear.x = 0.2
cmd_right_slow.angular.z = -0.95

pi = 3.14



import os


class Nav2ParallelEnv(ParallelEnv, Node):

    # アクション番号とTwistコマンドの対応表
    action_data = {
        0: cmd_forward,  # 前進(高速)
        1: cmd_backward,      # 後退
        2: cmd_left_fast,     # 左回転(高速)
        3: cmd_left_slow,     # 左回転(低速)
        4: cmd_right_fast,    # 右回転(高速)
        5: cmd_right_slow,    # 右回転(低速)
    }

    def __init__(self, robot_count=2, world_name="square15", use_rl=False, action_type="continuous", render_mode=None):
        Node.__init__(self, 'nav2_parallel_env')

        self.use_obstacbles = True
        self.render_mode = render_mode
        self.robot_count = robot_count
        self.world_name = world_name
        self.use_rl = use_rl

        # 使用する時間をシミュレータ内時間に設定
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        # ランタイムで true にする
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.get_logger().info('__init__')

        time.sleep(1.0)

        self.action_type = action_type
        self.log_path, self.error_log_path = get_unique_log_file_path()

        self.initial_poses = {}
        self.goal_pose_dict = {}
        self._update_all_agents_pose()
        

        self.obstacle_poses = {}
        self._update_all_obstacles_pose()

        # PettingZoo必須属性
        self.metadata = {
            "render_modes": ["human"],
            "name": "nav2_parallel_v0",
            "is_parallelizable": True,
        }


        self.agents = []
        self.possible_agents = [f"robot_{i+1}" for i in range(self.robot_count)]

        self.costmaps_lock = threading.Lock()
        self.vels_lock     = threading.Lock()
        self.ang_vels_lock = threading.Lock()
        self.poses_lock    = threading.Lock()
        self.plan_lock    = threading.Lock()
        self.tfs_lock     = threading.Lock()
        self.accelerations_lock     = threading.Lock()
        self._rclpy_lock  = threading.Lock()

        self.connections = {}
        self.ep_starttime = None # エピソードの開始時刻

        self.sim_time = 0.0  # シミュレーション時間

        # 観測情報を格納する配列の定義
        self.costmaps_buffer = {agt_id: None for agt_id in self.possible_agents}
        self.costmaps        = {agt_id: None for agt_id in self.possible_agents}
        self.vels            = {agt_id: None for agt_id in self.possible_agents}
        self.ang_vels        = {agt_id: None for agt_id in self.possible_agents}
        self.poses           = {agt_id: None for agt_id in self.possible_agents}
        self.plan           = {agt_id: None for agt_id in self.possible_agents}
        self.accelerations   = {agt_id: None for agt_id in self.possible_agents}

        # その他ナビゲーション関係の状態辞書の定義
        self.goal_handle      = {agt_id: None  for agt_id in self.possible_agents}
        self.achieved         = {agt_id: False for agt_id in self.possible_agents}
        self.clear_time       = {agt_id: None  for agt_id in self.possible_agents}
        self.done_estimation  = {agt_id: False for agt_id in self.possible_agents}
        self.latest_tfs       = {agt_id: None  for agt_id in self.possible_agents}
        self.prev_distance    = {agt_id: None  for agt_id in self.possible_agents}
        self.prev_subgoal_pos = {agt_id: None  for agt_id in self.possible_agents}

        # 終了判定用の辞書
        self.terminations = {agt_id: False for agt_id in self.possible_agents}
        self.truncations  = {agt_id: False for agt_id in self.possible_agents}

        # コールバック関数の待機状態を管理する辞書
        self.waiting_timeout_callback = {agt_id: False for agt_id in self.possible_agents}
        self.waiting_crash_callback   = {agt_id: False for agt_id in self.possible_agents}
        self.timeout_rewarded         = {agt_id: False for agt_id in self.possible_agents}
        self.collision_rewarded       = {agt_id: False for agt_id in self.possible_agents}

        # TF関連の初期化
        self.tf_buffers  = {agt_id: Buffer() for agt_id in self.possible_agents}
        self.tf_listener = {agt_id: None     for agt_id in self.possible_agents}
        self.tf_timers   = {agt_id: None     for agt_id in self.possible_agents}

        # 状態の初期化
        self.states      = {agt_id: ControllerState.NONE for agt_id in self.possible_agents}

        # 全ロボット分のwaiting_*_callbackを事前に初期化（contact_callbackが早期に呼ばれる対策）
        self.waiting_crash_callback   = {agt_id: False for agt_id in self.possible_agents}
        self.waiting_timeout_callback = {agt_id: False for agt_id in self.possible_agents}
        self.collision_detected_by_planner = {agt_id: False for agt_id in self.possible_agents}

        # エピソードステップ数
        self.step_count = None
        self.ep_total_reward = {agt_id: None for agt_id in self.agents}

        # matplotlib用の図を事前に作成
        self.figure = plt.figure(f"Costmaps", figsize=(6, 6))
        self.images = {}


        dummy_costmap = np.zeros((COSTMAP_SIZE, COSTMAP_SIZE), dtype=np.int8)
        grid_len = math.ceil(math.sqrt(self.robot_count))

        self.grid_len = grid_len

        for i in range(grid_len):
            for j in range(grid_len):
                id = i * grid_len + (j + 1)
                if id > self.robot_count:
                    break
                agt_id = f"robot_{id}"
                ax = self.figure.add_subplot(grid_len, grid_len, id)
                ax.set_title(f"robot_{id}")
                ax.axis('off')
                self.images[agt_id] = ax.imshow(
                    dummy_costmap, cmap='gray', origin='lower', vmin=0, vmax=100
                )
        
        # 各ロボット用のサービスクライアント辞書を初期化
        self.spawn_client = self.create_client(SpawnEntity, f'/world/{world_name}/create')
        self.delete_client = self.create_client(DeleteEntity, f'/world/{world_name}/remove')

        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self._clock_callback,
            qos_profile_sensor_data,
        )

        # サービスの利用可能確認
        self.get_logger().info('サービス待機中...')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('Spawn service waiting...')
            pass
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('Delete service waiting...')
            pass
        self.get_logger().info('サービス接続完了')

        
        
        for agt_id in self.possible_agents:

            # TFリスナーの作成
            self.tf_listener[agt_id] = TransformListener(
                self.tf_buffers[agt_id],
                self,
                ns=agt_id,
            )

            # コールバック関数のラップ
            tf_timer_cb = lambda agt_id     =agt_id: self._tf_timer_callback(agt_id)
            costmap_cb  = lambda msg, agt_id=agt_id: self._costmap_callback(msg, agt_id)
            odom_cb     = lambda msg, agt_id=agt_id: self._odom_callback(msg, agt_id)
            contact_cb  = lambda msg, agt_id=agt_id: self._contact_callback(msg, agt_id)
            amcl_cb     = lambda msg, agt_id=agt_id: self._amcl_pose_callback(msg, agt_id)
            plan_cb     = lambda msg, agt_id=agt_id: self._transformed_plan_callback(msg, agt_id)
            imu_cb      = lambda msg, agt_id=agt_id: self._imu_callback(msg, agt_id)

            freq = 1/STEP_FREQ


            # qosの設定
            amcl_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL)  # AMCLのPublisher設定と一致させる
            pose_init_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE) # AMCLのPublisher設定と一致させる


            # コネクタの設定
            cmd_vel_rl_pub = self.create_publisher(Twist, f'/{agt_id}/cmd_vel_rl', 10) 
            pose_init_pub = self.create_publisher(PoseWithCovarianceStamped, f'/{agt_id}/initialpose', pose_init_qos)
            contact_sub = self.create_subscription(Contacts, f'/{agt_id}/contact', contact_cb, 10,)
            costmap_sub = self.create_subscription(OccupancyGrid, f'/{agt_id}/local_costmap/costmap', costmap_cb, 10,)
            odom_sub = self.create_subscription(Odometry, f'/{agt_id}/odom', odom_cb, 10,)
            amcl_sub = self.create_subscription(PoseWithCovarianceStamped, f'/{agt_id}/amcl_pose', amcl_cb, amcl_qos,)
            plan_sub = self.create_subscription(Path, f'/{agt_id}/transformed_global_plan', plan_cb, 10,)
            imu_sub  = self.create_subscription(Imu, f'/{agt_id}/imu', imu_cb, 10,)
            goal_client = ActionClient(self, NavigateToPose, f'/{agt_id}/navigate_to_pose',)
            self.tf_timers[agt_id] = self.create_timer(freq, tf_timer_cb)

            global_costmap_clear_client = self.create_client(ClearEntireCostmap, f'/{agt_id}/global_costmap/clear_entirely_global_costmap')


            # コネクタを辞書に格納
            self.connections[agt_id] = {
                "costmap_sub"    : costmap_sub,
                "odom_sub"       : odom_sub,
                "cmd_vel_rl_pub" : cmd_vel_rl_pub,
                "pose_init_pub"  : pose_init_pub,
                "goal_client"    : goal_client,
                "contact_sub"    : contact_sub,
                "amcl_sub"       : amcl_sub,
                "plan_sub"       : plan_sub,
                "imu_sub"        : imu_sub,
                "gmap_client"    : global_costmap_clear_client,
            }
            
        self.get_logger().info(f'{self.initial_poses}')
        self.get_logger().info(f'{self.initial_poses.keys()}')

        self.get_logger().info('スポーン中...')
        self._spawn_all_models()

        if self.use_obstacbles:
            self._spawn_all_obstacles()

        self.get_logger().info('全ロボットのinitialposeを送信中...')
        self._initialize_all_agents_pose()

        self.get_logger().info('ライフサイクル遷移中...')
        self._run_all_agents_lifecycle_transitions()
        
        # 成功/失敗をレポート
        result = self._is_all_lifecycle_successful()
        if not result:
            raise RuntimeError('ライフサイクルの遷移に失敗しました')

        self.step_count = 0
        self.is_first = True

    def reset(self, seed=None, options=None):
        self.get_logger().info('='*60)
        self.get_logger().info('reset')

        # スポーンさせるエンティティの位置情報の更新
        self._update_all_agents_pose()
        self._update_all_obstacles_pose()

        self.agents = self.possible_agents.copy()

        # 状態変数の初期化
        self._initilaize_all_agents_state_variables()

        # ゾンビプロセスの掃除
        self._cleanup_zombie_processes()
        time.sleep(1.0)

        self.get_logger().info('全エージェントを削除中...')
        self._delete_all_models()
        if self.use_obstacbles:
            self._delete_all_obstacles()
        time.sleep(1.2)
        self.get_logger().info('再スポーン中...')
        self._spawn_all_models()
        if self.use_obstacbles:
            self._spawn_all_obstacles()
        time.sleep(1.0)

        for _ in range(50):
            with self._rclpy_lock:
                rclpy.spin_once(self, timeout_sec=0.1)

        self._clear_all_agents_global_costmap()
        
        self.get_logger().info('全ロボットのinitialposeを送信中...')
        self._initialize_all_agents_pose()

        self.get_logger().info('全ロボットのTF正常化関数の呼び出し...')
        self._wait_for_normalize_tf()

        # Gazeboの物理シミュレーションを安定化させるため、追加の待機時間を設ける
        time.sleep(2.0)  # スポーン直後の接触誤検知を防ぐ
        
        for _ in range(100):
            with self._rclpy_lock:
                rclpy.spin_once(self, timeout_sec=0.01)

        # 状態をRUNNINGに設定（この時点から衝突判定が有効になる）
        # self.get_logger().info('全エージェントの状態をRUNNINGにリセットします。')
        self.states = {agt_id: ControllerState.RUNNING for agt_id in self.agents}
        
        self.get_logger().info('全ロボットのgoal設定中...')
        result = self._set_all_goals()
        if not result:
            self.get_logger().error('✗ 全ロボットの目標設定に失敗しました。')

        self._send_all_agents_stop_action()

        time.sleep(2.0)
        
        for _ in range(2000):
            with self._rclpy_lock:
                rclpy.spin_once(self, timeout_sec=0.1)

        # 観測情報取得
        obss = self._get_observations()
        infos = {agt_id: {} for agt_id in self.agents}

        self.ep_starttime = self.sim_time

        self.get_logger().info('='*60)

        return obss, infos

    def step(self, actions):
        # self.get_logger().info('Step関数が呼ばれました')

        # 有効エージェントの再計算
        self.agents = [agt_id for agt_id in self.agents if not self.terminations[agt_id] and not self.truncations[agt_id]]

        # self.get_logger().info(f'選択された行動: {action_name}')

        step_start_time = self.sim_time
     
        infos = {agt_id: {} for agt_id in self.agents}

        # 実行
        self._send_actions(actions)

        # タイムステップの時間分トピックを監視しながら待機
        while (self.sim_time - step_start_time) < (1.0 / STEP_FREQ):
            with self._rclpy_lock:
                rclpy.spin_once(self, timeout_sec=0.1)

        # 終了判定
        self._cleanup_by_collision_detection()
        self._cleanup_by_timeout_detection()

        # 報酬計算
        rewards = self._get_rewards(actions=actions)
            
        # 観測情報取得
        obss = self._get_observations()

        # 各エージェントの終了情報
        step_terminations = {agt_id: self.terminations[agt_id] for agt_id in self.agents}
        step_truncations  = {agt_id: self.truncations[agt_id] for agt_id in self.agents}

        self.step_count += 1

        return obss, rewards, step_terminations, step_truncations, infos

    def save_log(self, state: ControllerState, reward: float, clear_time: float):
        step_count = self.step_count
        path = self.log_path
        state_num = state.value
        elasped_time = self.sim_time
        with open(path, 'a') as f:
            f.write(f'{step_count:6}, {elasped_time:6.4f}, {state_num:2}, {reward:3.2f}, {clear_time:3}\n')           

    def render(self):
        return

    @functools.lru_cache(maxsize=None)
    def observation_space(self, agent):
        low = np.concatenate([
            np.zeros(COSTMAP_SIZE*COSTMAP_SIZE*COSTMAP_BUFFER_SIZE, dtype=np.float32),                # 画像: 0
            np.full(1, -np.inf, dtype=np.float32),            # 速度
            np.full(1, -np.inf, dtype=np.float32),            # 角速度
            np.full(1, -pi, dtype=np.float32),  # 角度誤差(yaw)
        ])
        high = np.concatenate([
            np.full(COSTMAP_SIZE*COSTMAP_SIZE*COSTMAP_BUFFER_SIZE, 255, dtype=np.float32),            # 画像: 255
            np.full(1, np.inf, dtype=np.float32),             # 速度
            np.full(1, np.inf, dtype=np.float32),             # 角速度
            np.full(1, pi, dtype=np.float32),  # 角度誤差(yaw)
        ])
        return Box(low=low, high=high, dtype=np.float32)

    @functools.lru_cache(maxsize=None)
    def action_space(self, agent):
        if self.action_type == "discrete":
            # 離散アクション空間
            return Discrete(len(self.action_data))
        elif self.action_type == "continuous":
            # [並進速度(v), 回転速度(w)] を -1.0 〜 1.0 の範囲で出力
            return Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
    
    def _update_all_agents_pose(self):
        """
        全エージェントのスポーン位置情報を更新する
        """
        #obs 静的障害物環境
        if self.world_name == "obs":
            for row in range(2):
                for col in range(5):
                    id = row * 5 + col + 1
                    agt_id = f'robot_{id}'
                    x = -8.0 + col * 4.0 + random.uniform(-0.5, 0.5)
                    y = -7.0 + row * 9.0
                    yaw = random.uniform(-pi, pi)
                    self.initial_poses[agt_id] = Pose(x=x, y=y, yaw=yaw)

            for row in range(2):
                for col in range(5):
                    id = row * 5 + col + 1
                    agt_id = f'robot_{id}'
                    x = -8.0 + col * 4.0
                    y = -2.0 + row * 9.0
                    yaw = random.uniform(3*pi/4, pi/4)
                    self.goal_pose_dict[agt_id] = Pose(x=x, y=y, yaw=yaw)

        # # obs 交差環境　
        # if self.world_name == "obs":
        #     for row in range(2):
        #         for col in range(5):
        #             id = row + col*2 + 1
        #             agt_id = f'robot_{id}'
        #             x = -8.0 + col * 4.0
        #             # y = -7.0 + row * 9.0
        #             y = -7.0 if row == 0 else -2.0
        #             yaw = random.uniform(-pi, pi)
        #             # yaw = pi/2 if row == 0 else -pi/2
        #             self.initial_poses[agt_id] = Pose(x=x, y=y, yaw=yaw)

        #     for row in range(2):
        #         for col in range(5):
        #             id = row + col*2 + 1
        #             agt_id = f'robot_{id}'
        #             x = -8.0 + col * 4.0
        #             # y = -2.0 + row * 9.0
        #             y = -2.0 if row == 0 else -7.0
        #             yaw = random.uniform(3*pi/4, pi/4)
        #             # yaw = pi/2 if row == 0 else -pi/2
        #             self.goal_pose_dict[agt_id] = Pose(x=x, y=y, yaw=yaw)

        if self.world_name == "follow_path":
            # follow_path 環境
            
            for col in range(5):
                for row in range(2):
                    id = row + col*2 + 1
                    agt_id = f'robot_{id}'
                    x = -6.0 + col * 3.0
                    y = -7.0 if row == 0 else -2.0
                    yaw = random.uniform(-pi, pi)
                    self.initial_poses[agt_id] = Pose(x=x, y=y, yaw=yaw)
        
            
            for col in range(5):
                for row in range(2):
                    id = row + col*2 + 1
                    agt_id = f'robot_{id}'
                    x = -6.0 + col * 3.0
                    y = -2.0 if row == 0 else -7.0
                    yaw = random.uniform(3*pi/4, pi/4)
                    self.goal_pose_dict[agt_id] = Pose(x=x, y=y, yaw=yaw)
            return

    def _update_all_obstacles_pose(self):
        """
        障害物の位置情報を更新する
        """
        for row in range(2):
            for col in range(5):
                id = row * 5 + col + 1
                agt_id = f'obstacle_{id}'
                x = -8.0 + col * 4.0 + random.uniform(-0.5, 0.5)
                y = -4.0 + row * 9.0 
                z = 0.5
                yaw = random.uniform(-pi, pi)
                self.obstacle_poses[agt_id] = Pose(x=x, y=y, z=z, yaw=yaw)
    
    def _clear_all_agents_global_costmap(self):
        for agt_id in self.agents:
            client = self.connections[agt_id]['gmap_client']
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'[{agt_id}] グローバルコストマップクリアサービス待機中...')
            req = ClearEntireCostmap.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

    def _initilaize_all_agents_state_variables(self):
        """
        全エージェントの状態変数の初期化
        """

        # コストマップバッファーの作成(観測に複数フレーム使用する場合)
        self.costmaps_buffer = {}
        for agt_id in self.agents:
            dq = deque(maxlen=COSTMAP_BUFFER_SIZE)
            dummy_map = np.full(COSTMAP_SIZE * COSTMAP_SIZE, 255, dtype=np.float32)
            for _ in range(COSTMAP_BUFFER_SIZE):
                dq.append(dummy_map)
            self.costmaps_buffer[agt_id] = dq
    
        self.costmaps                       = {agt_id: None  for agt_id in self.agents}
        self.vels                           = {agt_id: None  for agt_id in self.agents}
        self.ang_vels                       = {agt_id: None  for agt_id in self.agents}
        self.poses                          = {agt_id: None  for agt_id in self.agents}
        self.plan                          = {agt_id: None  for agt_id in self.agents}
        self.accelerations                  = {agt_id: None  for agt_id in self.agents}
        self.goal_handle                   = {agt_id: None  for agt_id in self.agents}
        self.achieved                      = {agt_id: False for agt_id in self.agents}
        self.clear_time                    = {agt_id: None  for agt_id in self.agents}
        self.done_estimation               = {agt_id: False for agt_id in self.agents}
        self.latest_tfs                    = {agt_id: None  for agt_id in self.agents}
        self.prev_distance                 = {agt_id: None  for agt_id in self.agents}
        self.prev_subgoal_pos              = {agt_id: None  for agt_id in self.agents}
                 
        self.terminations                  = {agt_id: False for agt_id in self.agents}
        self.truncations                   = {agt_id: False for agt_id in self.agents}

        self.waiting_timeout_callback      = {agt_id: False for agt_id in self.agents}
        self.waiting_crash_callback        = {agt_id: False for agt_id in self.agents}
        self.collision_detected_by_planner = {agt_id: False for agt_id in self.agents}
        self.timeout_rewarded              = {agt_id: False for agt_id in self.agents}
        self.collision_rewarded            = {agt_id: False for agt_id in self.agents}

        self.ep_total_reward = {agt_id: 0.0 for agt_id in self.agents}

    def _get_observations(self):

        with self.costmaps_lock:
            costmaps = self.costmaps.copy()
        with self.vels_lock:
            vels = self.vels.copy()
        with self.ang_vels_lock:
            ang_vels = self.ang_vels.copy()
        with self.tfs_lock:
            tfs = self.latest_tfs.copy()
        with self.poses_lock:
            poses = self.poses.copy()
        with self.accelerations_lock:
            accelerations = self.accelerations.copy()

        observations = {}
        for agt_id in self.agents:
            costmap = costmaps.get(agt_id, np.zeros((COSTMAP_SIZE, COSTMAP_SIZE), dtype=np.uint8))
            if costmap is None:
                costmap = np.zeros((COSTMAP_SIZE, COSTMAP_SIZE), dtype=np.uint8)

            costmap = costmap.astype(np.uint8)
            pose = poses.get(agt_id, None)
            # self.get_logger().info(f'[{agt_id}] pose: {pose}')
            q = pose.orientation
            # self.get_logger().info(f'[{agt_id}] pose orientation: {rot}')
            yaw = quaternion_to_yaw(q)
            deg = int(math.degrees(yaw))  

            tf = tfs.get(agt_id, None)
            if tf is None:
                tf = Pose(x=0.0, y=0.0, yaw=0.0) # initialpoesに変更する？

            rows, cols = costmap.shape
            center = (cols // 2, rows // 2)
            # costmap = costmap.T
            M = cv2.getRotationMatrix2D(center, deg, 1.0)
            rotated_costmap = cv2.warpAffine(costmap, M, (cols, rows), borderValue=0)
            
            costmap = rotated_costmap.flatten()

            # costmap = costmap.flatten()
            self._update_buffer(agt_id, costmap)
            costmap_buffer = np.array(self.costmaps_buffer[agt_id]).flatten()
            vel = vels.get(agt_id, np.zeros(1, dtype=np.float32))
            if vel is None:
                vel = np.zeros(1, dtype=np.float32)
            vel = vel.flatten()
            ang_vel = ang_vels.get(agt_id, np.zeros(1, dtype=np.float32))
            if ang_vel is None:
                ang_vel = np.zeros(1, dtype=np.float32)
            ang_vel = ang_vel.flatten()
            ang_error = self._get_angle_error(agt_id)
            ang_error = np.array([ang_error], dtype=np.float32)
            ang_error = ang_error.flatten()

            obs_vec = np.concatenate([costmap_buffer, vel, ang_vel, ang_error], axis=0)

            observations[agt_id] = obs_vec 

        return observations
    
    def _get_target_pose(self):
        target_poss = {}
        with self.plan_lock:
            plans = self.plan.copy()
        for agt_id in self.agents:
            plan = plans.get(agt_id, None)
            if plan is None:
                target_poss[agt_id] = Pose(x=0.0, y=0.0, yaw=0.0)
                continue
            for pt in reversed(plan[:TARGET_PLAN_POINT_IDX]):
                if pt.x == DUMMY_VAL or pt.y == DUMMY_VAL:
                    continue
                target_poss[agt_id] = pt
                break

        return target_poss

    def _update_buffer(self, agt_id, new_costmap):
        self.costmaps_buffer[agt_id].appendleft(new_costmap)

    def _send_actions(self, actions):
        """cmd_velコマンドを実行する"""
        # 実行
        for agt_id, action in actions.items():
            if not self.use_rl:
                break
            cmd_vel_rl_pub = self.connections[agt_id]["cmd_vel_rl_pub"]

            # 離散行動空間の場合
            if self.action_type == "discrete":
                # 離散アクション空間の場合
                action_cmd = self.action_data[action]
                cmd_vel_rl_pub.publish(action_cmd)
                continue

            # 連続アクション空間の場合
            elif self.action_type == "continuous":
                linear_vel, angular_vel = self._map_continuous_action_to_real_value(action)
                
                # Twistメッセージ作成
                cmd = Twist()
                cmd.linear.x = float(linear_vel)
                cmd.angular.z = float(angular_vel)
                
                cmd_vel_rl_pub.publish(cmd)

    def _map_continuous_action_to_real_value(self, action):
        """
        連続アクション空間の値を実際の速度指令値に変換する
        
        Parameters
        ----------
        action : np.ndarray
            連続アクション空間の値 (shape: (2,), 範囲: [-1.0, 1.0])
        
        Returns
        -------
        linear_vel : float
            並進速度 (範囲: [MIN_LINEAR_VEL, MAX_LINEAR_VEL])
        angular_vel : float
            回転速度 (範囲: [-MAX_ANGULAR_VEL, +MAX_ANGULAR_VEL])
        """
        # 並進速度: 0 〜 MAX_LINEAR_VEL
        linear_vel = ((action[0] + 1.0) / 2.0) * (MAX_LINEAR_VEL - MIN_LINEAR_VEL) + MIN_LINEAR_VEL   
        # 回転速度: -MAX 〜 +MAX
        angular_vel = action[1] * MAX_ANGULAR_VEL
        
        return linear_vel, angular_vel

    def _clock_callback(self, msg):
        """/clockトピックのコールバック"""
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        # self.get_logger().info(f'Sim Time: {self.sim_time:.2f} sec')

    def _costmap_callback(self, msg, agt_id):
        """ローカルコストマップのコールバック（Nav2コントローラーと同じ情報源）"""
        # コストマップをnumpy配列に変換
        with self.costmaps_lock:
            width = msg.info.width
            height = msg.info.height
            self.costmaps[agt_id] = np.array(msg.data, dtype=np.int8).reshape(height, width)

    def _odom_callback(self, msg, agt_id):
        """オドメトリのコールバック（速度・角速度のみ抽出）"""
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        pose = msg.pose.pose
        with self.vels_lock:
            self.vels[agt_id] = np.array([linear.x])
        with self.ang_vels_lock:
            self.ang_vels[agt_id] = np.array([angular.z])
        with self.poses_lock:
            self.poses[agt_id] = pose
        
    def _contact_callback(self, msg, agt_id):
        """コンタクト情報のコールバック"""

        # 現在の状態がRUNNINGでない場合は無視(不正に状態をCLLIDEDで上書きするのを防ぐ)
        if self.states[agt_id] != ControllerState.RUNNING:
            return
        
        if self.sim_time is None or self.ep_starttime is None:
            return
        if self.sim_time - self.ep_starttime < CONTACT_DETECTION_START_DELAY:
            # スポーン直後の誤検知を防ぐため、一定時間は無視
            return  

        # 一度衝突検知あるいはタイムアウトでナビゲーションキャンセルの待機中の場合は無視
        if self.waiting_crash_callback[agt_id] or self.waiting_timeout_callback[agt_id]:
            return

        try:
            num_contacts = len(msg.contacts)
        except Exception:
            num_contacts = 0

        if num_contacts <= 0 or num_contacts is None:
            return

        for contact in msg.contacts:
            # contact は ros_gz_interfaces.msg.Contact 型の要素
            # センサではない側のコリジョン情報（ここでは collision2 を参照）
            opponent_name = 'unknown'
            c2 = contact.collision2
            if not hasattr(c2, 'name'):
                continue
            
            # 衝突相手が地面の場合は無視
            opponent_name = c2.name.split("::")[0]
            if opponent_name == "ground_plane":
                continue
            # 状態を更新 
            self.states[agt_id] = ControllerState.COLLIDED
  
    def _amcl_pose_callback(self, msg, agt_id):
        """AMCL位置推定のコールバック: 初回受信時にdone_estimationをTrueにする"""

        # self.get_logger().info(f'[{agt_id}] AMCL pose コールバック受信')

        if self.done_estimation.get(agt_id, False):
            return
        
        initial_pose = self.initial_poses[agt_id]
        
        # 受信したAMCL推定位置姿勢
        estimated_pose = msg.pose.pose
        position = estimated_pose.position
        x = position.x
        y = position.y
        orientation = estimated_pose.orientation
        yaw = quaternion_to_yaw(orientation)
        estimated_pose = Pose(x=x, y=y, yaw=yaw)
        
        # 推定位置が誤差範囲内かチェック
        if not self._in_torerance_range(estimated_pose, initial_pose):
            return
        
        self.done_estimation[agt_id] = True
        # self.get_logger().info(f'[{agt_id}] AMCL位置推定完了')

    def _transformed_plan_callback(self, msg, agt_id):
        """変換済みグローバルプランのコールバック: 目標到達判定に使用"""
        # self.get_logger().info(f'[{agt_id}]のプランを受け取りました')
        _plan = msg.poses

        plan = [Pose(x=DUMMY_VAL, y=DUMMY_VAL, yaw=DUMMY_VAL) for _ in range(MAX_POINTS)]
        for i, pt in enumerate(_plan[:MAX_POINTS]):
            pos = pt.pose.position  
            rot = pt.pose.orientation
            yaw = quaternion_to_yaw(rot)
            plan[i] = Pose(x=pos.x, y=pos.y, yaw=yaw)
    
        with self.plan_lock:
            self.plan[agt_id] = plan

    def _tf_timer_callback(self, agt_id):
        """TFリスナのタイマコールバック（定期的に呼ばれる）"""
        to_frame_id = f"{agt_id}/base_footprint"
        from_frame_id = f"{agt_id}/map"        
        try:
            trans = self.tf_buffers[agt_id].lookup_transform(
                from_frame_id,
                to_frame_id,
                rclpy.time.Time(),
            )
            pos = trans.transform.translation
            rot = trans.transform.rotation

            with self.tfs_lock:
                self.latest_tfs[agt_id] = Pose(
                    x=pos.x,
                    y=pos.y,
                    yaw=quaternion_to_yaw(rot),
                )
        except Exception as e:
            # self.get_logger().warn(f'[{agt_id}] TF取得に失敗: {e}')
            pass

    def _imu_callback(self, msg, agt_id):
        """IMUのコールバック"""
        # # 加速度情報の抽出
        # acceleration = msg.linear_acceleration.x
        # ang_vel      = msg.angular_velocity.z

        # with self.accelerations_lock:
        #     self.accelerations[agt_id] = np.array([acceleration], dtype=np.float32)
        # with self.ang_vels_lock:
        #     self.ang_vels[agt_id] = np.array([ang_vel], dtype=np.float32)
        
        return
        
    def _in_torerance_range(self, expected: Pose, actual: Pose):
        """expectedとactualがtolerance以内かチェック"""
        dist = math.sqrt((expected.x - actual.x)**2 + (expected.y - actual.y)**2)
        return dist <= POSITION_TOLERANCE and  \
               abs(expected.yaw - actual.yaw) <= YAW_TOLERANCE
    
    def _cleanup_by_collision_detection(self):
        """衝突検知によるクリーンアップ処理"""
        for agt_id in self.agents:
            if self.waiting_crash_callback[agt_id] or self.waiting_timeout_callback[agt_id]:
                continue
            if self.states[agt_id] != ControllerState.COLLIDED:
                continue

            if self.collision_detected_by_planner[agt_id]:
                continue  # すでにプランナー側で衝突検知されている場合はスキップ

            if self.goal_handle[agt_id] is not None:
                try:
                    # ナビゲーションの停止リクエストの送信（非同期キャンセル）
                    self.goal_handle[agt_id].cancel_goal_async()
                    self.waiting_crash_callback[agt_id] = True
                except Exception as e:
                    self.get_logger().error(f'[{agt_id}] キャンセルに失敗: {e}')
            else:
                self.get_logger().error(f'エージェント{agt_id}のgoal_handleがNoneのため、キャンセル処理をスキップします。')

    def _cleanup_by_timeout_detection(self):
        """タイムアウト検知によるクリーンアップ処理"""

        if self.sim_time - self.ep_starttime < SIMULATION_TIMEOUT:
            return
        
        for agt_id in self.agents:
            
            # すでにRUNNING状態でない場合はスキップ(状態が不正に上書きされるのを防ぐ)
            if self.states[agt_id] != ControllerState.RUNNING:
                continue
            # すでに衝突またはタイムアウトのコールバック待機中の場合はスキップ
            if self.waiting_crash_callback[agt_id] or self.waiting_timeout_callback[agt_id]:
                continue

            if self.collision_detected_by_planner[agt_id]:
                continue  # すでにプランナー側で衝突検知されている場合はスキップ
            
            self.states[agt_id] = ControllerState.TIMEOUT

            if self.goal_handle[agt_id] is not None:
                try:
                    self.goal_handle[agt_id].cancel_goal_async()
                    self.waiting_timeout_callback[agt_id] = True
                except Exception as e:
                    self.get_logger().error(f'[{agt_id}] キャンセルに失敗: {e}')
            else:
                self.get_logger().info(f'エージェント{agt_id}のgoal_handleがNoneのため、キャンセル処理をスキップします。')

    def _calc_reward_dist_to_subgoal(self, agt_id: str, tfs: dict, subgoals: dict) -> float:
        """
        サブゴールへの距離変化に基づく報酬計算
        
        Parameters
        ----------
        agt_id : str
            エージェントID
        tfs : dict
            各エージェントの現在位置姿勢を格納した辞書
        subgoals : dict
            各エージェントのサブゴール位置姿勢を格納した辞書

        Returns
        -------
        reward_dist_to_subgoal : float
            サブゴールへの距離変化に基づく報酬   
        """

        # エージェントの現在位置(global座標系)を取得
        tf = tfs.get(agt_id, None)
        if tf is None:
            tf = self.initial_poses[agt_id]
        robot_pose = tf

        # サブゴール位置をglobal座標系に変換
        subgoal_in_local_frame = subgoals.get(agt_id, None)
        src_frame = f'{agt_id}/base_footprint'
        target_frame = f'{agt_id}/map'
        subgoal_pose_in_global_frame = self._transform_pose(agt_id=agt_id, pose=subgoal_in_local_frame, source_frame=src_frame, target_frame=target_frame)

        # サブゴール位置が取得できない場合は現在位置を使用
        if subgoal_pose_in_global_frame is None:
            subgoal_pose_in_global_frame = robot_pose

        # 前回計算したサブゴールへの距離とサブゴール位置を取得
        prev_dist_to_subgoal = self.prev_distance.get(agt_id, None)
        prev_subgoal_pos = self.prev_subgoal_pos.get(agt_id, None)

        # 現在のサブゴールへの距離を算出
        current_dist_to_subgoal = math.sqrt((subgoal_pose_in_global_frame.x - robot_pose.x)**2 + (subgoal_pose_in_global_frame.y - robot_pose.y)**2)

        # 現在のサブゴール位置とサブゴールまでの距離を保存
        self.prev_subgoal_pos[agt_id] = Pose(x=subgoal_pose_in_global_frame.x, y=subgoal_pose_in_global_frame.y, yaw=0.0)
        self.prev_distance[agt_id] = current_dist_to_subgoal

        # 前ステップのデータがない場合, 報酬は０とする
        if prev_subgoal_pos is None:
            return 0.0
        if prev_dist_to_subgoal is None:
            return 0.0
        
        # 前回のサブゴール位置への現在のロボットからの距離を算出
        current_dist_to_prev_subgoal = math.sqrt((prev_subgoal_pos.x - robot_pose.x)**2 + (prev_subgoal_pos.y - robot_pose.y)**2)
        
        # サブゴールへの距離変化に基づく報酬計算
        dist_diff = prev_dist_to_subgoal - current_dist_to_prev_subgoal
        reward_dist_to_subgoal = REWARD_SUBGOAL_COEF * dist_diff

        return reward_dist_to_subgoal 

    def _calc_reward_speed(self, speed: float) -> float:
        """
        移動速度に基づく報酬計算
        
        Parameters
        ----------
        speed : float
            エージェントの現在速度 [m/s]

        Returns
        -------
        reward_speed : float
            移動速度に基づく報酬
        """

        # バック時の報酬
        if speed < 0.0:
            return REWARD_BACKWARD

        # 高速移動時の報酬
        return np.interp(
            speed, 
            [HIGH_SPEED_DOMAIN['min'], HIGH_SPEED_DOMAIN['max']], 
            [0.0, REWARD_HIGH_SPEED_COEF]
        )

    def _get_rewards(self, actions=None):
        """
        報酬計算用の関数(同時に学習ログの作成も行う)
        
        Returns
        -------
        rewards : dict
            各エージェントの報酬を格納した辞書
        """
        rewards = {agt_id: 0.0 for agt_id in self.agents}

        with self.tfs_lock:
            tfs = self.latest_tfs.copy()

        target_poss = self._get_target_pose()

        # 報酬計算
        for agt_id in self.agents:

            # すでに衝突ペナルティまたはタイムアウト報酬が与えられている場合はスキップ(ナビゲーションのキャンセル待ち)
            if self.collision_rewarded.get(agt_id, False) or self.timeout_rewarded.get(agt_id, False):
                continue

            # 走行状態の場合の報酬計算
            if self.states[agt_id] == ControllerState.RUNNING:
                # ステップ毎の微小報酬
                rewards[agt_id] += REWARD_STEP
                # 高速移動, バックに基づく報酬
                if self.action_type == "continuous":
                    linear_vel, _ = self._map_continuous_action_to_real_value(actions[agt_id])
                else:
                    linear_vel = self.action_data[actions[agt_id]].linear.x
                rewards[agt_id] += self._calc_reward_speed(linear_vel)
                # サブゴールへの距離変化に基づく報酬  
                rewards[agt_id] += self._calc_reward_dist_to_subgoal(agt_id, tfs, target_poss)
                continue

            # 衝突時の報酬計算
            if self.states[agt_id] == ControllerState.COLLIDED:
                reward = REWARD_COLLISION
                rewards[agt_id] = reward  # 衝突ペナルティ
                self.collision_rewarded[agt_id] = True
                total_reward = self.ep_total_reward[agt_id] + rewards[agt_id]
                elapsed_time = self.sim_time - self.ep_starttime
                self.save_log(self.states[agt_id], total_reward, elapsed_time)
                continue
            
            # タイムアウト時の報酬計算
            elif self.states[agt_id] == ControllerState.TIMEOUT:   
                rewards[agt_id] = REWARD_STEP
                self.timeout_rewarded[agt_id] = True
                total_reward = self.ep_total_reward[agt_id] + REWARD_STEP 
                self.save_log(self.states[agt_id], total_reward, SIMULATION_TIMEOUT)
                continue
            
            # 目標到達時の報酬計算
            elif self.states[agt_id] == ControllerState.ACHIEVED:
                clear_time = self.clear_time[agt_id]
                self.terminations[agt_id] = True
                total_reward = self.ep_total_reward[agt_id] + REWARD_GOAL
                self.save_log(self.states[agt_id], total_reward, clear_time)
                self.states[agt_id] = ControllerState.WAITING

        # エピソード累積報酬の更新
        for agt_id, r in rewards.items():
            self.ep_total_reward[agt_id] += r

        return rewards
    
    def _transform_pose(self, agt_id: str, pose: Pose, target_frame: str, source_frame: str):
        """
        座標をsource_frameからtarget_frameに変換する関数
        
        Parameters
        ----------
        agt_id : str
            エージェントID
        pose : Pose
            位置姿勢を表現するオブジェクト
        target_frame : str
            変換先フレーム名
        source_frame : str
            変換元フレーム名

        Returns
        -------
        pos : geometry_msgs.msg.Point
            変換後の位置情報
        rot : geometry_msgs.msg.Quaternion
            変換後の姿勢情報
        """
        try:
            # 変換用のPoseStampedメッセージを作成
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rclpy.time.Time(seconds=0).to_msg()
            pose_msg.header.frame_id = source_frame
            pose_msg.pose.position.x = pose.x   
            pose_msg.pose.position.y = pose.y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation = quaternion_from_yaw(pose.yaw)
            # TF変換の実行
            tf_buffer = self.tf_buffers[agt_id]
            pose_on_target_frame = tf_buffer.transform(
                pose_msg,
                target_frame,
            )
            pos = pose_on_target_frame.pose.position
            rot = pose_on_target_frame.pose.orientation
            yaw = quaternion_to_yaw(rot)
            return Pose(x=pos.x, y=pos.y, yaw=yaw)
        except Exception as e:
            self.get_logger().error(f'[{agt_id}] ゴールのロボットフレーム変換に失敗: {e}')
            return None, None

    def _initialize_all_agents_pose(self):
        """全ロボットのinitialposeを送信する（初期化用）"""
        threads = []
        for agt_id in self.possible_agents:
            s_pose = self.initial_poses[agt_id]
            t = threading.Thread(
                target=self._handle_initial_pose,
                args=(agt_id, s_pose),
                daemon=False
            )
            t.start()
            threads.append(t)

        while not all(self.done_estimation.values()):
            with self._rclpy_lock:
                rclpy.spin_once(self, timeout_sec=0.1)

        # 全スレッドの完了を待つ
        for t in threads:
            t.join()

    def _handle_initial_pose(self, agt_id: str, pose: Pose):
        """
        自己位置推定化が完了するまでロボットのinitialposeを送信する
        
        Parameters
        ----------
        agt_id : str
            エージェントID
        pose : Pose
            位置姿勢を表現するオブジェクト
        """
        
        while not self.done_estimation[agt_id]:
            self._publish_initial_pose(agt_id, pose)
            time.sleep(1.0)

    def _publish_initial_pose(self, agt_id: str, pose: Pose):
        '''
        ロボットの自己推定位置を配信する
        
        Parameters
        ----------
        agt_id : str
            エージェントID
        pose : Pose
            位置姿勢を表現するオブジェクト
        '''
        pose_msg = PoseWithCovarianceStamped()
        
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = f"{agt_id}/map"
        pose_msg.pose.pose.position.x = float(pose.x)
        pose_msg.pose.pose.position.y = float(pose.y)
        pose_msg.pose.pose.position.z = 0.0  # 2Dロボットは0.0
        pose_msg.pose.pose.orientation = quaternion_from_yaw(pose.yaw)
        pose_msg.pose.covariance = [
            0.001, 0.0  , 0.0, 0.0, 0.0, 0.0,
            0.0  , 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0  , 0.0  , 0.0, 0.0, 0.0, 0.0,
            0.0  , 0.0  , 0.0, 0.0, 0.0, 0.0,
            0.0  , 0.0  , 0.0, 0.0, 0.0, 0.0,
            0.0  , 0.0  , 0.0, 0.0, 0.0, 0.0,
            0.0  , 0.0  , 0.0, 0.0, 0.0, 0.0001
        ]
        publisher = self.connections[agt_id]["pose_init_pub"]
        publisher.publish(pose_msg)
        # self.get_logger().info(f'[{agt_id}] initialposeを送信しました: x={pose.x:.2f}, y={pose.y:.2f}, yaw={math.degrees(pose.yaw):.2f} deg')
   
    def _send_stop_action(self, agt_id: str):
        """ナビゲーションを停止するゴールを送信する"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0 
        cmd_vel_rl_pub = self.connections[agt_id]["cmd_vel_rl_pub"]
        cmd_vel_rl_pub.publish(cmd)

    def _send_all_agents_stop_action(self):
        """全ロボットのナビゲーションを停止するゴールを送信する"""
        for agt_id in self.agents:
            self._send_stop_action(agt_id)

    def _get_angle_error(self, agt_id: str, target_point=None) -> float:
        """ロボットの現在の向きと目標位置への角度誤差を計算する"""
        target_point = self._get_target_pose().get(agt_id, None)
        tx = target_point.x
        ty = target_point.y
        error = math.atan2(ty, tx)
        return error

    def _send_nav_goal_action(self, agt_id: str, x: float, y: float, yaw: float):
        """Action Clientを通じてゴールを送信し、GoalHandleを保存する"""
        
        goal_client = self.connections[agt_id]["goal_client"]
        
        # Action Serverの準備を確認
        if not goal_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{agt_id}] Action Server タイムアウト')
            return
        
        # 実行中のナビゲーション情報があれば削除
        existing = self.goal_handle.get(agt_id, None)
        if existing is not None:
            try:
                # ゴールをキャンセルしてクリーンアップ
                cancel_future = existing.cancel_goal_async()
                # キャンセル完了を待たずに次へ（ノンブロッキング）
                self.goal_handle[agt_id] = None
            except Exception as e:
                self.get_logger().warn(f'[{agt_id}] 既存ゴールのキャンセルに失敗: {e}')
                self.goal_handle[agt_id] = None

        # ActionのGoalメッセージを作成
        goal_msg = NavigateToPose.Goal() 
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = f"{agt_id}/map"
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = quaternion_from_yaw(yaw)

        # ゴールを送信
        send_goal_future = goal_client.send_goal_async(goal_msg)

        # ゴール送信リクエストの応答を受け取ったときにに呼び出されるコールバック関数を定義
        def handle_goal_response(future):

            # self.get_logger().info(f'[{agt_id}] handle_goal_response コールバックが呼ばれました')
            try:
                goal_handle = future.result()
            except Exception as e:
                self.get_logger().error(f'[{agt_id}] ゴール送信Futureの実行中に例外が発生: {e}')
                self.goal_handle[agt_id] = None 
                return
            
            if goal_handle is None:
                self.get_logger().error(f'[{agt_id}] goal_handleがNoneです（Action Server応答なし）')
                self.goal_handle[agt_id] = None
                return
            
            if not goal_handle.accepted:
                self.get_logger().error(f'[{agt_id}] ゴール要求がAction Serverに拒否されました。ナビゲーション開始不可。')
                self.goal_handle[agt_id] = None # 失敗した場合はNoneに戻す
                return

            # ハンドルを保存
            # self.get_logger().info(f'[{agt_id}] ゴールが受理されました。追跡ハンドルを保存しました。')
            if goal_handle is None:
                # self.get_logger().info(f'[{agt_id}] goal_handleがNoneです（予期せぬエラー）')
                pass
            self.goal_handle[agt_id] = goal_handle

            # ナビゲーション完了時のコールバック関数を登録
            try:
                result_future = goal_handle.get_result_async()
            except Exception as e:
                self.get_logger().error(f'[{agt_id}] ナビゲーション完了時のコールバック関数登録に失敗: {e}')
                return
            
            # ナビゲーション完了時に呼び出されるコールバック関数を定義
            def _handle_result(fut):
                try:
                    r = fut.result()
                except Exception as e:
                    self.get_logger().error(f'[{agt_id}] ナビゲーション結果の取得に失敗: {e}')
                    return
                
                elapsed = self.sim_time - self.ep_starttime if self.ep_starttime else None

                status = getattr(r, 'status', None)
                result = getattr(r, 'result', None)
                error_code = getattr(result, 'error_code', None) if result is not None else None

                # ナビゲーションの状態に応じた処理
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(f'{agt_id}: ✅ ナビゲーション成功！ ')
                    self.get_logger().info(f'クリアタイム: {elapsed:.2f}秒')
                    self.states[agt_id] = ControllerState.ACHIEVED
                    self.clear_time[agt_id] = elapsed
                elif status == GoalStatus.STATUS_CANCELED:
                    # self.get_logger().warn(f'{agt_id}:タスクは外部からキャンセルされました。')
                    if self.states[agt_id] == ControllerState.COLLIDED:
                        self.get_logger().info(f'[{agt_id}]  ⚠️ 衝突によるキャンセルを検知 状態をWAITINGに更新します。')
                    elif self.states[agt_id] == ControllerState.TIMEOUT:
                        self.get_logger().info(f'[{agt_id}]  ⚠️ タイムアウトによるキャンセルを検知しました。 状態をWAITINGに更新します。')
                    else:
                        self.get_logger().info(f'[{agt_id}]  ⚠️ 不明な理由によるキャンセルを検知しました。 状態: {self.states[agt_id].name}')

                    self.terminations[agt_id] = True
                    self.states[agt_id] = ControllerState.WAITING
                    
                elif status == GoalStatus.STATUS_ABORTED:
                    # エラーコードで失敗原因を判定
                    if error_code == ERRORCODE.NO_VALID_PATH:  # ComputePathToPose.NO_VALID_PATH
                        self.get_logger().error(f'{agt_id}: ❌ 有効な経路が見つかりません')
                        with open(self.error_log_path, 'a') as f:
                            f.write(f'{agt_id} NO_VALID_PATH at sim_time {self.sim_time}\n')
                    elif error_code == ERRORCODE.START_OCCUPIED:  # FollowPath.NO_VALID_CONTROL
                        self.get_logger().error(f'{agt_id}: ❌ 開始地点が専有されています')
                        self.states[agt_id] = ControllerState.COLLIDED
                        self.collision_detected_by_planner[agt_id] = True
                        self.terminations[agt_id] = True
                        return
                    elif error_code == ERRORCODE.TF_ERROR:  # FollowPath.FAILED_TO_MAKE_PROGRESS
                        self.get_logger().error(f'{agt_id}: ❌ TF エラー')
                        with open(self.error_log_path, 'a') as f:
                            f.write(f'{agt_id} TF_ERROR at sim_time {self.sim_time}\n')
                    elif error_code == ERRORCODE.TIMEOUT:  # TF_ERROR (複数アクション)
                        self.get_logger().error(f'{agt_id}: ❌ タイムアウト')
                        with open(self.error_log_path, 'a') as f:
                            f.write(f'{agt_id} TIMEOUT at sim_time {self.sim_time}\n')
                    elif error_code == ERRORCODE.GOAL_OCCUPIED:  # TIMEOUT (複数アクション)
                        self.get_logger().error(f'{agt_id}: ❌ 目標地点が専有されています')
                        with open(self.error_log_path, 'a') as f:
                            f.write(f'{agt_id} GOAL_OCCUPIED at sim_time {self.sim_time}\n')
                    elif error_code == ERRORCODE.GOAL_OUTSIDE_MAP:  # GOAL_OUTSIDE_MAP (複数アクション)
                        self.get_logger().error(f'{agt_id}: ❌ 目標地点がマップ外です')
                        with open(self.error_log_path, 'a') as f:
                            f.write(f'{agt_id} GOAL_OUTSIDE_MAP at sim_time {self.sim_time}\n')
                    elif error_code == ERRORCODE.START_OUTSIDE_MAP:  # START_OUTSIDE_MAP (複数アクション)
                        self.get_logger().error(f'{agt_id}: ❌ 開始地点がマップ外です')
                        with open(self.error_log_path, 'a') as f:
                            f.write(f'{agt_id} START_OUTSIDE_MAP at sim_time {self.sim_time}\n')
                    elif error_code == ERRORCODE.INVALID_PLANNER:  # INVALID_PLANNER
                        self.get_logger().error(f'{agt_id}: ❌ 無効なプランナーです')
                        with open(self.error_log_path, 'a') as f:
                            f.write(f'{agt_id} INVALID_PLANNER at sim_time {self.sim_time}\n')
                    else:
                        self.get_logger().error(f'{agt_id}: ❌ ナビゲーション失敗 (error_code: {error_code})')
                        with open(self.error_log_path, 'a') as f:
                            f.write(f'{agt_id} UNKNOWN_ERROR_{error_code} at sim_time {self.sim_time}\n')


                    self.states[agt_id] = ControllerState.WAITING
                    self.terminations[agt_id] = True

                else:
                    self.get_logger().debug(f'[{agt_id}] 謎のステータス: {status}')
                    with open(self.error_log_path, 'a') as f:
                        f.write(f'{agt_id} UNKNOWN_STATUS_{status} at sim_time {self.sim_time}\n')

                # GoalHandleをクリア
                try:
                    self.goal_handle[agt_id] = None
                except Exception:
                    pass
            
            result_future.add_done_callback(_handle_result)
            
        send_goal_future.add_done_callback(handle_goal_response)

    def _run_all_agents_lifecycle_transitions(self):
        '''全ロボットのライフサイクル遷移を実行'''

        self._lifecycle_success = {}
        lifecycle_threads = []
        for i in range(self.robot_count):
            agt_id = f"robot_{i+1}"
            self._lifecycle_success[agt_id] = False
            thread = threading.Thread(
                target=self._run_lifecycle_script_wrapper,
                args=(agt_id,),
                daemon=False
            )
            thread.start()
            lifecycle_threads.append(thread)
        
        # 全スレッドの完了を待つ
        for thread in lifecycle_threads:
            thread.join()
            self.get_logger().info(f'ライフサイクル遷移スレッド完了: {thread.name}')

    def _run_lifecycle_script_wrapper(self, agt_id):
        """ライフサイクルスクリプトのラッパー（成功/失敗を記録、失敗時は最大2回再試行）"""
        max_attempts = 3
        for attempt in range(max_attempts):
            success = self._run_lifecycle_script(agt_id)
            if success:
                self._lifecycle_success[agt_id] = True
                return
            
            if attempt < max_attempts - 1:
                wait_time = 5.0 * (attempt + 1)  # 指数バックオフ: 5秒、10秒
                self.get_logger().warn(
                    f'ライフサイクル遷移失敗: {agt_id} '
                    f'({attempt + 1}/{max_attempts}回目) '
                )
                time.sleep(wait_time)
        
        self._lifecycle_success[agt_id] = False
    
    def _run_lifecycle_script(self, agt_id):
        """ライフサイクルスクリプトを実行し、成功/失敗を返す"""
        try:
            ok = self._run_lifecycle_via_rclpy(agt_id)
            if ok:
                self.get_logger().info(f'{agt_id}: のライフサイクル遷移に成功')
            else:
                self.get_logger().error(f'{agt_id}: のライフサイクル遷移に失敗')
            return ok
        except Exception as e:
            self.get_logger().error(f'{agt_id}: のライフサイクル遷移で例外が発生: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            return False

    def _call_lifecycle_get(self, temp_node, node_name, timeout_sec=5.0):
        """ライフサイクル状態を取得"""
        from lifecycle_msgs.srv import GetState

        svc_name = f'{node_name}/get_state'
        client = None
        try:
            client = temp_node.create_client(GetState, svc_name)
            
            if not client.wait_for_service(timeout_sec=timeout_sec):
                temp_node.get_logger().debug(f'{svc_name} サービスの応答なし')
                return None

            # リクエスト送信
            req = GetState.Request()
            fut = client.call_async(req)
            
            # Futureの完了を待つ
            start_time = time.time()
            while not fut.done() and (time.time() - start_time) < timeout_sec:
                with self._rclpy_lock:
                    rclpy.spin_once(temp_node, timeout_sec=0.1)
            
            if not fut.done():
                return None
                
            resp = fut.result()
            return getattr(resp.current_state, 'label', None)
            
        except Exception as e:
            temp_node.get_logger().debug(f'サービス {svc_name} の呼び出しに失敗: {e}')
            return None
        finally:
            if client is not None:
                temp_node.destroy_client(client)

    def _call_lifecycle_change(self, temp_node, node_name, transition_id, timeout_sec=20.0):
        """ライフサイクル状態を変更"""
        from lifecycle_msgs.srv import ChangeState
        from lifecycle_msgs.msg import Transition

        svc_name = f'{node_name}/change_state'
        client = None
        try:
            client = temp_node.create_client(ChangeState, svc_name)
            
            if not client.wait_for_service(timeout_sec=timeout_sec):
                temp_node.get_logger().debug(f'{svc_name} サービスの応答なし')
                return False

            # リクエスト送信
            req = ChangeState.Request()
            t = Transition()
            t.id = int(transition_id)
            req.transition = t
            fut = client.call_async(req)
            
            # Futureの完了を待つ
            start_time = time.time()
            while not fut.done() and (time.time() - start_time) < timeout_sec:
                with self._rclpy_lock:
                    rclpy.spin_once(temp_node, timeout_sec=0.1)
            
            if not fut.done():
                return False
                
            resp = fut.result()
            return getattr(resp, 'success', False)
            
        except Exception as e:
            temp_node.get_logger().debug(f'サービス {svc_name} の呼び出しに失敗: {e}')
            return False
        finally:
            if client is not None:
                temp_node.destroy_client(client)

    def _run_lifecycle_via_rclpy(self, agt_id: str) -> bool:
        """ライフサイクル遷移をrclpyサービス経由で実行"""
        from lifecycle_msgs.msg import Transition as LMTransition

        # このスレッドのライフサイクル管理用に一時的なノードを作成
        temp_node = None
        try:
            temp_node = rclpy.create_node(f'lifecycle_mgr_{agt_id}_{int(time.time() * 1000)}')
            
            ns = f'/{agt_id}'
            # nodes and desired transitions in order (node_suffix, transition_name)
            sequence = [
                ('/global_costmap/global_costmap', 'configure'),
                ('/local_costmap/local_costmap', 'configure'),
                ('/planner_server', 'configure'),
                ('/controller_server', 'configure'),
                ('/behavior_server', 'configure'),
                ('/bt_navigator', 'configure'),

                ('/behavior_server', 'activate'),
                ('/bt_navigator', 'activate'),

                ('/controller_server', 'activate'),
                ('/global_costmap/global_costmap', 'activate'),
                ('/local_costmap/local_costmap', 'activate'),
                ('/planner_server', 'activate'),
            ]

            # ライフサイクル遷移名とIDの対応表
            transition_map = {
                'configure': LMTransition.TRANSITION_CONFIGURE if hasattr(LMTransition, 'TRANSITION_CONFIGURE') else 1,
                'activate': LMTransition.TRANSITION_ACTIVATE if hasattr(LMTransition, 'TRANSITION_ACTIVATE') else 3,
            }

            # 全体のタイムアウト: 大規模ロボット数に対応してリトライ回数と待機時間を増やす
            for node_suffix, trans_name in sequence:
                node_full = ns + node_suffix
                state = None
                # サービス呼び出し
                for _ in range(10):
                    state = self._call_lifecycle_get(temp_node, node_full, timeout_sec=8.0)
                    if state is not None:
                        break
                    time.sleep(1.5)
                if state is None:
                    temp_node.get_logger().error(f'{node_full} の状態取得に失敗')
                    return False
    
                # 遷移が必要か確認
                need_transition = False
                if trans_name == 'configure' and state == 'unconfigured':
                    need_transition = True
                if trans_name == 'activate' and state == 'inactive':
                    need_transition = True

                # 遷移が不要なら次のシーケンスへ
                if not need_transition:
                    continue

                # 遷移IDを取得
                trans_id = transition_map.get(trans_name)
                if trans_id is None:
                    return False

                success = self._call_lifecycle_change(temp_node, node_full, trans_id, timeout_sec=20.0)
                if not success:
                    temp_node.get_logger().error(f'{node_full} の状態変更に失敗')
                    return False
                time.sleep(1.0)

            return True
            
        finally:
            if temp_node is not None:
                temp_node.destroy_node()

    def _is_all_lifecycle_successful(self):
        """全ロボットのライフサイクル遷移が成功したか確認"""
        result = all(self._lifecycle_success.values())
        if not result:
            failed_robots = [agt_id for agt_id, success in self._lifecycle_success.items() if not success]
            self.get_logger().error(
                f'次のロボットのライフサイクル正常化に失敗しました: {failed_robots}. '
            )
        return result

    def _wait_for_normalize_tf(self):
        """
        tfが正常化されるまで待機(ブロッキング)する関数
        """
        tfs = None

        # 全ロボットのtfが正常化されるまで繰り返す
        while True:

            # すべて正常化されたかのフラグ
            all_normalized = True

            # tf更新のためspin
            with self._rclpy_lock:
                for _ in range(100):
                    rclpy.spin_once(self, timeout_sec=0.1)

            # 最新のtfを取得
            with self.tfs_lock: 
                tfs = self.latest_tfs.copy()
            
            # 各ロボットのtfを確認
            for agt_id in self.agents:
                tf = tfs.get(agt_id, None)
                initial_pose = self.initial_poses[agt_id]
                # tfが取得できない、または許容範囲外の場合はinitialposeを再送信
                if tf is None:
                    all_normalized = False
                    self._publish_initial_pose(agt_id, initial_pose)
                    continue
                if not self._in_torerance_range(tf, initial_pose):
                    all_normalized = False
                    self._publish_initial_pose(agt_id, initial_pose)

            # すべて正常化された場合はループを抜ける
            if all_normalized:         
                break

    def _set_all_goals(self):
        """
        全ロボットのゴール位置を設定（lifecycle遷移完了後に呼び出す）

        Returns
        -------
        success : bool
            全ロボットのゴール設定が成功した場合にTrueを返す
        """
        # lifecycle遷移の成功を確認
        if not all(self._lifecycle_success.values()):
            failed_robots = [agt_id for agt_id, success in self._lifecycle_success.items() if not success]
            self.get_logger().error(
                f'次のロボットのライフサイクル正常化に失敗したためゴールを登録できません'
                f'{failed_robots}. '
            )
            # ライフサイクル正常化に失敗した場合にはFalseを返す
            return False

        # 全ロボットのゴールを送信
        for agt_id in self.agents:
            g_pose = self.goal_pose_dict[agt_id]
            self._send_nav_goal_action(agt_id, g_pose.x, g_pose.y, g_pose.yaw)
        
        return True

    def _cleanup_zombie_processes(self):
        """ゾンビプロセスをクリーンアップする関数"""

        kill_spawn_cmd = ["pkill", "-f", "/opt/ros/jazzy/lib/ros_gz_sim/create"]

        try:
            subprocess.run(kill_spawn_cmd , check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,)
        except Exception:
            pass
        
        return

    def _send_spawn_request(self, name: str, xml: str, pose: Pose):
        """
        gazeboにスポーンリクエストを送信する関数

        Parameters
        ----------
        name : str
            スポーンするエンティティの名前
        xml : str
            SDFまたはURDF形式のエンティティ定義
        pose : Pose
            スポーン位置姿勢を表現するオブジェクト

        Returns
        -------
        success : bool
            スポーン成功時にTrueを返す
        """
        req = SpawnEntity.Request()
        
        # エンティティ情報の設定
        req.entity_factory.name = name
        req.entity_factory.sdf = xml 
        req.entity_factory.relative_to = "world"
        req.entity_factory.allow_renaming = False 

        # Poseの設定
        q = quaternion_from_yaw(pose.yaw)
        req.entity_factory.pose.position.x = float(pose.x)
        req.entity_factory.pose.position.y = float(pose.y)
        req.entity_factory.pose.position.z = float(pose.z)
        req.entity_factory.pose.orientation.x = q.x
        req.entity_factory.pose.orientation.y = q.y
        req.entity_factory.pose.orientation.z = q.z
        req.entity_factory.pose.orientation.w = q.w

        # サービスコール
        future = self.spawn_client.call_async(req)
        with self._rclpy_lock:
            rclpy.spin_until_future_complete(self, future, timeout_sec=SPAWN_TIMEOUT)
        try:
            # タイムアウト付きで結果を待つ
            response = future.result()
            if response.success:
                return True
            else:
                # Gazebo側でエラー理由が返ってこない場合もあるがログは出す
                self.get_logger().warn(f"スポーン失敗 ({name})")
                return False
        except Exception as e:
            self.get_logger().error(f"サービスコールエラー ({name}): {e}")
            return False

    def _spawn_obstacle(self, model_name: str, obs_size: dict, pose: Pose):
        """
        障害物をスポーン
        
        Parameters
        ----------
        model_name : str
            障害物のモデル名
        obs_size : dict
            障害物のサイズ辞書（x, y, zキーを持つ）
        pose : Pose
            スポーン位置姿勢を表現するオブジェクト

        Returns
        -------
        success : bool
            スポーン成功時にTrueを返す
        """

        # 障害物のSDF定義を作成
        obstacle_xml = f"""<?xml version='1.0'?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="{model_name}_link">
      <collision name="{model_name}_collision">
        <geometry><box><size>{obs_size['x']} {obs_size['y']} {obs_size['z']}</size></box></geometry>
      </collision>
      <visual name="{model_name}_visual">
        <geometry><box><size>{obs_size['x']} {obs_size['y']} {obs_size['z']}</size></box></geometry>
        <material><ambient>1 0 0 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>
"""
        return self._send_spawn_request(model_name, obstacle_xml, pose)

    def _spawn_all_obstacles(self):
        """
        全障害物を順番にスポーン
        """

        for obs_id, pose in self.obstacle_poses.items():
            model_name = obs_id
            obs_size = {'x': 0.6, 'y': 0.6, 'z': 1.0} 
            success = self._spawn_obstacle(model_name, obs_size, pose)
            if not success:
                self.get_logger().error(f"致命的エラー: {model_name} のスポーンを諦めました")
            time.sleep(0.1)

    def _delete_all_obstacles(self):
        """
        全障害物を順番に削除
        """

        for obs_id in self.obstacle_poses.keys():
            model_name = obs_id
            success = self._delete_model(model_name)
            if not success:
                self.get_logger().warn(f"{model_name} の削除に失敗しました")
            time.sleep(0.1)

    def _spawn_model(self, agt_id: str, pose: Pose):
        """
        ロボットをスポーン
        
        Parameters
        ----------
        agt_id : str
            エージェントID
        pose : Pose
            スポーン位置姿勢を表現するオブジェクト

        Returns
        -------
        success : bool
            スポーン成功時にTrueを返す
        """
        file_path = os.path.join(sdf_dir, f"{agt_id}.sdf")
        
        if not os.path.exists(file_path):
            self.get_logger().error(f"ファイルなし: {file_path}")
            return False

        try:
            with open(file_path, 'r') as f:
                model_xml = f.read()
        except Exception as e:
            self.get_logger().error(f"読込エラー: {e}")
            return False

        return self._send_spawn_request(agt_id, model_xml, pose)

    def _delete_model(self, model_name: str):
        """
        モデル削除
        
        Parameters
        ----------
        model_name : str
            削除するモデル名

        Returns
        -------
        success : bool
            削除成功時にTrueを返す
        """
        req = DeleteEntity.Request()
        
        # 削除対象のエンティティ情報の設定
        req.entity.name = model_name
        req.entity.type = Entity.MODEL 
        

        future = self.delete_client.call_async(req)
        with self._rclpy_lock:
            rclpy.spin_until_future_complete(self, future, timeout_sec=SPAWN_TIMEOUT)
        
        try:
            response = future.result() 
            
            if response.success:
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f"削除エラー ({model_name}): {e}")
            return False
    
    def _spawn_model_wrapper(self, agt_id: str, pose: Pose):
        """
        リトライ機能付きスポーン
        
        Parameters
        ----------
        agt_id : str
            エージェントID
        pose : Pose
            スポーン位置姿勢を表現するオブジェクト

        Returns
        -------
        success : bool
            スポーン成功時にTrueを返す
        """
        max_attempts = 2
        for i in range(max_attempts):
            if self._spawn_model(agt_id, pose):
                return True
            if i < max_attempts - 1:
                self.get_logger().warn(f'{agt_id} 再試行中...')
                time.sleep(0.5)
        return False
    
    def _spawn_all_models(self):
        """
        全ロボットを順番にスポーン
        """

        for agt_id in self.possible_agents:
            s_pose = self.initial_poses[agt_id]
            success = self._spawn_model_wrapper(agt_id, s_pose)
            if not success:
                self.get_logger().error(f"致命的エラー: {agt_id} のスポーンを諦めました")
            time.sleep(0.1)

    def _delete_all_models(self):
        """
        全ロボットを順番に削除
        """

        for agt_id in self.possible_agents:
            success = self._delete_model(agt_id)
            if not success:
                self.get_logger().warn(f"{agt_id} の削除に失敗しました")
            time.sleep(0.1)