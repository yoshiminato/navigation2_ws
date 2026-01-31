import sys
sys.path.append('/home/yoshi/navigation2_ws/src')
from parallel_controller_env.parallel_controller_env import Nav2ParallelEnv
from pettingzoo.test import parallel_api_test
from stable_baselines3.common.vec_env import VecMonitor

from future_extractor import Nav2CombinedExtractor

import rclpy
import threading
import matplotlib.pyplot as plt

import matplotlib
matplotlib.use('Agg')

import numpy as np
import math
import argparse
import time
import gymnasium as gym

from rclpy.executors import SingleThreadedExecutor

# Stable-Baselines3とSupersuit（依存関係チェック）
try:
    from stable_baselines3 import PPO, DQN, SAC
    from stable_baselines3.common.vec_env import DummyVecEnv, VecEnv
    from stable_baselines3.common.callbacks import CheckpointCallback
    from sb3_compatible_wrapper import SB3CompatibleWrapper, wrap_env_for_sb3
    from stable_baselines3.common.evaluation import evaluate_policy
    HAS_SB3 = True
except ImportError:
    HAS_SB3 = False
    print("Warning: stable-baselines3 not installed. Run: pip install stable-baselines3")

try:
    import supersuit as ss
    HAS_SUPERSUIT = True
except ImportError:
    HAS_SUPERSUIT = False
    print("Warning: supersuit not installed. Run: pip install supersuit")

policy_kwargs = dict(
    net_arch=[256, 256],
    features_extractor_class=Nav2CombinedExtractor,
    features_extractor_kwargs=dict(features_dim=256), # 特徴量の出力次元
)

algorithm_class_map = {
    'DQN': DQN,
    'SAC': SAC,
}

algorithm_action_type_map = {
    'DQN': 'discrete',
    'SAC': 'continuous',
}


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--robot_count', type=int, default=9, help='ロボットの数')
    parser.add_argument('--world_name',  type=str, default='square15', help='worldファイルの名前')
    parser.add_argument('--use_rl', action='store_true', default=False, help='RLを使用するかどうか')
    parser.add_argument('--rl_algorithm', type=str, default='SAC', help='RLアルゴリズムの種類 (DQN, SAC, …)')
    parser.add_argument('--train', action='store_true', help='SAC学習モード')
    parser.add_argument('--timesteps', type=int, default=100000, help='学習ステップ数')
    parser.add_argument('--load_model', type=str, default=None, help='学習済みモデルをロード')
    parser.add_argument('--eval', action='store_true', default=False, help='評価/推論モード（学習しない）')
    parser.add_argument('--eval_episodes', type=int, default=100, help='評価時のエピソード数')
    args = parser.parse_args()

    if not rclpy.ok():
        rclpy.init()

    robot_count = args.robot_count
    world_name = args.world_name
    use_rl = args.use_rl
    alg_name = args.rl_algorithm
    action_type = algorithm_action_type_map.get(alg_name, 'continuous')
    alg_class = algorithm_class_map.get(alg_name, SAC)
    model_path = f"{alg_name.lower()}_nav2_model"

    print(f"アルゴリズム: {alg_name}, アクションタイプ: {action_type}")

    # 環境の作成
    env = Nav2ParallelEnv(robot_count=robot_count, world_name=world_name, use_rl=use_rl, action_type=action_type)

    # 学習モード
    if use_rl:
        if not HAS_SB3 or not HAS_SUPERSUIT:
            print("Error: stable-baselines3 and supersuit are required for training.")
            print("Install: pip install stable-baselines3 supersuit")
            rclpy.shutdown()
            sys.exit(1)
        
        print("=" * 60)
        print(f"{alg_name}学習モード開始")
        print(f"ロボット台数: {robot_count}")
        print(f"学習ステップ数: {args.timesteps}")
        print("=" * 60)

        print("load_model:", args.load_model)
        print("eval:", args.eval)
        print("eval_episodes:", args.eval_episodes)
        print("=" * 60)
        
        # SB3互換に変換
        try:
            wrapped_env = wrap_env_for_sb3(env)
            wrapped_env = VecMonitor(wrapped_env)
        except Exception as e:
            print(f"Error: 環境の変換に失敗しました: {e}")
            import traceback
            traceback.print_exc()
            rclpy.shutdown()
            # spin_thread.join()
            sys.exit(1)

        # モデルの作成またはロード
        if args.load_model:
            print(f"学習済みモデルをロード: {args.load_model}")

            match alg_name:
                case 'DQN':
                    model = alg_class.load(
                        args.load_model, 
                        env=wrapped_env,
                        learning_starts=0,
                        exploration_fraction=0.0,
                        exploration_initial_eps=0.05,  # 1.0ではなく0.1から再開
                        exploration_final_eps=0.05,
                    )
                case 'SAC':
                    model = alg_class.load(
                        args.load_model, 
                        env=wrapped_env,
                        learning_starts=0,
                    )
                    
            try:
                if args.eval:
                    model.load_replay_buffer(f"{args.load_model}")
                print("✓ リプレイバッファをロードしました")
            except Exception as e:
                print(f"Warning: Failed to load replay buffer: {e}")

        else:
            print(f"新規{alg_name}モデルを作成")

            model = None

            match alg_name:
                case 'DQN':
                    model = alg_class(
                        "MlpPolicy",
                        wrapped_env,
                        policy_kwargs=policy_kwargs,
                        learning_rate=1e-4,
                        buffer_size=100000,
                        learning_starts=5000,
                        batch_size=256,
                        tau=0.005,
                        gamma=0.99,
                        train_freq=4,
                        target_update_interval=1000,
                        exploration_fraction=0.1,
                        exploration_initial_eps=1.0,
                        exploration_final_eps=0.05,
                        verbose=1,
                        tensorboard_log=None#f"./logs/{model_path}_tensorboard/"
                    )
                case 'SAC':
                    model = alg_class(
                        "MlpPolicy",
                        wrapped_env,
                        policy_kwargs=policy_kwargs,
                        learning_rate=1e-4,
                        buffer_size=50000,
                        learning_starts=5000,
                        batch_size=256,
                        tau=0.005,
                        gamma=0.99,
                        train_freq=4,
                        gradient_steps=1,
                        target_update_interval=1000,
                        verbose=1,
                        tensorboard_log=None
            )
                    
        
        # チェックポイントコールバック（10000ステップごとに保存）
        checkpoint_callback = CheckpointCallback(
            save_freq=10000,
            save_path=f"./checkpoints/{model_path}/",
            name_prefix=f"{alg_name.lower()}_nav2"
        )
        # 評価／推論モード（学習を行わない）
        if args.eval:
            print("✓ 評価/推論モードに入ります...")
            mean_reward, std_reward = evaluate_policy(
                model, 
                wrapped_env, 
                n_eval_episodes=args.eval_episodes, 
                deterministic=True, # 完全な推論モード（ランダム性なし）
                render=False
            )

            print(f"📊 結果 (Episodes: {args.eval_episodes})")
            print(f"平均報酬: {mean_reward:.2f} +/- {std_reward:.2f}")

            env.close()
            rclpy.shutdown()
            sys.exit(0)

        # 学習開始
        print("学習を開始します...")
        try:
            model.learn(
                total_timesteps=args.timesteps,
                callback=checkpoint_callback,
                log_interval=100
            )
            
            # モデル保存
            model.save(model_path)
            model.save_replay_buffer(f"{model_path}_final_buffer")
            print(f"✓ モデルを保存しました: {model_path}.zip")
            
        except KeyboardInterrupt:
            print("\n学習が中断されました")
            model.save(f"{model_path}_interrupted")
            print(f"✓ 中断時のモデルを保存: {model_path}_interrupted.zip")
            model.save_replay_buffer(f"{model_path}_final_buffer")
        except Exception as e:
            print(f"学習中にエラーが発生: {e}")
            import traceback
            traceback.print_exc()
        
        rclpy.shutdown()
        # spin_thread.join()
        sys.exit(0)
    
    else:
        print("✓ 学習モードではありません。インタラクティブモードで環境を実行します。")
        # インタラクティブモード（デフォルト）
        print("=" * 60)
        print("インタラクティブモード（ランダムアクション）")
        print("=" * 60)

        obs, infos = env.reset()
        for step_idx in range(10**6):
            # ランダムアクションで動きを確保（progress checkerエラー回避）
            actions = {agent: env.action_space(agent).sample() for agent in env.agents}

            obs, rewards, terminations, truncations, infos = env.step(actions)


            # env.render()
            import cv2
            agent_visuals = []
            # タイル表示の設定 (例: 1行に5台並べる)
            cols_count = 4 
            # 各タイルの統一サイズ (Noneの場合は最初の画像に合わせる)
            tile_size = (200, 200) 

            COSTMAP_SIZE = 30
            BUFFER_SIZE = 4

            tile_size = (200, 200)

            for i in range (robot_count):
                agt_id = f"robot_{i+1}"
                # print(obs)
                img_size = COSTMAP_SIZE * COSTMAP_SIZE * BUFFER_SIZE

                agt_obs = None
                try:
                    agt_obs = obs[agt_id]
                except Exception as e:
                    print(f"Error: エージェント {agt_id} の観測データ取得に失敗: {e}")
                    continue
                 
                costmaps = np.array(agt_obs[:img_size].reshape((BUFFER_SIZE, COSTMAP_SIZE, COSTMAP_SIZE)))

                imgs = []
                imgs_color = []
                for b in range(BUFFER_SIZE):
                    imgs.append(np.fliplr(np.flipud(costmaps[b].T)))
                    imgs[b] = imgs[b].astype(np.uint8)
                    imgs_color.append(cv2.cvtColor(imgs[b], cv2.COLOR_GRAY2BGR))
                    imgs_color[b] = cv2.resize(imgs_color[b], tile_size)

                agent_visuals.append(imgs_color)

            # 横に結合
            rows = []
            for i in range(len(agent_visuals)):
                row_img = cv2.hconcat(agent_visuals[i][0:BUFFER_SIZE])
                rows.append(row_img)

            # 縦に結合
            combined_img = cv2.vconcat(rows)

            # 表示
            cv2.imshow("Multi-Agent RL Monitor", combined_img)
            cv2.waitKey(1)


            # すべてのエージェントがterminationsまたはtruncationsのどちらか一方でもTrueならリセット
            if all(t or u for t, u in zip(terminations.values(), truncations.values())):
                env.reset()

        rclpy.shutdown()
