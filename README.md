# 概要
gazebo, navigation2, stablebaselineの連携によりロボットの行動計画への強化学習の適用を実現するシステム.

---

# 構成ファイル
```text

.
├── README.md
├── scripts
│   ├── analyze.ipynb                 # 強化学習結果解析用
│   ├── convert_pgm_to_world.py       # pgmファイル(2値画像:2D)をsdfファイル(環境:3dに変換)
│   ├── create_map_yaml.py            # マップのパラメータファイル生成
│   ├── future_extractor.py           # 強化学習のNN構造定義
│   ├── interactive_pgm_editor.py     # pgmファイルを作成アプリ
│   ├── pettingZoo_test.py            # 強化学習のプロセス起動
│   ├── rec_partition.py              # pgm2sdfにおいて効率的な環境生成のためのアルゴリズム
│   ├── sb3_compatible_wrapper.py     # arkovVectorEnvをSB3互換のVecEnvに変換するラッパー
│   └── visualize                     # 強化学習結果解析用
│       ├── test_visualize.py
│       └── train_visualize.py
│ 
├── src                           
│   ├── nav2_pure_pursuit_controller  # local_plannerプラグイン(公式demoの実装)
│   ├── nav2_rl_controller            # 強化学習用local_plannerプラグイン
│   ├── parallel_controller_env       # 強化学習の各種プロセス統合
│   ├── rl_launcher                   # エントリーポイント
│   ├── tb4_launcher                  # 自律移動プロセス起動
│   └── world_xacro_creator           # 環境生成
│ 
├── urdf_dumps
└── sdf_dumps



```