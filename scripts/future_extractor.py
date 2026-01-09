import torch
import torch.nn as nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

class Nav2CombinedExtractor(BaseFeaturesExtractor):
    """
    フラットな観測ベクトルを受け取り、内部で画像と数値に分割して処理するクラス
    """
    def __init__(self, observation_space, features_dim=256):
        # features_dim: CNNと数値を結合した後の出力次元数
        super().__init__(observation_space, features_dim)

        # 定数の定義（環境側の設定に合わせてください）
        self.img_h, self.img_w = 30, 30
        self.n_stack_frames = 1  # スタックするCostmapの枚数

        self.img_size = self.img_h * self.img_w * self.n_stack_frames  # 900
        
        # 入力全体の次元数
        total_dim = observation_space.shape[0]
        # 数値データの次元数 (全体 - 画像)
        self.vec_dim = total_dim - self.img_size

        # --- CNN層の定義 (画像用) ---
        self.cnn = nn.Sequential(
            # 入力: (Batch, 1, 30, 30)
            nn.Conv2d(self.n_stack_frames, 32, kernel_size=3, stride=2),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.Flatten(),
        )

        # CNNの出力サイズを計算
        with torch.no_grad():
            # ダミー入力でサイズ確認
            sample_img = torch.zeros(1, self.n_stack_frames, self.img_h, self.img_w)
            n_flatten = self.cnn(sample_img).shape[1]

        # --- 結合層 ---
        # CNNの出力 + 数値データ -> features_dim
        self.linear = nn.Sequential(
            nn.Linear(n_flatten + self.vec_dim, features_dim),
            nn.ReLU()
        )

    def forward(self, observations):
        # observations は shape (Batch, Total_Dim) のテンソル

        # 1. スライスして分割
        # 前半 3600個 が画像
        img_part = observations[:, :self.img_size]
        # 残りが数値データ
        vec_part = observations[:, self.img_size:]

        # 2. 画像部分をリシェイプ (Batch, Channel, H, W)
        # Costmapは0-255の値なので、0.0-1.0に正規化すると学習が安定します
        img_part = img_part.reshape(-1, self.n_stack_frames, self.img_h, self.img_w)

        img_part = img_part / 255.0  # 正規化

        # 3. CNNに通す
        cnn_out = self.cnn(img_part)

        # 4. 数値データと結合
        combined = torch.cat((cnn_out, vec_part), dim=1)

        # 5. 全結合層に通して返す
        return self.linear(combined)