
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
import math
import re
import os
from typing import Optional
import tf2_ros
from rclpy.node import Node
import numpy as np
from scipy import signal


def quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

def quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

_PATTERN = re.compile(r"^robot_(\d+)$")

# def is_robot_id(s: str) -> bool:
#     return bool(_PATTERN.fullmatch(s))


def extract_robot_number(s: str) -> Optional[int]:
    m = _PATTERN.fullmatch(s)
    if not m:
        return None
    try:
        return int(m.group(1))
    except (ValueError, TypeError):
        return None
    


def normalize_angle(angle: float) -> float:
    """
    正規化された角度を返す[-pi, pi]
    """
    if angle is None:
        return None
    # atan2(sin, cos) を使うと数値的に安定して [-pi, pi] に丸められます
    try:
        return float(math.atan2(math.sin(float(angle)), math.cos(float(angle))))
    except Exception:
        # 万一変換できなければ元の値を返す
        return angle


    
HOME = os.path.expanduser('~')
sdf_dir = os.path.join(HOME, "navigation2_ws/sdf_dumps/")
target_dir = os.path.join(HOME, "navigation2_ws/simulation_logs/")
sim_base_name  = "simulation"
error_base_name = "error"
ext = ".log"

def get_unique_log_file_path() -> str:
    counter = 1
    sim_filename = f"{sim_base_name}_{counter}{ext}"
    error_filename = f"{error_base_name}_{counter}{ext}"

    simpath = os.path.join(target_dir, sim_filename)
    errorpath = os.path.join(target_dir, error_filename)     
    
    while os.path.exists(simpath) or os.path.exists(errorpath):
        counter += 1
        sim_filename = f"{sim_base_name}_{counter}{ext}"
        error_filename = f"{error_base_name}_{counter}{ext}"
        simpath = os.path.join(target_dir, sim_filename)
        errorpath = os.path.join(target_dir, error_filename)
    return simpath, errorpath

class Pose:
    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def __repr__(self):
        return f"Pose(x={self.x}, y={self.y}, z={self.z}, yaw={self.yaw})"
    
    def copy(self):
        return Pose(self.x, self.y, self.z, self.yaw)
    

def create_gaussian_kernel(size, sigma):
    """
    size: カーネルのサイズ (奇数が推奨, 例: 3, 5, 7...)
    sigma: 標準偏差
    """
    # 1次元のガウス分布を作成
    g = signal.windows.gaussian(size, std=sigma)
    # 2次元に拡張 (外積をとる)
    kernel = np.outer(g, g)
    # 合計が1になるように正規化 (任意ですが、画像処理等では一般的)
    kernel /= kernel.sum()
    return kernel
# # 畳み込み演算
# def convolution2d(img, kernel):
#     m, n = kernel.shape # カーネルサイズ取得
#     # カーネル中心からみた幅
#     dy = int((m-1)/2)  # カーネル上下幅
#     dx = int((n-1)/2)  # カーネル左右幅
#     h, w = img.shape   # イメージサイズ
#     out = np.zeros((h, w)) # 出力用イメージ
#     # 畳み込み
#     for y in range(dy, h - dy):
#         for x in range(dx, w - dx):
#             out[y][x] = np.sum(img[y-dy:y+dy+1, x-dx:x+dx+1]*kernel)
#     return out
def apply_kernel_at_center(img, kernel):
    """
    入力画像の中心画素に対して、カーネルを用いた積和演算（畳み込みの1ステップ）を行う関数。
    
    Args:
        img (np.ndarray): 入力画像 (2次元配列)
        kernel (np.ndarray): フィルタ/カーネル (2次元配列, 奇数サイズ推奨)
        
    Returns:
        float: 積和演算の結果（スカラー値）
    """
    # カーネルサイズ取得
    k_h, k_w = kernel.shape
    
    # 画像サイズ取得
    img_h, img_w = img.shape
    
    # カーネルの中心オフセット（半径）を計算
    dy = k_h // 2
    dx = k_w // 2
    
    # 画像の中心座標を計算
    center_y = img_h // 2
    center_x = img_w // 2
    
    # 画像からカーネルサイズ分の領域（ROI）を切り出す
    # スライス範囲: 中心 - 半径  ～  中心 + 半径 + 1
    # 注意: 画像サイズがカーネルより小さい場合のエラー処理は省略しています
    roi = img[center_y - dy : center_y + dy + 1, 
              center_x - dx : center_x + dx + 1]
    
    # 切り出した領域とカーネルの要素ごとの積をとり、総和を計算
    result = np.sum(roi * kernel)
    
    return result