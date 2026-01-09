
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
import math
import re
import os
from typing import Optional


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
sdf_dir = os.path.join(HOME, "nav2_ws/sdf_dumps/")
target_dir = os.path.join(HOME, "nav2_ws/simulation_logs/")
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
    