import os
import argparse
from ament_index_python.packages import get_package_share_directory

world_pkg_share_dir = get_package_share_directory('world_xacro_creator')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=int, required=True, help='マップの幅（ピクセル数）')
    parser.add_argument('--height', type=int, required=True, help='マップの高さ（ピクセル数）')
    parser.add_argument('--world_name', type=str, default='map', help='出力PGMファイル名')
    parser.add_argument('--resolution', type=float, default=0.05, help='マップの解像度（メートル/ピクセル）')
    args = parser.parse_args()

    yaml_path = os.path.join(world_pkg_share_dir, 'maps', f'{args.world_name}.yaml')
    origin = {
        'x': - (args.width / 2.0),
        'y': - (args.height / 2.0),
        'z': 0.0
    }

    try:
        with open(yaml_path, 'w') as f:
            f.write(f"image: {args.world_name}.pgm\n")
            f.write(f"resolution: {args.resolution}\n")
            f.write(f"origin: [{origin['x']}, {origin['y']}, {origin['z']}]\n")
            f.write(f"negate: 0\n")
            f.write(f"occupied_thresh: 0.65\n")
            f.write(f"free_thresh: 0.196\n")
        print(f"マップYAMLファイルを生成しました: {yaml_path}")
    except Exception as e:
        print(f"エラー: マップYAMLファイルの生成に失敗しました: {e}")

if __name__ == "__main__":
    main()