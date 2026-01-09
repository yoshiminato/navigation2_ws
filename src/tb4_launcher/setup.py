from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb4_launcher'

# rvizファイルのリストを取得
rviz_files = glob(os.path.join('rviz', '*.rviz'))
rviz_data = [(os.path.join('share', package_name, 'rviz'), rviz_files)] if rviz_files else []

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch ファイルを含める
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), 
        # resource ディレクトリを含める
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        # paramsディレクトリ内のすべての.yamlファイルをインストール
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
        # configディレクトリ内のすべての.yamlファイルをインストール
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # urdfディレクトリ内のすべてのファイルを再帰的にインストール
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '**', '*.xacro'), recursive=True)),
        (os.path.join('share', package_name, 'urdf', 'icreate'), glob(os.path.join('urdf', 'icreate', '*.xacro'))),
        (os.path.join('share', package_name, 'urdf', 'icreate', 'sensors'), glob(os.path.join('urdf', 'icreate', 'sensors', '*.xacro'))),
        (os.path.join('share', package_name, 'urdf', 'standard'), glob(os.path.join('urdf', 'standard', '*.xacro'))),
        (os.path.join('share', package_name, 'urdf', 'sensors'), glob(os.path.join('urdf', 'sensors', '*.xacro'))),
        # meshesディレクトリ内のすべてのファイルをインストール
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.dae'))),
    ] + rviz_data,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoshi',
    maintainer_email='yoshimitsuminatoo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
