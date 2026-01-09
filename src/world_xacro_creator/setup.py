from setuptools import find_packages, setup
import os, glob

package_name = 'world_xacro_creator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,        
            ['package.xml']
        ),
        (
            f'share/{package_name}/launch', 
            glob.glob('launch/*.py')
        ),
        (
            f'share/{package_name}/maps',   
            glob.glob('maps/*')
            ),
        (
            f'share/{package_name}/worlds', 
            glob.glob('worlds/*')
        ),
        (
            f'share/{package_name}/configs', 
            glob.glob('configs/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoshi',
    maintainer_email='yoshimitsuminatoo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
