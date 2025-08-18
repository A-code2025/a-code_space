from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'traj_follower_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # パッケージ登録に必要
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ↓ これで install/share/traj_follower_py/launch に入る
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # ↓ config もコピー
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # 任意: 軌道CSVなど静的データ
        ('share/' + package_name + '/trajectory', glob('trajectory/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='traj follower',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # `ros2 run traj_follower_py traj_follower`
            'traj_follower = traj_follower_py.traj_follower:main',
        ],
    },
)

