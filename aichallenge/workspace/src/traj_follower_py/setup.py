# /aichallenge/workspace/src/traj_follower_py/setup.py
from setuptools import setup, find_packages
from glob import glob

package_name = 'traj_follower_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/trajectory', glob('trajectory/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='traj follower',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 左が ros2 run の <exe> 名、右が モジュール:関数
            'traj_follower = traj_follower_py.traj_follower_node:main',
        ],
    },
)



