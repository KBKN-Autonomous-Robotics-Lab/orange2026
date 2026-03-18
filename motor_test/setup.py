from setuptools import find_packages, setup
# osパッケージに含まれる関数群を使用可能にする
import os
# globパッケージからglob関数を使用可能にする
from glob import glob

package_name = 'motor_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # Launch関連をインストールするためにリストアップ
        (os.path.join('share', package_name), glob('./launch/*.launch.xml')),
        # During installation, we need to copy the launch files
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        # Same with the RViz configuration file.
        (os.path.join('share', package_name, "config"), glob('config/*')),
        # アクションファイルのインストール
        (os.path.join('share', package_name), glob('action/*.action')),  
    ],
   # setuptoolsを使ってこのパッケージをインストールすることを指定
    install_requires=['setuptools'],
    zip_safe=True,
    # パッケージのメンテナ（動かないときに連絡窓口になるひと）の名前
    maintainer='hosei',
    # メンテナーの連絡先
    maintainer_email='hosei',
    # パッケージの説明
    description='try navgation for ROS 2.',
    # パッケージのライセンスを指定
    license='Apache-2.0',
    # 単体テストのため依存を追加
    tests_require=['pytest'],
    # ros2 runコマンドやros2 launchコマンドでノードを起動できるようにするための設定。
    # ここを忘れていると実行ができません。
    entry_points={
        'console_scripts': [
        'path_follower_test = motor_test.path_follower_test:main',
        "motor_driver_node = motor_test.motor_driver_node:main",
        "radius_comp_test = motor_test.radius_comp_test:main",
        "rad_vel_test = motor_test.rad_vel_test:main"
        ],
    },
)
