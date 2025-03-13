from setuptools import setup
import os
from glob import glob

package_name = 'xbox_joy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Xbox Controller를 사용하여 ROS2로 이동 명령을 전송하는 노드',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_teleop = xbox_joy.joystick_teleop:main',
        ],
    },

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),  # launch 파일도 포함 가능
    ],
)
