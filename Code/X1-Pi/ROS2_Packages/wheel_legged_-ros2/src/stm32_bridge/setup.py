from setuptools import setup
import os
from glob import glob

package_name = 'stm32_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='aidlux',
    maintainer_email='aidlux@example.com',
    description='轮足机器人 STM32 USB CDC 串口通讯桥接节点',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wheel_foot_bridge = stm32_bridge.wheel_foot_bridge:main',
        ],
    },
)
