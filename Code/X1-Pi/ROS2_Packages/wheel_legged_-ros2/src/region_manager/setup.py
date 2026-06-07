from setuptools import setup
import os
from glob import glob

package_name = 'region_manager'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aidlux',
    maintainer_email='aidlux@example.com',
    description='区域导航管理 — 矩形区域划分、持久化存储、导航点发布',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'region_manager = region_manager.region_manager:main',
        ],
    },
)
