from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
        ('share/' + package_name + '/launch', ['launch/person_follower.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/ament_index/resource_index/packages',
         ['resource/perception']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CodeRyne',
    maintainer_email='ryne.xie@qq.com',
    description='人体跟随 — LiDAR 质心漂移跟踪 + Kalman + Nav2 目标发布',
    license='MIT',
    entry_points={
        'console_scripts': [
            'person_follower = perception.person_follower:main',
        ],
    },
)
