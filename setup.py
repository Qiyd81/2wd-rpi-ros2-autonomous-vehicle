from setuptools import setup

package_name = 'lidar_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chinedu Amadi',
    maintainer_email='chineduamadi@protonmail.com',
    description='A package for a 2 wheeled robot fitted with a RPLIDAR A1 sensor for obstacle avoidance and SLAM operation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = lidar_robot.simple_motor_subscriber:main'
        ],
    },
)
