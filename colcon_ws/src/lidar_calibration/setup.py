from setuptools import setup

package_name = 'lidar_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for LiDAR calibration',
    license='TODO: License declaration',
    entry_points={
    'console_scripts': [
        'lidar_calibration_node = lidar_calibration.lidar_calibration_node:main',
        'uncalibrated_node = lidar_calibration.uncalibrated_node:main',
    ],
},
)
