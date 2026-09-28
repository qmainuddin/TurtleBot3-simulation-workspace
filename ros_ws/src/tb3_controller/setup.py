from glob import glob
from setuptools import find_packages, setup

package_name = 'tb3_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mainuddin Talukdar',
    maintainer_email='mainuddin.talukdar.2024@gmail.com',
    description='Reactive LiDAR obstacle-avoidance controller for a simulated TurtleBot3.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoider = tb3_controller.obstacle_avoider:main',
            'scan_monitor = tb3_controller.scan_monitor:main',
        ],
    },
)
