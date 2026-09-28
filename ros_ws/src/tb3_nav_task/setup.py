from glob import glob
from setuptools import find_packages, setup

package_name = 'tb3_nav_task'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mainuddin Talukdar',
    maintainer_email='mainuddin.talukdar.2024@gmail.com',
    description='COSC471 Task 02 TurtleBot3 goal navigation (VFH-lite vs Bug2) with evaluation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = tb3_nav_task.navigator:main',
            'trial_monitor = tb3_nav_task.trial_monitor:main',
            'sim2d = tb3_nav_task.sim2d:main',
        ],
    },
)
