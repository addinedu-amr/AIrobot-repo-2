from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot4_custom'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/pt_files', glob(os.path.join('pt_files', '*.pt')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwj',
    maintainer_email='jwj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'behavior_decision_server = turtlebot4_custom.behavior_decision_server:main',
        'object_detector = turtlebot4_custom.object_detection:main',
        'lane_detector = turtlebot4_custom.lane_detection:main',
        ],
    },
)
