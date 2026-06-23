from setuptools import setup, find_packages
import os

package_name = 'ms_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['ms_robot', 'ms_robot.*']),
    py_modules=[
        'ms_robot.cbf',
        'ms_robot.graph',
        'ms_robot.ricoh',
        'ms_robot.robot',
        'ms_robot.rover',
        'ms_robot.rplidar_sub',
        'ms_robot.servo',
        'ms_robot.tscf',
        'ms_robot.yolo_sub',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ms_robot']),
        ('share/ms_robot', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='Robot control package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ms_robot = ms_robot.robot:main',
        ],
    },
)
