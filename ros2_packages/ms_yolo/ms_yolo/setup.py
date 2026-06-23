from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ms_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['ms_yolo', 'ms_yolo.*']),
    py_modules=[
        'ms_yolo.yolo_pub',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='YOLO integration package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ms_yolo = ms_yolo.yolo_pub:main',
        ],
    },
)
