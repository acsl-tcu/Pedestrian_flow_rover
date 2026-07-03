from setuptools import setup
from glob import glob
import os

package_name = 'ms_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],  # Python モジュールなし
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # ここが必要
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='Launch files for MovingSignage',
    license='TODO: License declaration',
)
