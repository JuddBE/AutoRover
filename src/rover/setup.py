from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rover'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*')),
        # ('share/' + package_name + '/rover_description', glob('rover_description/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juddbe',
    maintainer_email='juddbe@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization = rover.localization:main',
            'mapping = rover.mapping:main',
            'path_planning = rover.path_planning:main',
            'path_tracking = rover.path_tracking:main',
            'controller = rover.controller:main'
        ],
    },
)
