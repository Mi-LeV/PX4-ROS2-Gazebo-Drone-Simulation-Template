from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cre',
    maintainer_email='cre@todo.todo',
    description='TODO: Package description',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_ctrl_example = image_processing.offboard_ctrl_example:main',
            'pub_cam = image_processing.pub_cam:main',
            'aruco_node = image_processing.aruco_node:main',
            'line_node = image_processing.line_recognition:main',
        ],
    },
)
