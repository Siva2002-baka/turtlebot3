import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'turtlebot3', 'launch'),glob('launch/*.py')),
        (os.path.join('share', 'turtlebot3', 'urdf'),glob('urdf/*.*')),
        (os.path.join('share', 'turtlebot3', 'config'),glob('config/*.*')),
        (os.path.join('share', 'turtlebot3', 'meshes','bases'),glob('meshes/bases/*.stl')),
        (os.path.join('share', 'turtlebot3', 'meshes','sensors'),glob('meshes/sensors/*.*')),
        (os.path.join('share', 'turtlebot3', 'meshes','wheels'),glob('meshes/wheels/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siva',
    maintainer_email='saikumar14970@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtlebot_controller = turtlebot3.turtlebot3_controller_node:main',
            'teleop_node = turtlebot3.teleop_node:main',
        ],
    },
)
