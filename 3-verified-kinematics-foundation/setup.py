import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'Rsine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'analysis']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'docs'),
            glob('docs/*.md') + glob('docs/*.pdf') + glob('docs/*.pptx')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='royabhishek4650roy@gmail.com',
    description='myCobot 280 sine-wave tracer: modified-DH kinematics, velocity-propagation Jacobian, DLS IK, RViz visualization.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sine_ik_node = Rsine.sine_ik_node:main',
            'path_tracer_node = Rsine.path_tracer_node:main',
        ],
    },
)
