import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alkady',
    maintainer_email='youremail@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = control.nodes.IMUNode:main',
            'depth_node = control.nodes.DepthNode:main',
            'temp_node = control.nodes.TempNode:main',
            'ms5_node = control.nodes.Ms5Node:main',
            'echo_node = control.nodes.EchoNode:main',
            'pca_node = control.nodes.PCANode:main',
            'joy_listener = control.nodes.JoyListenerNode:main',
        ],
    },
)
