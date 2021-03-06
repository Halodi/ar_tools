from setuptools import setup

import os
from glob import glob

package_name = 'ar_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='james@halodi.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_zed = ar_tools.aruco_broadcaster:zed',
            'aruco_grpc = ar_tools.aruco_broadcaster:grpc',
            'stf_server = ar_tools.stf_server:main',
            'calibration_motion_head = ar_tools.calibration_motions:head',
            'calibration_extrinsic_head = ar_tools.extrinsic_calibration_head:main'
        ],
    },
)
