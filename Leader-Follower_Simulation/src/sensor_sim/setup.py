import os
from glob import glob
from setuptools import setup

from setuptools import find_packages, setup

package_name = 'sensor_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giorgio',
    maintainer_email='giorgio.pellegrini@studenti.unitn.it',
    description='Simulated UWB Sensor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'sensor_sim = sensor_sim.sensor_sim:main'
        ],
    },
)
