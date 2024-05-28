from setuptools import find_packages
from setuptools import setup

setup(
    name='mymsg_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('mymsg_msgs', 'mymsg_msgs.*')),
)
