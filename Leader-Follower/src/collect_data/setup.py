from setuptools import find_packages, setup

package_name = 'collect_data'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'range_aoa = collect_data.range_aoa:main',
        	'drone_target = collect_data.drone_target:main',
        ],
    },
)
