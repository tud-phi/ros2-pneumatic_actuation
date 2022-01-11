import os
from glob import glob
from setuptools import setup

package_name = 'pneumatic_actuation_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'pygbn'
    ],
    zip_safe=True,
    maintainer='Maximilian Stoelzle',
    maintainer_email='maximilian@stoelzle.ch',
    description='Demos for pneumatic actuation of soft robotic arms.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pressure_trajectory_node = pneumatic_actuation_demos.pressure_trajectory_node:main',
        ],
    },
)
