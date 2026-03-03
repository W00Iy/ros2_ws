from setuptools import find_packages, setup

import os
from glob import glob


package_name = 'joint_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/joint_description']),
        ('share/joint_description', ['package.xml']),
        ('share/joint_description/urdf', ['urdf/joint_model.urdf']),
        ('share/joint_description/launch', ['launch/view_model.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wooly',
    maintainer_email='benjambd@stud.ntnu.no',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
