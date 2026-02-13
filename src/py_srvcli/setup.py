from setuptools import find_packages, setup

package_name = 'py_srvcli'

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
    maintainer='mats',
    maintainer_email='mats@todo.todo',
    description='Service client',
    license='Apache-2.0',
    entry_points={
    'console_scripts': [
        'reference_input_node=py_srvcli.reference_input_node:main',
    ],
},
)

