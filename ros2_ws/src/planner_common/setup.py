from setuptools import setup, find_packages

package_name = 'planner_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anmol Sureka',
    maintainer_email='anmolsureka006@gmail.com',
    description='Shared adapters and utilities for path planner ROS 2 packages',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mock_map_node = planner_common.nodes.mock_map_node:main',
            'path_visualizer_node = planner_common.nodes.path_visualizer_node:main',
        ],
    },
)
