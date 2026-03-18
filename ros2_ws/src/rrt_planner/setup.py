from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rrt_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anmol Sureka',
    maintainer_email='anmolsureka006@gmail.com',
    description='ROS 2 wrapper for RRT* family planners',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rrt_planner_node = rrt_planner.nodes.rrt_planner_node:main',
        ],
    },
)
