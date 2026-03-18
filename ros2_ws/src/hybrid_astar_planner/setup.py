from setuptools import setup, find_packages

package_name = 'hybrid_astar_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anmol Sureka',
    maintainer_email='anmolsureka006@gmail.com',
    description='Hybrid A* path planner ROS 2 node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hybrid_astar_node = hybrid_astar_planner.nodes.planner_node:main',
        ],
    },
)
