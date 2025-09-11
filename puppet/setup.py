#author by DungTD
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puppet'

#describe how to build package
#initialize metadata, dependencies, entry points, etc.
setup(
    name=package_name,#name of package
    version='0.0.0',#version of package
    packages=find_packages(exclude=['test']), #find all children Python packages except test
    #commmon knowledge for ROS2/launch find appropriate files in install/share/<package_name>/
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),#instal marker file for package, ROS2 uses ament index to locate packages
        # package.xml
        (os.path.join('share', package_name), ['package.xml']),#copy package.xml to share/<package_name> so ament/ros2 can read metadata
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')), #copy all launch files to share/<package_name>/launch to call ros2 launch
        # Cấu hình
        (os.path.join('share', package_name, 'config'), glob('config/*')),# install config folder to share/<package_name>/config
        # RViz configs (nếu có)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),# install rviz config files to share/<package_name>/rviz
        # URDF/Xacro
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')), #copy all urdf files to share/<package_name>/urdf
        # Meshes
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')), #copy all mesh files to share/<package_name>/meshes
    ],
    #python dependencies to build and run this package
    install_requires=['setuptools'],
    zip_safe=True,
    #metadata for displaying 
    maintainer='dungx',
    maintainer_email='dungx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    #define entry points for executables ros2 run <package_name> <executable>
    entry_points={
        'console_scripts': [
            'joint_states = puppet.joint_states_publisher:main',
            'odometry = puppet.odometry:main', 
            'controller = puppet.controller:main',
            'scan_merger = puppet.scan_merger_simple:main',
            'robot_pose = puppet.robot_pose_in_map:main',
            'auto_mapping = puppet.auto_mapping:main'
        ],
    },
)
