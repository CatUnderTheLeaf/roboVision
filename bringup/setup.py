from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anastasiia',
    maintainer_email='catundertheleaf@gmail.com',
    description='Contains launch files for the roboVision project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

# content of the CMakeLists.txt
# cmake_minimum_required(VERSION 3.5)

# project(bringup)

# find_package(ament_cmake REQUIRED)
# find_package(description REQUIRED)
# find_package(ros_gz_gazebo REQUIRED)

# # Install project launch files
# install(
#   DIRECTORY
#     launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# # Install project configuration files
# install(
#   DIRECTORY
#     config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

