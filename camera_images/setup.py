from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_images'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anastasiia',
    maintainer_email='catundertheleaf@gmail.com',
    description='everything to deal with camera images',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_publisher = camera_images.webcam_publisher:main',
            'cam_subscriber = camera_images.camera_subscriber:main'
        ],
    },
)
