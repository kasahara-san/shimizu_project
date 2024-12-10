import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'shimizu_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'rviz2'),glob('rviz2/*.rviz')),
        (os.path.join('share', package_name, 'images'),glob('images/*.png')),   
        (os.path.join('share', package_name, 'srv'),glob('srv/*.yaml')),     
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='common',
    maintainer_email='kasahara.yuichiro.res@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'operasim_physx_default_map_publisher = shimizu_project.operasim_physx_default_map_publisher:main',
            'tf_broadcaster_map_to_baselink = shimizu_project.tf_broadcaster_map_to_baselink:main',
        ],
    },
)
