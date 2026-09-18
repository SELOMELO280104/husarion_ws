from glob import glob
from setuptools import find_packages, setup


package_name = 'uav_gps_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'gps_pointcloud_mapper = '
            'uav_gps_mapping.gps_pointcloud_mapper:main',
            'pcd_to_occupancy = '
            'uav_gps_mapping.pcd_to_occupancy:main',
            'nav_cmd_relay = '
            'uav_gps_mapping.nav_cmd_relay:main',
        ],
    },
)
