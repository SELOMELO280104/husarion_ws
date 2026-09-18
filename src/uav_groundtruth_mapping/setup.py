from glob import glob
from setuptools import find_packages, setup


package_name = 'uav_groundtruth_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
        (f'share/{package_name}/sdf', glob('sdf/*.sdf')),
        (f'share/{package_name}/urdf', glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'groundtruth_pointcloud_mapper = '
            'uav_groundtruth_mapping.groundtruth_pointcloud_mapper:main',
            'row_corridor_mask = '
            'uav_groundtruth_mapping.row_corridor_mask:main',
            'serpentine_mission_manager = '
            'uav_groundtruth_mapping.serpentine_mission_manager:main',
            'native_panther_drive = '
            'uav_groundtruth_mapping.native_panther_drive:main',
            'rl_search_ros_adapter = '
            'uav_groundtruth_mapping.rl_search_ros_adapter:main',
            'rl_search_training = '
            'uav_groundtruth_mapping.rl_search_training_node:main',
            'rl_training_dashboard = '
            'uav_groundtruth_mapping.rl_training_dashboard:main',
        ],
    },
)
