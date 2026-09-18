from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'uav_ugv_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eee-Andrew',
    maintainer_email='eee18387048@uniwa.gr',
    description='Example UAV/UGV coordination nodes for ROS 2 Jazzy.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'formation_demo = uav_ugv_control.formation_demo:main',
            'vegetation_traverse = uav_ugv_control.vegetation_traverse:main',
            'x500_follow_ugv = uav_ugv_control.x500_follow_ugv:main',
            'x500_survey = uav_ugv_control.x500_survey:main',
            (
                'visual_tag_follower = '
                'uav_ugv_control.visual_tag_follower:main'
            ),
            (
            'uav_camera_viewer = '
                'uav_ugv_control.uav_camera_viewer:main'
            ),
            'groundtruth_relative_tracker = '
            'uav_ugv_control.groundtruth_relative_tracker:main',
            'repo_style_controller = '
            'uav_ugv_control.repo_style_controller:main',
            'repo_relative_state = '
            'uav_ugv_control.repo_relative_state:main',
            'repo_landing_controller = '
            'uav_ugv_control.repo_landing_controller:main',
            'repo_landing_trainer = '
            'uav_ugv_control.repo_landing_trainer:main',
            'repo_policy_controller = '
            'uav_ugv_control.repo_policy_controller:main',
        ],
    },
)
