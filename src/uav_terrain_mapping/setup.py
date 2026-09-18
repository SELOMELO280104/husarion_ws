from setuptools import find_packages, setup


package_name = 'uav_terrain_mapping'

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
    maintainer='robocare',
    maintainer_email='robocare@todo.todo',
    description='Experimental local-ground terrain mapping from UAV PCD files.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'build_terrain_map = uav_terrain_mapping.terrain_mapper:main',
        ],
    },
)
