#!/usr/bin/env python3
"""Create an RTAB-only orchard SDF with absolute local mesh URIs."""

from pathlib import Path


SOURCE = Path(
    '/home/robocare/husarion_ws/src/husarion_gz_worlds/worlds/orchard.sdf')
OUTPUT = Path('/tmp/uav_rtabmap_orchard.sdf')
ORCHARD_MESHES = Path(
    '/home/robocare/uav_ugv_ros2_jazzy/external/cpr_gazebo/'
    'cpr_orchard_gazebo/meshes')
ACCESSORY_MESHES = Path(
    '/home/robocare/uav_ugv_ros2_jazzy/external/cpr_gazebo/'
    'cpr_accessories_gazebo/meshes')


def main():
    text = SOURCE.read_text(encoding='utf-8')
    replacements = {
        'model://orchard/orchard_world.dae':
            f'file://{ORCHARD_MESHES / "orchard_world.dae"}',
        'model://orchard/orchard_trunks.dae':
            f'file://{ORCHARD_MESHES / "orchard_trunks.dae"}',
        'model://orchard/orchard_leaves.dae':
            f'file://{ORCHARD_MESHES / "orchard_leaves.dae"}',
        'model://accessories/base_station_with_tripod.dae':
            f'file://{ACCESSORY_MESHES / "BaseStationWithTripod.dae"}',
        'model://accessories/base_station_with_tripod.stl':
            f'file://{ACCESSORY_MESHES / "BaseStationWithTripod.stl"}',
    }
    for old, new in replacements.items():
        if old not in text:
            raise RuntimeError(f'Expected mesh URI is missing: {old}')
        text = text.replace(old, new)
    OUTPUT.write_text(text, encoding='utf-8')
    print(OUTPUT)


if __name__ == '__main__':
    main()
