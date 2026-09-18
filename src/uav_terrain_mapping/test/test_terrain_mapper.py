import numpy as np

from uav_terrain_mapping.terrain_mapper import build_terrain_map


def test_raised_ground_is_not_an_obstacle_but_tree_is():
    points = []
    for x in np.linspace(0.0, 3.0, 31):
        for y in np.linspace(0.0, 2.0, 21):
            ground_z = 0.25 * x
            points.extend([[x, y, ground_z]] * 3)
    # A vertical tree return 1 m above the local sloping ground.
    points.extend([[1.5, 1.0, 1.375]] * 6)
    image, _, _ = build_terrain_map(
        np.asarray(points), resolution=0.1, padding=0.2,
        obstacle_height=0.4, obstacle_inflation=0.0)
    assert np.count_nonzero(image == 0) > 0
    assert np.count_nonzero(image == 254) > np.count_nonzero(image == 0)


def test_empty_cloud_is_rejected():
    try:
        build_terrain_map(np.empty((0, 3)))
    except ValueError:
        return
    raise AssertionError('empty cloud must be rejected')
