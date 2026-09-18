import numpy as np

from uav_gps_mapping.pcd_to_occupancy import (
    is_ascii_pcd,
    make_occupancy,
    make_traversability,
    newest_gps_map,
    save_traversability_products,
)


def test_obstacle_is_rasterized_and_inflated():
    points = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 0.5],
        [1.01, 1.01, 0.6],
        [2.0, 2.0, 0.0],
    ])
    image, origin = make_occupancy(
        points, resolution=0.1, padding=0.2,
        inflation_radius=0.2, min_points_per_cell=2)
    assert origin == (-0.2, -0.2)
    assert np.count_nonzero(image == 0) > 4
    assert np.count_nonzero(image == 254) > 0


def test_rejects_empty_cloud():
    try:
        make_occupancy(np.empty((0, 3)))
    except ValueError:
        return
    raise AssertionError('empty cloud must be rejected')


def test_parked_panther_is_removed_from_static_map():
    points = np.repeat([[-6.0, -8.0, 0.5]], 10, axis=0)
    points = np.vstack((points, [[-8.0, -10.0, 0.0], [-4.0, -6.0, 0.0]]))
    image, origin = make_occupancy(
        points, resolution=0.1, padding=0.2, inflation_radius=0.0)
    map_y = int((-8.0 - origin[1]) / 0.1)
    map_x = int((-6.0 - origin[0]) / 0.1)
    image_y = image.shape[0] - 1 - map_y
    assert image[image_y, map_x] == 254


def test_newest_map_ignores_binary_compressed_pcd(tmp_path):
    ascii_map = tmp_path / 'uav_gps_map_older.pcd'
    ascii_map.write_text(
        '# .PCD v0.7\n'
        'FIELDS x y z\n'
        'WIDTH 1\n'
        'HEIGHT 1\n'
        'POINTS 1\n'
        'DATA ascii\n'
        '0 0 0\n'
        + '# padding\n' * 200,
        encoding='ascii',
    )
    binary_map = tmp_path / 'uav_gps_map_newer.pcd'
    binary_map.write_bytes(
        b'# .PCD v0.7\nFIELDS x y z\nWIDTH 100\n'
        b'POINTS 100\nDATA binary_compressed\n'
        + bytes(range(256)) * 8)

    assert is_ascii_pcd(ascii_map)
    assert not is_ascii_pcd(binary_map)
    assert newest_gps_map(tmp_path) == ascii_map


def test_traversability_inflates_obstacle_by_robot_footprint():
    coordinates = np.arange(0.0, 4.01, 0.1)
    ground_x, ground_y = np.meshgrid(coordinates, coordinates)
    ground = np.column_stack((
        ground_x.ravel(),
        ground_y.ravel(),
        np.zeros(ground_x.size),
    ))
    obstacle = np.array([
        [2.0, 2.0, 0.36],
        [2.01, 2.01, 0.45],
        [2.02, 2.02, 0.55],
        [2.03, 2.03, 0.64],
    ])
    points = np.vstack((ground, obstacle))

    reachable, occupied, observed, origin, start = make_traversability(
        points,
        resolution=0.1,
        robot_radius=0.5,
        safety_margin=0.1,
        observation_fill_radius=0.1,
        padding=0.2,
        start_location=(0.5, 0.5),
    )

    obstacle_x = int(np.floor((2.0 - origin[0]) / 0.1))
    obstacle_y = int(np.floor((2.0 - origin[1]) / 0.1))
    assert start is not None
    assert observed[obstacle_y, obstacle_x]
    assert occupied[obstacle_y, obstacle_x]
    assert occupied[obstacle_y, obstacle_x + 5]
    assert not reachable[obstacle_y, obstacle_x + 5]
    assert np.count_nonzero(reachable) > 100


def test_traversability_products_are_colored_and_viewable(tmp_path):
    coordinates = np.arange(0.0, 2.01, 0.1)
    ground_x, ground_y = np.meshgrid(coordinates, coordinates)
    points = np.column_stack((
        ground_x.ravel(),
        ground_y.ravel(),
        np.zeros(ground_x.size),
    ))
    points = np.vstack((
        points,
        [
            [1.0, 1.0, 0.36],
            [1.01, 1.01, 0.45],
            [1.02, 1.02, 0.55],
            [1.03, 1.03, 0.64],
        ],
    ))

    pcd_path, ppm_path, reachable_count = save_traversability_products(
        points,
        tmp_path / 'panther_mask',
        resolution=0.1,
        robot_radius=0.2,
        safety_margin=0.1,
        observation_fill_radius=0.1,
        padding=0.2,
        start_location=(0.3, 0.3),
    )

    assert reachable_count > 0
    assert ppm_path.read_bytes().startswith(b'P6\n')
    assert (tmp_path / 'panther_mask_nav.pgm').exists()
    assert (tmp_path / 'panther_mask_nav.yaml').exists()
    pcd_text = pcd_path.read_text(encoding='ascii')
    assert 'FIELDS x y z rgb' in pcd_text
    assert 'DATA ascii' in pcd_text
