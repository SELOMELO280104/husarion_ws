import numpy as np

from uav_groundtruth_mapping.row_corridor_mask import (
    orchard_boundary_wall_mask,
    estimate_row_geometry,
    make_row_corridor_mask,
    save_row_corridor_products,
)


def test_orchard_boundary_walls_are_rasterized_at_known_coordinates():
    origin = (-10.0, -11.0)
    resolution = 0.1
    mask = orchard_boundary_wall_mask((400, 500), origin, resolution)

    def occupied(x, y):
        column = int((x - origin[0]) / resolution)
        row = int((y - origin[1]) / resolution)
        return bool(mask[row, column])

    assert occupied(-9.05, 0.0)
    assert occupied(37.83, 0.0)
    assert occupied(0.0, -9.90)
    assert occupied(0.0, 27.74)
    assert not occupied(0.0, 0.0)


def synthetic_orchard():
    ground_axis_x = np.arange(-3.0, 18.01, 0.20)
    ground_axis_y = np.arange(-3.0, 12.01, 0.20)
    ground_x, ground_y = np.meshgrid(
        ground_axis_x, ground_axis_y)
    ground = np.column_stack((
        ground_x.ravel(),
        ground_y.ravel(),
        np.zeros(ground_x.size),
    ))

    vegetation = []
    for row_y in (0.0, 3.0, 6.0, 9.0):
        for row_x in np.arange(0.0, 15.01, 0.20):
            for offset in (-0.20, 0.0, 0.20):
                for height in (0.40, 0.55, 0.70):
                    vegetation.append(
                        [row_x, row_y + offset, height])
    return np.vstack((ground, np.asarray(vegetation)))


def test_detects_regular_vegetation_rows():
    points = synthetic_orchard()
    vegetation = points[points[:, 2] >= 0.35]

    geometry = estimate_row_geometry(vegetation[:, :2])

    assert geometry['angle_deg'] < 2.0
    assert len(geometry['row_centers']) == 4
    assert np.isclose(geometry['row_spacing'], 3.0, atol=0.2)


def test_mask_connects_panther_to_each_inter_row_corridor():
    (
        reachable,
        occupied,
        observed,
        centerlines,
        _origin,
        start,
        geometry,
    ) = make_row_corridor_mask(
        synthetic_orchard(),
        resolution=0.10,
        robot_radius=0.35,
        safety_margin=0.10,
        corridor_half_width=0.45,
        observation_fill_radius=0.20,
        start_location=(-2.0, -2.0),
    )

    assert start is not None
    assert np.count_nonzero(reachable) > 1000
    assert np.count_nonzero(occupied) > 0
    assert np.count_nonzero(observed) > 0
    assert np.count_nonzero(centerlines) > 100
    assert len(geometry['corridor_centers']) == 3


def test_semantic_lane_core_is_not_split_by_low_canopy_artifact():
    points = synthetic_orchard()
    artifact = np.repeat(
        np.asarray([[5.0, 1.5, 0.50]]),
        repeats=20,
        axis=0,
    )
    (
        reachable,
        _occupied,
        _observed,
        _centerlines,
        origin,
        _start,
        _geometry,
    ) = make_row_corridor_mask(
        np.vstack((points, artifact)),
        resolution=0.10,
        robot_radius=0.35,
        safety_margin=0.10,
        corridor_half_width=0.45,
        lane_core_half_width=0.40,
        observation_fill_radius=0.20,
        start_location=(-2.0, -2.0),
    )
    column = int((5.0 - origin[0]) / 0.10)
    row = int((1.5 - origin[1]) / 0.10)

    assert reachable[row, column]


def test_saves_viewer_and_nav2_row_mask_products(tmp_path):
    products = save_row_corridor_products(
        synthetic_orchard(),
        tmp_path / 'row_mask',
        resolution=0.10,
        robot_radius=0.35,
        safety_margin=0.10,
        corridor_half_width=0.45,
        observation_fill_radius=0.20,
        start_location=(-2.0, -2.0),
        source_name='synthetic.pcd',
    )

    assert products['row_count'] == 4
    assert products['corridor_count'] == 3
    assert products['accessible_corridor_count'] == 3
    assert products['reachable_cells'] > 1000
    assert products['pcd'].exists()
    assert products['ppm'].read_bytes().startswith(b'P6\n')
    assert products['nav_pgm'].exists()
    assert products['nav_yaml'].exists()
    assert products['centerlines_csv'].exists()
    assert products['metadata_yaml'].exists()
    assert 'FIELDS x y z rgb' in products['pcd'].read_text(
        encoding='ascii')
