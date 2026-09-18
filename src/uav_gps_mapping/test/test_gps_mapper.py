from collections import deque
import math
from types import SimpleNamespace

import numpy as np

from uav_gps_mapping.gps_pointcloud_mapper import (
    decode_rgb_image,
    gps_seeded_icp,
    interpolate_quaternion,
    interpolate_vector,
    project_rgb_to_lidar,
    quaternion_matrix,
    statistical_outlier_mask,
    voxel_centroids,
    write_colored_ascii_pcd,
)


def test_identity_quaternion():
    assert np.allclose(quaternion_matrix(1.0, 0.0, 0.0, 0.0), np.eye(3))


def test_ninety_degree_yaw():
    half = math.sqrt(0.5)
    rotation = quaternion_matrix(half, 0.0, 0.0, half)
    assert np.allclose(rotation @ np.array([1.0, 0.0, 0.0]),
                       np.array([0.0, 1.0, 0.0]))


def test_interpolate_position_at_scan_time():
    samples = deque([
        (10.0, np.array([0.0, 0.0, 0.0])),
        (10.1, np.array([1.0, 2.0, 3.0])),
    ])
    assert np.allclose(
        interpolate_vector(samples, 10.05),
        np.array([0.5, 1.0, 1.5]),
    )


def test_interpolate_attitude_at_scan_time():
    half = math.sqrt(0.5)
    samples = deque([
        (10.0, np.array([1.0, 0.0, 0.0, 0.0])),
        (10.1, np.array([half, 0.0, 0.0, half])),
    ])
    quaternion = interpolate_quaternion(samples, 10.05)
    rotation = quaternion_matrix(*quaternion)
    rotated = rotation @ np.array([1.0, 0.0, 0.0])
    expected = np.array([half, half, 0.0])
    assert np.allclose(rotated, expected)


def test_interpolation_rejects_stale_pose():
    samples = [(10.0, np.array([0.0, 0.0, 0.0]))]
    assert interpolate_vector(samples, 10.5) is None


def test_decode_bgr_image_to_rgb():
    message = SimpleNamespace(
        encoding='bgr8',
        height=1,
        width=1,
        step=3,
        data=bytes([30, 20, 10]),
    )
    image = decode_rgb_image(message)
    assert image.tolist() == [[[10, 20, 30]]]


def test_project_camera_center_color_to_lidar_point():
    image = np.zeros((5, 5, 3), dtype=np.uint8)
    image[2, 2] = [10, 20, 30]
    colors, valid = project_rgb_to_lidar(
        np.array([[1.0, 0.0, 0.0]]),
        image,
        (2.0, 2.0, 2.0, 2.0),
        np.eye(3),
        np.zeros(3),
        np.zeros(3),
    )
    assert valid.tolist() == [True]
    assert colors.tolist() == [[10, 20, 30]]


def test_voxel_centroids_average_repeated_cells():
    points = np.array([
        [0.01, 0.01, 0.01],
        [0.09, 0.09, 0.09],
        [0.21, 0.01, 0.01],
    ])
    centroids = voxel_centroids(points, 0.1)
    assert len(centroids) == 2
    assert np.any(np.all(np.isclose(
        centroids, [0.05, 0.05, 0.05]), axis=1))


def test_gps_seeded_icp_corrects_small_rigid_pose_error():
    generator = np.random.default_rng(7)
    target = generator.uniform(
        [-4.0, -3.0, 0.2],
        [4.0, 3.0, 2.0],
        size=(2000, 3),
    )
    yaw = math.radians(1.2)
    error_rotation = np.array([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    source = (
        target @ error_rotation.T
        + np.array([0.16, -0.10, 0.05]))

    result = gps_seeded_icp(
        source,
        target,
        minimum_correspondences=200,
        maximum_translation=0.4,
        maximum_rotation_degrees=3.0,
    )

    assert result.accepted
    assert result.rmse_after < 0.02
    assert result.rmse_after < result.rmse_before * 0.2
    assert np.mean(np.linalg.norm(
        result.transform(source) - target, axis=1)) < 0.02


def test_gps_seeded_icp_rejects_correction_outside_gps_bound():
    generator = np.random.default_rng(11)
    target = generator.uniform(-2.0, 2.0, size=(1000, 3))
    source = target + np.array([0.75, 0.0, 0.0])

    result = gps_seeded_icp(
        source,
        target,
        maximum_correspondence_distance=1.5,
        minimum_correspondences=100,
        maximum_translation=0.25,
    )

    assert not result.accepted


def test_gps_seeded_icp_rejects_flat_degenerate_geometry():
    x, y = np.meshgrid(
        np.linspace(-2.0, 2.0, 30),
        np.linspace(-2.0, 2.0, 30),
    )
    target = np.column_stack((
        x.ravel(),
        y.ravel(),
        np.zeros(x.size),
    ))
    source = target + np.array([0.1, -0.1, 0.0])

    result = gps_seeded_icp(
        source,
        target,
        minimum_correspondences=100,
    )

    assert not result.accepted
    assert result.reason == 'scan geometry is degenerate'


def test_translation_only_icp_preserves_px4_attitude():
    generator = np.random.default_rng(19)
    target = generator.uniform(
        [-3.0, -2.0, 0.2],
        [3.0, 2.0, 1.8],
        size=(1500, 3),
    )
    source = target + np.array([0.12, -0.08, 0.04])

    result = gps_seeded_icp(
        source,
        target,
        translation_only=True,
        minimum_correspondences=150,
        maximum_translation=0.20,
    )

    assert result.accepted
    assert np.allclose(result.rotation, np.eye(3))
    assert result.rotation_correction_deg == 0.0
    assert np.mean(np.linalg.norm(
        result.transform(source) - target, axis=1)) < 0.01


def test_statistical_outlier_mask_removes_isolated_point():
    x, y, z = np.meshgrid(
        np.linspace(0.0, 0.4, 5),
        np.linspace(0.0, 0.4, 5),
        np.linspace(0.0, 0.2, 3),
    )
    cluster = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
    points = np.vstack((cluster, np.array([[10.0, 10.0, 10.0]])))

    keep = statistical_outlier_mask(
        points,
        neighbor_count=8,
        stddev_multiplier=1.0,
        chunk_size=10,
    )

    assert np.all(keep[:-1])
    assert not keep[-1]


def test_write_colored_ascii_pcd(tmp_path):
    path = tmp_path / 'colored.pcd'
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    colors = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)

    write_colored_ascii_pcd(path, points, colors, 'test registration')

    text = path.read_text(encoding='ascii')
    assert '# Registration: test registration' in text
    assert 'FIELDS x y z rgb' in text
    assert 'POINTS 2' in text
