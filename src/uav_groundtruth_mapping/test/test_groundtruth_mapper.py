import math

import numpy as np

from uav_groundtruth_mapping.groundtruth_pointcloud_mapper import (
    transform_lidar_points,
)


def test_groundtruth_transform_applies_model_pose_and_sensor_offset():
    points = np.array([[1.0, 0.0, 0.0]])
    model_position = np.array([10.0, -5.0, 2.0])
    model_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    rotation = np.eye(3)
    offset = np.array([0.0, 0.0, -0.1])

    transformed, sensor_position = transform_lidar_points(
        points,
        model_position,
        model_quaternion,
        rotation,
        offset,
    )

    assert np.allclose(sensor_position, [10.0, -5.0, 1.9])
    assert np.allclose(transformed, [[11.0, -5.0, 1.9]])


def test_groundtruth_transform_uses_exact_model_attitude():
    half = math.sqrt(0.5)
    points = np.array([[1.0, 0.0, 0.0]])

    transformed, _sensor_position = transform_lidar_points(
        points,
        np.zeros(3),
        np.array([half, 0.0, 0.0, half]),
        np.eye(3),
        np.zeros(3),
    )

    assert np.allclose(transformed, [[0.0, 1.0, 0.0]])
