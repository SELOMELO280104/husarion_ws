"""Unit tests for GPS-denied visual/depth following math."""

import math

import cv2
import numpy as np

from uav_ugv_control.visual_tag_follower import (
    bounded_velocity_toward,
    decode_quiet_zone_marker,
    depth_at_marker,
    detect_marker,
    detect_marker_quiet_zone,
    expanding_square_target,
    filtered_target_velocity,
    gazebo_visual_velocity,
    height_acceleration,
    identify_marker,
    locating_gate_ready,
    marker_detector,
    marker_position_relative_to_uav,
    point_along_polyline,
    route_intercept_target,
    triangular_sweep,
    visual_acceleration,
    visual_velocity,
)


def test_locating_gate_requires_button_route_and_head_start():
    assert not locating_gate_ready(True, False, 100.0, 200.0, 30.0)
    assert not locating_gate_ready(True, True, None, 200.0, 30.0)
    assert not locating_gate_ready(True, True, 100.0, 129.9, 30.0)
    assert locating_gate_ready(True, True, 100.0, 130.0, 30.0)
    assert locating_gate_ready(False, False, 100.0, 130.0, 30.0)


def _marker_image():
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50)
    marker = np.zeros((100, 100), dtype=np.uint8)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(
            dictionary, 0, 100)
    else:
        cv2.aruco.drawMarker(dictionary, 0, 100, marker, 1)
    canvas = np.full((300, 400), 255, dtype=np.uint8)
    canvas[100:200, 150:250] = marker
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def _quiet_zone_image(marker_id=0, blur=False):
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50)
    marker = np.zeros((84, 84), dtype=np.uint8)
    cv2.aruco.drawMarker(dictionary, marker_id, 84, marker, 1)
    image = np.full((480, 640, 3), (75, 105, 70), dtype=np.uint8)
    image[190:290, 270:370] = 255
    image[198:282, 278:362] = cv2.cvtColor(
        marker, cv2.COLOR_GRAY2BGR)
    if blur:
        image = cv2.GaussianBlur(image, (7, 7), 1.8)
    return image


def test_detects_expected_aruco_and_extracts_depth():
    image = _marker_image()
    corners = detect_marker(image, marker_detector(), marker_id=0)
    assert corners is not None
    assert np.allclose(np.mean(corners, axis=0), (199.5, 149.5), atol=1.0)

    depth = np.full(image.shape[:2], 10.0, dtype=np.float32)
    depth[0, 0] = np.nan
    assert depth_at_marker(depth, corners) == 10.0


def test_detects_simulated_marker_quiet_zone_fallback():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[200:280, 280:360] = 255
    marker = _marker_image()[100:200, 150:250]
    marker = cv2.resize(marker, (64, 64), interpolation=cv2.INTER_NEAREST)
    image[208:272, 288:352] = marker
    corners = detect_marker_quiet_zone(image)
    assert corners is not None
    assert np.allclose(np.mean(corners, axis=0), (319.5, 239.5), atol=2.0)


def test_quiet_zone_fallback_decodes_configured_id_after_blur():
    image = _quiet_zone_image(marker_id=0, blur=True)
    quiet_corners = detect_marker_quiet_zone(image)
    decoded = decode_quiet_zone_marker(image, quiet_corners)
    assert decoded is not None
    assert decoded['id'] == 0
    assert decoded['confidence'] >= 0.85

    detection = identify_marker(
        image, marker_detector(), marker_id=0)
    assert detection is not None
    assert detection['id'] == 0
    assert detection['confidence'] >= 0.85


def test_detector_rejects_a_different_marker_id():
    image = _quiet_zone_image(marker_id=7, blur=True)
    detection = identify_marker(
        image, marker_detector(), marker_id=0)
    assert detection is None


def test_centered_target_holds_position_and_height():
    velocity, error = visual_velocity(
        (200, 150), (300, 400, 3), 10.0, (350, 350), 0.0)
    assert velocity == (0.0, 0.0, 0.0)
    assert error == (0.0, 0.0)


def test_image_right_commands_body_right_for_gazebo_camera_axes():
    velocity, error = visual_velocity(
        (300, 150), (300, 400, 3), 10.0, (350, 350), 0.0)
    assert error == (100.0, 0.0)
    assert velocity[0] == 0.0
    assert velocity[1] > 0.0


def test_visual_velocity_centers_and_climbs_toward_ten_metres():
    velocity, error = visual_velocity(
        (300, 200), (300, 400, 3), 2.0, (350, 350), 0.0)
    assert error == (100.0, 50.0)
    assert velocity[0] < 0.0
    assert velocity[1] > 0.0
    assert velocity[2] < 0.0


def test_velocity_is_bounded_and_rotated_to_ned():
    velocity, _ = visual_velocity(
        (390, 10),
        (300, 400, 3),
        14.0,
        (350, 350),
        math.pi / 2.0,
        max_horizontal_speed=0.8,
        max_vertical_speed=0.5,
    )
    assert np.hypot(velocity[0], velocity[1]) <= 0.8 + 1e-9
    assert velocity[2] == 0.5


def test_gazebo_velocity_uses_camera_to_world_axis_signs():
    velocity, error = gazebo_visual_velocity(
        (240, 100), (480, 640, 3), 5.0, (320, 320))
    assert error == (-80.0, -140.0)
    assert velocity[0] > 0.0
    assert velocity[1] > 0.0
    assert np.hypot(*velocity) <= 1.0 + 1e-9


def test_marker_position_is_relative_to_uav_camera_origin():
    relative = marker_position_relative_to_uav(
        (240, 100), (480, 640, 3), 5.0, (320, 320))
    assert np.allclose(relative, (2.1875, 1.25))


def test_target_velocity_removes_uav_ego_motion():
    # The UAV moves +0.5 m/s. A stationary target consequently moves -0.05 m
    # relative to the camera over 0.1 s, so its estimated inertial speed is 0.
    estimate = filtered_target_velocity(
        (0.95, 0.0),
        (1.0, 0.0),
        0.1,
        (0.5, 0.0),
        filter_alpha=1.0,
    )
    assert np.allclose(estimate, (0.0, 0.0), atol=1e-9)


def test_target_velocity_estimates_panther_motion_without_world_pose():
    # UAV moves at 0.5 m/s and the target at 0.32 m/s, producing -0.018 m
    # relative displacement in 0.1 s.
    estimate = filtered_target_velocity(
        (0.982, 0.0),
        (1.0, 0.0),
        0.1,
        (0.5, 0.0),
        filter_alpha=1.0,
    )
    assert np.allclose(estimate, (0.32, 0.0), atol=1e-9)


def test_centered_marker_uses_velocity_feedforward():
    velocity, error = gazebo_visual_velocity(
        (320, 240),
        (480, 640, 3),
        10.0,
        (500, 500),
        target_velocity=(0.32, -0.10),
        feedforward_gain=0.75,
    )
    assert error == (0.0, 0.0)
    assert np.allclose(velocity, (0.24, -0.075))


def test_route_intercept_predicts_progress_and_sweeps_uncertainty():
    route = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0)]
    # Ten seconds at 0.4 m/s predicts x=4. The sweep begins 2 m behind.
    assert route_intercept_target(
        route, 10.0, 0.0, 0.4, 2.0, 8.0) == (2.0, 0.0)
    # Halfway through the rising half-cycle it reaches 2 m ahead.
    assert route_intercept_target(
        route, 10.0, 4.0, 0.4, 2.0, 8.0) == (6.0, 0.0)


def test_polyline_prediction_clamps_and_follows_corners():
    route = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]
    assert point_along_polyline(route, -1.0) == (0.0, 0.0)
    assert point_along_polyline(route, 5.0) == (3.0, 2.0)
    assert point_along_polyline(route, 99.0) == (3.0, 4.0)


def test_triangle_and_expanding_square_search_are_continuous():
    assert triangular_sweep(0.0, 4.0, 8.0) == -4.0
    assert triangular_sweep(2.0, 4.0, 8.0) == 0.0
    assert triangular_sweep(4.0, 4.0, 8.0) == 4.0
    assert expanding_square_target(
        (10.0, 20.0), 1.0, speed=3.0, spacing=3.0
    ) == (13.0, 20.0)
    assert expanding_square_target(
        (10.0, 20.0), 2.0, speed=3.0, spacing=3.0
    ) == (13.0, 23.0)


def test_search_velocity_is_bounded_and_stops_at_target():
    assert bounded_velocity_toward(
        (0.0, 0.0), (0.1, 0.1), 1.0, 0.2
    ) == (0.0, 0.0)
    velocity = bounded_velocity_toward(
        (0.0, 0.0), (3.0, 4.0), 1.0, 0.2)
    assert np.allclose(velocity, (0.6, 0.8))


def test_visual_acceleration_moves_toward_tag_and_is_bounded():
    acceleration, error = visual_acceleration(
        (400, 0), (480, 640, 3), 0.0)
    assert error == (80.0, -240.0)
    assert acceleration[0] < 0.0
    assert acceleration[1] < 0.0
    assert np.hypot(*acceleration) <= 1.5 + 1e-9


def test_height_acceleration_climbs_holds_and_damps():
    assert height_acceleration(0.0, 10.0) == -2.5
    assert height_acceleration(10.0, 10.0) == 0.0
    assert height_acceleration(10.0, 10.0, -1.0) > 0.0


def test_visual_acceleration_rotates_to_ned():
    acceleration, _ = visual_acceleration(
        (400, 240), (480, 640, 3), math.pi / 2.0)
    assert acceleration[0] > 0.0
    assert abs(acceleration[1]) < 1e-9
