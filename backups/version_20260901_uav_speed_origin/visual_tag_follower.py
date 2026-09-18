"""Follow a roof-mounted ArUco tag using only downward RGB-D observations."""

from __future__ import annotations

from collections import deque
import json
import math
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as PathMessage
import numpy as np
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger


def marker_detector():
    """Construct an OpenCV ArUco 4X4_50 detector."""
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        parameters = cv2.aruco.DetectorParameters_create()
    else:
        parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.minMarkerPerimeterRate = 0.02
    parameters.maxMarkerPerimeterRate = 1.5
    if hasattr(cv2.aruco, 'ArucoDetector'):
        return cv2.aruco.ArucoDetector(dictionary, parameters)
    return dictionary, parameters


def ordered_quad(corners):
    """Return corners ordered top-left, top-right, bottom-right, bottom-left."""
    points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    ordered = np.zeros((4, 2), dtype=np.float32)
    coordinate_sum = points.sum(axis=1)
    coordinate_difference = np.diff(points, axis=1).reshape(-1)
    ordered[0] = points[np.argmin(coordinate_sum)]
    ordered[2] = points[np.argmax(coordinate_sum)]
    ordered[1] = points[np.argmin(coordinate_difference)]
    ordered[3] = points[np.argmax(coordinate_difference)]
    return ordered


def marker_quiet_zone_candidates(image):
    """
    Find square white backings that may contain the simulated roof marker.

    Gazebo can blur the individual ArUco cells at survey height even though
    the high-contrast quiet zone remains unambiguous.
    """
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(
        hsv,
        np.array((0, 0, 165), dtype=np.uint8),
        np.array((179, 75, 255), dtype=np.uint8),
    )
    white = cv2.morphologyEx(
        white,
        cv2.MORPH_CLOSE,
        np.ones((3, 3), dtype=np.uint8),
    )
    contours, _ = cv2.findContours(
        white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 50.0 or area > 0.25 * height * width:
            continue
        rectangle = cv2.minAreaRect(contour)
        center, size, _ = rectangle
        short_side = min(size)
        long_side = max(size)
        if short_side < 6.0 or long_side / max(short_side, 1.0) > 1.35:
            continue
        if area / max(float(size[0] * size[1]), 1.0) < 0.65:
            continue
        x, y, box_width, box_height = cv2.boundingRect(contour)
        region = gray[y:y + box_height, x:x + box_width]
        dark_fraction = float(np.mean(region < 80)) if region.size else 0.0
        if not 0.15 <= dark_fraction <= 0.80:
            continue
        box = ordered_quad(cv2.boxPoints(rectangle))
        candidates.append((area, box))
    return [candidate[1] for candidate in sorted(
        candidates, key=lambda candidate: candidate[0], reverse=True)]


def detect_marker_quiet_zone(image):
    """Return the strongest square quiet-zone candidate, if one exists."""
    candidates = marker_quiet_zone_candidates(image)
    return candidates[0] if candidates else None


def aruco_marker_bits(marker_id, dictionary=None):
    """Generate the 6-by-6 black-border and payload bit grid for an ID."""
    if dictionary is None:
        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50)
    marker = np.zeros((120, 120), dtype=np.uint8)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(
            dictionary, int(marker_id), 120)
    else:
        cv2.aruco.drawMarker(
            dictionary, int(marker_id), 120, marker, 1)
    return np.asarray([
        [
            np.mean(cell) >= 127.5
            for cell in np.array_split(row, 6, axis=1)
        ]
        for row in np.array_split(marker, 6, axis=0)
    ], dtype=bool)


def decode_quiet_zone_marker(image, quiet_corners, dictionary=None):
    """Perspective-correct and identify an ArUco marker inside a quiet zone."""
    if dictionary is None:
        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50)
    canonical_size = 144
    destination = np.asarray([
        [0, 0],
        [canonical_size - 1, 0],
        [canonical_size - 1, canonical_size - 1],
        [0, canonical_size - 1],
    ], dtype=np.float32)
    transform = cv2.getPerspectiveTransform(
        ordered_quad(quiet_corners), destination)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    rectified = cv2.warpPerspective(
        gray, transform, (canonical_size, canonical_size))

    _, dark = cv2.threshold(
        rectified, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(
        dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    marker_regions = []
    image_center = np.asarray(
        (0.5 * canonical_size, 0.5 * canonical_size))
    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        short_side = min(width, height)
        long_side = max(width, height)
        if short_side < 0.45 * canonical_size:
            continue
        if long_side / max(short_side, 1) > 1.25:
            continue
        center = np.asarray((x + 0.5 * width, y + 0.5 * height))
        if np.linalg.norm(center - image_center) > 0.20 * canonical_size:
            continue
        marker_regions.append((width * height, x, y, width, height))
    if not marker_regions:
        return None

    _, x, y, width, height = max(marker_regions)
    marker = cv2.resize(
        rectified[y:y + height, x:x + width],
        (120, 120),
        interpolation=cv2.INTER_AREA,
    )
    cell_means = np.asarray([
        [
            float(np.mean(cell))
            for cell in np.array_split(row, 6, axis=1)
        ]
        for row in np.array_split(marker, 6, axis=0)
    ])
    threshold = 0.5 * (float(np.min(cell_means)) + float(np.max(cell_means)))
    observed = cell_means >= threshold

    matches = []
    marker_count = int(dictionary.bytesList.shape[0])
    border = np.ones((6, 6), dtype=bool)
    border[1:-1, 1:-1] = False
    for candidate_id in range(marker_count):
        expected = aruco_marker_bits(candidate_id, dictionary)
        for rotation in range(4):
            rotated = np.rot90(observed, rotation)
            total_errors = int(np.count_nonzero(rotated != expected))
            border_errors = int(np.count_nonzero(rotated[border]))
            payload_errors = int(np.count_nonzero(
                rotated[1:-1, 1:-1] != expected[1:-1, 1:-1]))
            matches.append((
                total_errors,
                payload_errors,
                border_errors,
                candidate_id,
                rotation,
            ))
    matches.sort()
    best = matches[0]
    second_different_id = next(
        match for match in matches if match[3] != best[3])
    total_errors, payload_errors, border_errors, marker_id, rotation = best
    if border_errors > 2 or payload_errors > 2:
        return None
    if second_different_id[0] - total_errors < 1:
        return None
    return {
        'id': marker_id,
        'confidence': max(0.0, 1.0 - total_errors / 36.0),
        'rotation_quadrants': rotation,
        'total_bit_errors': total_errors,
    }


def identify_marker(image, detector, marker_id=0, minimum_confidence=0.85):
    """Identify one configured marker and return its metadata and corners."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(detector, 'detectMarkers'):
        corners, identifiers, _ = detector.detectMarkers(gray)
    else:  # OpenCV before ArucoDetector was introduced
        dictionary, parameters = detector
        corners, identifiers, _ = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=parameters)
    if identifiers is not None:
        for corners_item, identifier in zip(
                corners, identifiers.flatten()):
            if int(identifier) == int(marker_id):
                return {
                    'id': int(identifier),
                    'corners': ordered_quad(corners_item),
                    'confidence': 1.0,
                    'source': 'opencv_aruco',
                    'bit_errors': 0,
                }

    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50)
    for quiet_corners in marker_quiet_zone_candidates(image):
        decoded = decode_quiet_zone_marker(
            image, quiet_corners, dictionary)
        if decoded is None:
            continue
        if decoded['id'] != int(marker_id):
            continue
        if decoded['confidence'] < float(minimum_confidence):
            continue
        return {
            'id': decoded['id'],
            'corners': quiet_corners,
            'confidence': decoded['confidence'],
            'source': 'verified_quiet_zone',
            'bit_errors': decoded['total_bit_errors'],
        }
    return None


def detect_marker(image, detector, marker_id=0):
    """Return the requested marker's four image corners, or None."""
    detection = identify_marker(image, detector, marker_id)
    return detection['corners'] if detection is not None else None


def depth_at_marker(depth_image, corners, minimum=0.2, maximum=30.0):
    """Return robust median depth in metres within an inset marker polygon."""
    depth = np.asarray(depth_image)
    if depth.ndim != 2:
        return None
    depth = depth.astype(np.float32, copy=False)
    finite = depth[np.isfinite(depth)]
    if finite.size and float(np.nanmedian(finite)) > 100.0:
        depth = depth * 0.001
    center = np.mean(corners, axis=0)
    inset = center + 0.70 * (corners - center)
    mask = np.zeros(depth.shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.rint(inset).astype(np.int32), 1)
    samples = depth[
        (mask != 0)
        & np.isfinite(depth)
        & (depth >= minimum)
        & (depth <= maximum)
    ]
    if samples.size < 8:
        return None
    return float(np.median(samples))


def visual_velocity(
    marker_center,
    image_shape,
    depth,
    focal_xy,
    heading,
    desired_depth=10.0,
    horizontal_gain=0.60,
    vertical_gain=0.45,
    max_horizontal_speed=1.0,
    max_vertical_speed=0.6,
    deadband_pixels=4.0,
):
    """Convert image/depth errors to a bounded NED velocity command."""
    height, width = image_shape[:2]
    error_u = float(marker_center[0]) - 0.5 * width
    error_v = float(marker_center[1]) - 0.5 * height
    if abs(error_u) < deadband_pixels:
        error_u = 0.0
    if abs(error_v) < deadband_pixels:
        error_v = 0.0
    focal_x = max(float(focal_xy[0]), 1.0)
    focal_y = max(float(focal_xy[1]), 1.0)

    # For Gazebo's +X optical-axis camera after the +90 degree pitch:
    # image-right is vehicle-right and image-down is vehicle-rearward.
    # Move in the same direction as the observed horizontal offset.
    forward = -horizontal_gain * error_v * depth / focal_y
    right = horizontal_gain * error_u * depth / focal_x
    horizontal_norm = math.hypot(forward, right)
    if horizontal_norm > max_horizontal_speed:
        scale = max_horizontal_speed / horizontal_norm
        forward *= scale
        right *= scale

    north = math.cos(heading) * forward - math.sin(heading) * right
    east = math.sin(heading) * forward + math.cos(heading) * right
    down = vertical_gain * (depth - desired_depth)
    down = float(np.clip(
        down, -max_vertical_speed, max_vertical_speed))
    return (north, east, down), (error_u, error_v)


def gazebo_visual_velocity(
    marker_center,
    image_shape,
    depth,
    focal_xy,
    horizontal_gain=0.60,
    max_horizontal_speed=1.0,
    deadband_pixels=4.0,
    target_velocity=(0.0, 0.0),
    feedforward_gain=0.0,
):
    """Convert UAV-relative tag error and target speed into XY velocity."""
    height, width = image_shape[:2]
    error_u = float(marker_center[0]) - 0.5 * width
    error_v = float(marker_center[1]) - 0.5 * height
    if abs(error_u) < deadband_pixels:
        error_u = 0.0
    if abs(error_v) < deadband_pixels:
        error_v = 0.0
    focal_x = max(float(focal_xy[0]), 1.0)
    focal_y = max(float(focal_xy[1]), 1.0)

    # With the camera pitched +90 degrees, image-right is world -Y and
    # image-down is world -X while the vehicle yaw is held at zero.
    velocity_x = (
        -horizontal_gain * error_v * depth / focal_y
        + feedforward_gain * float(target_velocity[0])
    )
    velocity_y = (
        -horizontal_gain * error_u * depth / focal_x
        + feedforward_gain * float(target_velocity[1])
    )
    speed = math.hypot(velocity_x, velocity_y)
    if speed > max_horizontal_speed:
        scale = max_horizontal_speed / speed
        velocity_x *= scale
        velocity_y *= scale
    return (velocity_x, velocity_y), (error_u, error_v)


def marker_position_relative_to_uav(
    marker_center,
    image_shape,
    depth,
    focal_xy,
):
    """Project a marker pixel and depth into UAV-centred horizontal metres."""
    height, width = image_shape[:2]
    error_u = float(marker_center[0]) - 0.5 * width
    error_v = float(marker_center[1]) - 0.5 * height
    focal_x = max(float(focal_xy[0]), 1.0)
    focal_y = max(float(focal_xy[1]), 1.0)
    return np.asarray((
        -error_v * float(depth) / focal_y,
        -error_u * float(depth) / focal_x,
    ), dtype=np.float64)


def filtered_target_velocity(
    current_relative_position,
    previous_relative_position,
    dt,
    uav_velocity,
    previous_estimate=(0.0, 0.0),
    filter_alpha=0.20,
    maximum_speed=0.80,
):
    """Estimate target velocity without a world pose using relative motion."""
    dt = float(dt)
    if dt <= 1e-6:
        return np.asarray(previous_estimate, dtype=np.float64)
    relative_velocity = (
        np.asarray(current_relative_position, dtype=np.float64)
        - np.asarray(previous_relative_position, dtype=np.float64)
    ) / dt
    raw_target_velocity = relative_velocity + np.asarray(
        uav_velocity, dtype=np.float64)
    maximum_speed = max(float(maximum_speed), 0.0)
    raw_speed = float(np.linalg.norm(raw_target_velocity))
    if raw_speed > maximum_speed and raw_speed > 1e-9:
        raw_target_velocity *= maximum_speed / raw_speed
    alpha = float(np.clip(filter_alpha, 0.0, 1.0))
    return (
        (1.0 - alpha) * np.asarray(previous_estimate, dtype=np.float64)
        + alpha * raw_target_velocity
    )


def point_along_polyline(points, distance):
    """Return the XY point a clamped distance along a polyline."""
    if not points:
        return None
    if len(points) == 1:
        return tuple(float(value) for value in points[0])
    remaining = max(float(distance), 0.0)
    previous = np.asarray(points[0], dtype=np.float64)
    for point in points[1:]:
        current = np.asarray(point, dtype=np.float64)
        segment = current - previous
        length = float(np.linalg.norm(segment))
        if length <= 1e-9:
            previous = current
            continue
        if remaining <= length:
            result = previous + (remaining / length) * segment
            return (float(result[0]), float(result[1]))
        remaining -= length
        previous = current
    return (float(previous[0]), float(previous[1]))


def triangular_sweep(elapsed, amplitude, period):
    """Sweep continuously from -amplitude to +amplitude and back."""
    amplitude = max(float(amplitude), 0.0)
    period = max(float(period), 1e-6)
    phase = (max(float(elapsed), 0.0) % period) / period
    if phase < 0.5:
        return -amplitude + 4.0 * amplitude * phase
    return 3.0 * amplitude - 4.0 * amplitude * phase


def route_intercept_target(
    points,
    mission_elapsed,
    search_elapsed,
    nominal_speed=0.32,
    sweep_distance=4.0,
    sweep_period=12.0,
):
    """Predict and sweep around the UGV's likely point on its known route."""
    predicted_distance = (
        max(float(mission_elapsed), 0.0)
        * max(float(nominal_speed), 0.0)
    )
    uncertainty_offset = triangular_sweep(
        search_elapsed, sweep_distance, sweep_period)
    return point_along_polyline(
        points, predicted_distance + uncertainty_offset)


def expanding_square_target(
    origin,
    elapsed,
    speed=1.0,
    spacing=3.0,
):
    """Return a target on a continuous expanding-square coverage path."""
    origin = np.asarray(origin, dtype=np.float64)
    remaining = max(float(elapsed), 0.0) * max(float(speed), 0.0)
    spacing = max(float(spacing), 0.1)
    current = origin.copy()
    directions = (
        np.asarray((1.0, 0.0)),
        np.asarray((0.0, 1.0)),
        np.asarray((-1.0, 0.0)),
        np.asarray((0.0, -1.0)),
    )
    for leg in range(200):
        length = spacing * (leg // 2 + 1)
        direction = directions[leg % 4]
        if remaining <= length:
            result = current + remaining * direction
            return (float(result[0]), float(result[1]))
        current += length * direction
        remaining -= length
    return (float(current[0]), float(current[1]))


def bounded_velocity_toward(
    current,
    target,
    maximum_speed=1.0,
    arrival_radius=0.20,
):
    """Return a bounded world-XY velocity toward a search target."""
    delta = np.asarray(target, dtype=np.float64) - np.asarray(
        current, dtype=np.float64)
    distance = float(np.linalg.norm(delta))
    if distance <= max(float(arrival_radius), 0.0) or distance <= 1e-9:
        return (0.0, 0.0)
    speed = min(max(float(maximum_speed), 0.0), distance)
    velocity = speed * delta / distance
    return (float(velocity[0]), float(velocity[1]))


def visual_acceleration(
    marker_center,
    image_shape,
    heading,
    error_rate=(0.0, 0.0),
    proportional_gain=1.5,
    derivative_gain=0.20,
    max_horizontal_acceleration=1.5,
    deadband_pixels=4.0,
):
    """Convert marker image error to a bounded horizontal NED acceleration."""
    height, width = image_shape[:2]
    error_u = float(marker_center[0]) - 0.5 * width
    error_v = float(marker_center[1]) - 0.5 * height
    if abs(error_u) < deadband_pixels:
        error_u = 0.0
    if abs(error_v) < deadband_pixels:
        error_v = 0.0
    half_width = max(0.5 * width, 1.0)
    half_height = max(0.5 * height, 1.0)
    rate_u = float(error_rate[0])
    rate_v = float(error_rate[1])

    # Gazebo's optical image axes and PX4's NED/FRD control axes have
    # opposite horizontal signs for this +90 degree pitched camera. Move
    # opposite the pixel displacement. The derivative term damps closing
    # motion as the tag approaches image center.
    forward = (
        proportional_gain * error_v / half_height
        + derivative_gain * rate_v / half_height
    )
    right = (
        -proportional_gain * error_u / half_width
        - derivative_gain * rate_u / half_width
    )
    horizontal_norm = math.hypot(forward, right)
    if horizontal_norm > max_horizontal_acceleration:
        scale = max_horizontal_acceleration / horizontal_norm
        forward *= scale
        right *= scale

    north = math.cos(heading) * forward - math.sin(heading) * right
    east = math.sin(heading) * forward + math.cos(heading) * right
    return (north, east), (error_u, error_v)


def height_acceleration(
    height,
    target_height,
    vertical_velocity_down=0.0,
    proportional_gain=0.35,
    damping_gain=0.70,
    maximum_acceleration=2.5,
):
    """Return a bounded NED down acceleration for visual height holding."""
    acceleration_down = (
        proportional_gain * (height - target_height)
        - damping_gain * vertical_velocity_down
    )
    return float(np.clip(
        acceleration_down,
        -maximum_acceleration,
        maximum_acceleration,
    ))


def locating_gate_ready(
    manual_start_required,
    locating_requested,
    harvest_started_monotonic,
    now,
    search_start_delay,
):
    """Return true only after operator authorization and the UGV head start."""
    if manual_start_required and not locating_requested:
        return False
    if harvest_started_monotonic is None:
        return False
    return (
        float(now) - float(harvest_started_monotonic)
        >= float(search_start_delay)
    )


class VisualTagFollower(Node):
    """Control a GPS-disabled PX4 vehicle from a downward ArUco RGB-D view."""

    def __init__(self):
        super().__init__('uav_visual_tag_follower')
        self.declare_parameter('image_topic', '/uav/tag_camera/image')
        self.declare_parameter(
            'depth_topic', '/uav/tag_camera/depth_image')
        self.declare_parameter(
            'camera_info_topic', '/uav/tag_camera/camera_info')
        self.declare_parameter(
            'vehicle_status_topic', '/fmu/out/vehicle_status_v4')
        self.declare_parameter(
            'local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter(
            'command_ack_topic', '/fmu/out/vehicle_command_ack_v1')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('minimum_detection_confidence', 0.85)
        self.declare_parameter('desired_depth', 10.0)
        self.declare_parameter('takeoff_altitude', 10.0)
        self.declare_parameter('horizontal_gain', 0.60)
        self.declare_parameter('vertical_gain', 0.45)
        self.declare_parameter('max_horizontal_speed', 1.0)
        self.declare_parameter('max_vertical_speed', 0.6)
        self.declare_parameter('velocity_feedforward_gain', 0.75)
        self.declare_parameter('target_velocity_filter_alpha', 0.20)
        self.declare_parameter('max_estimated_target_speed', 0.80)
        self.declare_parameter('bootstrap_climb_speed', 0.5)
        self.declare_parameter('horizontal_acceleration_gain', 1.5)
        self.declare_parameter('horizontal_damping_gain', 0.20)
        self.declare_parameter('max_horizontal_acceleration', 1.5)
        self.declare_parameter('height_acceleration_gain', 0.35)
        self.declare_parameter('vertical_damping_gain', 0.70)
        self.declare_parameter('max_vertical_acceleration', 2.5)
        self.declare_parameter('depth_sync_tolerance', 0.35)
        self.declare_parameter('visual_engage_altitude', 1.5)
        self.declare_parameter('marker_timeout', 1.2)
        self.declare_parameter('ground_depth_timeout', 5.0)
        self.declare_parameter('critical_loss_timeout', 5.0)
        self.declare_parameter('search_start_delay', 30.0)
        self.declare_parameter('manual_start_required', False)
        self.declare_parameter('search_origin_x', -6.0)
        self.declare_parameter('search_origin_y', -9.0)
        self.declare_parameter('ugv_origin_x', -6.0)
        self.declare_parameter('ugv_origin_y', -8.0)
        self.declare_parameter('ugv_nominal_speed', 0.32)
        self.declare_parameter('route_sweep_distance', 4.0)
        self.declare_parameter('route_sweep_period', 12.0)
        self.declare_parameter('local_reacquire_timeout', 8.0)
        self.declare_parameter('fallback_search_spacing', 3.0)
        self.declare_parameter('search_arrival_radius', 0.20)
        self.declare_parameter('startup_setpoints', 30)
        self.declare_parameter('command_retry_seconds', 2.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_offboard', True)
        self.declare_parameter('control_backend', 'px4_acceleration')
        self.declare_parameter(
            'gazebo_velocity_topic',
            '/model/x500_flow_tag_0/cmd_vel',
        )

        self.marker_id = int(self.get_parameter('marker_id').value)
        self.minimum_detection_confidence = float(
            self.get_parameter('minimum_detection_confidence').value)
        self.desired_depth = float(
            self.get_parameter('desired_depth').value)
        self.takeoff_altitude = float(
            self.get_parameter('takeoff_altitude').value)
        self.horizontal_gain = float(
            self.get_parameter('horizontal_gain').value)
        self.vertical_gain = float(
            self.get_parameter('vertical_gain').value)
        self.max_horizontal_speed = float(
            self.get_parameter('max_horizontal_speed').value)
        self.max_vertical_speed = float(
            self.get_parameter('max_vertical_speed').value)
        self.velocity_feedforward_gain = float(
            self.get_parameter('velocity_feedforward_gain').value)
        self.target_velocity_filter_alpha = float(
            self.get_parameter('target_velocity_filter_alpha').value)
        self.max_estimated_target_speed = float(
            self.get_parameter('max_estimated_target_speed').value)
        self.bootstrap_climb_speed = float(
            self.get_parameter('bootstrap_climb_speed').value)
        self.horizontal_acceleration_gain = float(
            self.get_parameter('horizontal_acceleration_gain').value)
        self.horizontal_damping_gain = float(
            self.get_parameter('horizontal_damping_gain').value)
        self.max_horizontal_acceleration = float(
            self.get_parameter('max_horizontal_acceleration').value)
        self.height_acceleration_gain = float(
            self.get_parameter('height_acceleration_gain').value)
        self.vertical_damping_gain = float(
            self.get_parameter('vertical_damping_gain').value)
        self.max_vertical_acceleration = float(
            self.get_parameter('max_vertical_acceleration').value)
        self.depth_sync_tolerance = float(
            self.get_parameter('depth_sync_tolerance').value)
        self.visual_engage_altitude = float(
            self.get_parameter('visual_engage_altitude').value)
        self.marker_timeout = float(
            self.get_parameter('marker_timeout').value)
        self.ground_depth_timeout = float(
            self.get_parameter('ground_depth_timeout').value)
        self.critical_loss_timeout = float(
            self.get_parameter('critical_loss_timeout').value)
        self.search_start_delay = float(
            self.get_parameter('search_start_delay').value)
        self.manual_start_required = bool(
            self.get_parameter('manual_start_required').value)
        self.search_origin = np.asarray((
            float(self.get_parameter('search_origin_x').value),
            float(self.get_parameter('search_origin_y').value),
        ), dtype=np.float64)
        self.ugv_origin = (
            float(self.get_parameter('ugv_origin_x').value),
            float(self.get_parameter('ugv_origin_y').value),
        )
        self.ugv_nominal_speed = float(
            self.get_parameter('ugv_nominal_speed').value)
        self.route_sweep_distance = float(
            self.get_parameter('route_sweep_distance').value)
        self.route_sweep_period = float(
            self.get_parameter('route_sweep_period').value)
        self.local_reacquire_timeout = float(
            self.get_parameter('local_reacquire_timeout').value)
        self.fallback_search_spacing = float(
            self.get_parameter('fallback_search_spacing').value)
        self.search_arrival_radius = float(
            self.get_parameter('search_arrival_radius').value)
        self.startup_setpoints = int(
            self.get_parameter('startup_setpoints').value)
        self.command_retry_seconds = float(
            self.get_parameter('command_retry_seconds').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.auto_offboard = bool(
            self.get_parameter('auto_offboard').value)
        self.control_backend = str(
            self.get_parameter('control_backend').value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.gazebo_velocity_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('gazebo_velocity_topic').value),
            command_qos,
        )
        self.debug_image_pub = self.create_publisher(
            Image, '/uav/tag_follower/debug_image', sensor_qos)
        self.ready_pub = self.create_publisher(
            Bool, '/uav/tag_follower/ready', latched_qos)
        self.handoff_pub = self.create_publisher(
            Bool, '/uav/tag_follower/landing_handoff_ready', latched_qos)
        self.status_pub = self.create_publisher(
            String, '/uav/tag_follower/status', latched_qos)
        self.detection_pub = self.create_publisher(
            String, '/uav/tag_follower/detection', sensor_qos)

        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._image_callback,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._depth_callback,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            sensor_qos,
        )
        self.create_subscription(
            VehicleStatus,
            str(self.get_parameter('vehicle_status_topic').value),
            self._vehicle_status_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            str(self.get_parameter('local_position_topic').value),
            self._local_position_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleCommandAck,
            str(self.get_parameter('command_ack_topic').value),
            self._command_ack_callback,
            px4_qos,
        )
        self.create_subscription(
            String,
            '/panther/harvest/status',
            self._harvest_status_callback,
            latched_qos,
        )
        self.create_subscription(
            PathMessage,
            '/panther/harvest/planned_path',
            self._planned_path_callback,
            latched_qos,
        )
        self.create_service(
            Trigger,
            '/uav/tag_follower/start_locating',
            self._start_locating_service,
        )
        self.create_service(
            Trigger,
            '/uav/tag_follower/request_rl_landing',
            self._landing_service,
        )

        self.bridge = CvBridge()
        self.detector = marker_detector()
        self.depth_frames = deque(maxlen=12)
        self.focal_xy = (692.8, 692.8)
        self.vehicle_status = None
        self.local_position = None
        self.latest_observation = None
        self.latest_ground_depth = None
        self.last_ground_depth_monotonic = None
        self.last_detection_monotonic = None
        self.stable_since = None
        self.ever_ready = False
        self.route_complete = False
        self.setpoint_count = 0
        self.last_command_monotonic = 0.0
        self.last_status_text = ''
        self.hold_position = None
        self.yaw_target = None
        self.previous_pixel_error = None
        self.previous_error_monotonic = None
        self.last_processed_observation = None
        self.last_visual_command = (0.0, 0.0)
        self.harvest_state = 'WAITING_FOR_SENSORS'
        self.harvest_started_monotonic = None
        self.locating_requested = not self.manual_start_required
        self.locating_started_monotonic = None
        self.planned_route = []
        self.estimated_world_xy = self.search_origin.copy()
        self.last_motion_tick_monotonic = time.monotonic()
        self.last_gazebo_xy_command = (0.0, 0.0)
        self.marker_relative_xy = None
        self.previous_marker_relative_xy = None
        self.previous_marker_observation_monotonic = None
        self.last_velocity_observation_monotonic = None
        self.estimated_panther_velocity_xy = np.zeros(2, dtype=np.float64)
        self.last_marker_world_xy = None
        self.search_target_xy = None
        self.current_mode = (
            'WAITING_FOR_OPERATOR'
            if self.manual_start_required else 'WAITING_FOR_UGV'
        )
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'GPS-denied tag follower waiting for RGB-D '
            f'(backend={self.control_backend}). '
            'The UGV starts independently; UAV search begins after '
            f'{self.search_start_delay:.1f} s and '
            + (
                'an operator presses Start Locating.'
                if self.manual_start_required
                else 'automatic authorization.'
            ))

    def _timestamp_us(self):
        return int(self.get_clock().now().nanoseconds // 1000)

    def _depth_callback(self, message):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='passthrough')
            depth_stamp = (
                int(message.header.stamp.sec)
                + 1e-9 * int(message.header.stamp.nanosec)
            )
            self.depth_frames.append((depth_stamp, depth_image))
            depth_metres = np.asarray(
                depth_image, dtype=np.float32)
            finite = depth_metres[np.isfinite(depth_metres)]
            if finite.size and float(np.nanmedian(finite)) > 100.0:
                depth_metres = depth_metres * 0.001
            height, width = depth_metres.shape[:2]
            half_roi = max(min(height, width) // 20, 4)
            center_y = height // 2
            center_x = width // 2
            roi = depth_metres[
                center_y - half_roi:center_y + half_roi,
                center_x - half_roi:center_x + half_roi,
            ]
            valid = roi[
                np.isfinite(roi)
                & (roi >= 0.15)
                & (roi <= 30.0)
            ]
            if valid.size >= 8:
                self.latest_ground_depth = float(np.median(valid))
                self.last_ground_depth_monotonic = time.monotonic()
        except Exception as error:
            self.get_logger().warning(
                f'Cannot decode depth image: {error}',
                throttle_duration_sec=2.0)

    def _camera_info_callback(self, message):
        if len(message.k) >= 5 and message.k[0] > 0 and message.k[4] > 0:
            self.focal_xy = (float(message.k[0]), float(message.k[4]))

    def _image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().warning(
                f'Cannot decode RGB image: {error}',
                throttle_duration_sec=2.0)
            return
        detection = identify_marker(
            image,
            self.detector,
            self.marker_id,
            self.minimum_detection_confidence,
        )
        corners = (
            detection['corners'] if detection is not None else None)
        depth = None
        if corners is not None and self.depth_frames:
            stamp = (
                int(message.header.stamp.sec)
                + 1e-9 * int(message.header.stamp.nanosec)
            )
            depth_stamp, depth_image = min(
                self.depth_frames,
                key=lambda frame: abs(stamp - frame[0]),
            )
            if abs(stamp - depth_stamp) <= self.depth_sync_tolerance:
                depth = depth_at_marker(depth_image, corners)
        annotated = image.copy()
        detection_payload = {
            'detected': detection is not None,
            'expected_id': self.marker_id,
            'id': None,
            'confidence': 0.0,
            'source': None,
            'center_px': None,
            'depth_m': None,
            'depth_valid': False,
        }
        if corners is not None:
            polygon = np.rint(corners).astype(np.int32)
            cv2.polylines(annotated, [polygon], True, (0, 255, 0), 2)
            center = np.mean(corners, axis=0)
            detection_payload.update({
                'id': detection['id'],
                'confidence': round(float(detection['confidence']), 4),
                'source': detection['source'],
                'center_px': [
                    round(float(center[0]), 2),
                    round(float(center[1]), 2),
                ],
                'depth_m': (
                    round(float(depth), 3) if depth is not None else None),
                'depth_valid': depth is not None,
            })
            cv2.circle(
                annotated, tuple(np.rint(center).astype(int)),
                5, (0, 0, 255), -1)
            if depth is not None:
                received_monotonic = time.monotonic()
                self.latest_observation = {
                    'center': center,
                    'shape': image.shape,
                    'depth': depth,
                    'id': detection['id'],
                    'confidence': detection['confidence'],
                    'source': detection['source'],
                    'received_monotonic': received_monotonic,
                }
                self.last_detection_monotonic = received_monotonic
                cv2.putText(
                    annotated,
                    (
                        f'id={detection["id"]} '
                        f'conf={detection["confidence"]:.2f} '
                        f'depth={depth:.2f}m'
                    ),
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
            else:
                cv2.putText(
                    annotated,
                    (
                        f'id={detection["id"]} '
                        f'conf={detection["confidence"]:.2f} '
                        'waiting for depth'
                    ),
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 200, 255),
                    2,
                )
        cv2.putText(
            annotated,
            f'mode={self.current_mode}',
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2,
        )
        estimated_speed = float(np.linalg.norm(
            self.estimated_panther_velocity_xy))
        cv2.putText(
            annotated,
            (
                'origin=UAV camera  '
                f'Panther velocity=('
                f'{self.estimated_panther_velocity_xy[0]:.2f}, '
                f'{self.estimated_panther_velocity_xy[1]:.2f}) m/s  '
                f'speed={estimated_speed:.2f} m/s'
            ),
            (10, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 220, 80),
            1,
        )
        self.detection_pub.publish(String(data=json.dumps(detection_payload)))
        try:
            debug = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            debug.header = message.header
            self.debug_image_pub.publish(debug)
        except Exception as error:
            self.get_logger().warning(
                f'Cannot publish debug image: {error}',
                throttle_duration_sec=2.0)

    def _vehicle_status_callback(self, message):
        self.vehicle_status = message

    def _local_position_callback(self, message):
        self.local_position = message

    def _command_ack_callback(self, message):
        if message.command not in (
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        ):
            return
        if message.result not in (
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
        ):
            self.get_logger().warning(
                f'PX4 rejected command {message.command}: '
                f'result={message.result} detail={message.result_param2}')

    def _harvest_status_callback(self, message):
        try:
            state = str(json.loads(message.data).get('state', 'UNKNOWN'))
        except (json.JSONDecodeError, AttributeError):
            return
        previous = self.harvest_state
        self.harvest_state = state
        self.route_complete = state == 'COMPLETE'
        if (
                state == 'RUNNING'
                and self.harvest_started_monotonic is None):
            self.harvest_started_monotonic = time.monotonic()
            self.get_logger().info(
                'Panther route is RUNNING. Giving the UGV a '
                f'{self.search_start_delay:.1f} s head start before UAV '
                'takeoff and route-prior search.')
        if state == 'RUNNING' and previous != 'RUNNING':
            self.get_logger().info('Panther motion detected from mission state.')

    def _planned_path_callback(self, message):
        route = [self.ugv_origin]
        route.extend(
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
            )
            for pose in message.poses
        )
        self.planned_route = route
        self.get_logger().info(
            f'Received {len(route)} route-prior points for visual search.',
            once=True,
        )

    def _landing_service(self, _, response):
        response.success = False
        response.message = (
            'RL landing is intentionally reserved for the next phase. '
            'The UAV will continue visual hover over the completed UGV route.')
        return response

    def _start_locating_service(self, _, response):
        """Authorize the UAV search/follow state machine from the camera UI."""
        if self.locating_requested:
            response.success = True
            response.message = 'Locating is already authorized.'
            return response
        self.locating_requested = True
        response.success = True
        if self.harvest_started_monotonic is None:
            response.message = (
                'Locating armed; waiting for the Panther route to start.')
        else:
            remaining = max(
                self.search_start_delay
                - (time.monotonic() - self.harvest_started_monotonic),
                0.0,
            )
            response.message = (
                f'Locating authorized; UGV head-start remaining: '
                f'{remaining:.1f} s.'
                if remaining > 0.0
                else 'Locating authorized; search is starting now.'
            )
        self.get_logger().info(response.message)
        return response

    def _local_position_ready(self):
        message = self.local_position
        return bool(
            message is not None
            and message.xy_valid
            and message.z_valid
            and math.isfinite(message.x)
            and math.isfinite(message.y)
            and math.isfinite(message.z)
            and math.isfinite(message.heading)
        )

    def _vertical_state_ready(self):
        """Return whether height and heading are safe for bootstrap takeoff."""
        message = self.local_position
        return bool(
            message is not None
            and message.z_valid
            and math.isfinite(message.z)
            and math.isfinite(message.heading)
        )

    def _is_armed(self):
        return bool(
            self.vehicle_status is not None
            and self.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED)

    def _is_offboard(self):
        return bool(
            self.vehicle_status is not None
            and self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _publish_control_mode(
            self, position_control=False, acceleration=False):
        message = OffboardControlMode()
        message.timestamp = self._timestamp_us()
        message.position = bool(position_control)
        message.velocity = False
        message.acceleration = bool(acceleration)
        self.offboard_pub.publish(message)

    def _publish_position_setpoint(self, position, heading):
        message = TrajectorySetpoint()
        message.timestamp = self._timestamp_us()
        message.position = [float(value) for value in position]
        message.velocity = [math.nan, math.nan, math.nan]
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = float(heading)
        message.yawspeed = math.nan
        self.setpoint_pub.publish(message)

    def _publish_velocity_setpoint(self, velocity, heading):
        message = TrajectorySetpoint()
        message.timestamp = self._timestamp_us()
        message.position = [math.nan, math.nan, math.nan]
        message.velocity = [float(value) for value in velocity]
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = float(heading)
        message.yawspeed = math.nan
        self.setpoint_pub.publish(message)

    def _publish_acceleration_setpoint(self, acceleration, heading):
        message = TrajectorySetpoint()
        message.timestamp = self._timestamp_us()
        message.position = [math.nan, math.nan, math.nan]
        message.velocity = [math.nan, math.nan, math.nan]
        message.acceleration = [
            float(value) for value in acceleration]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = float(heading)
        message.yawspeed = math.nan
        self.setpoint_pub.publish(message)

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        message = VehicleCommand()
        message.timestamp = self._timestamp_us()
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.command = int(command)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_pub.publish(message)

    def _maybe_arm_and_enter_offboard(self):
        self.setpoint_count += 1
        if self.setpoint_count < self.startup_setpoints:
            return
        if not self._vertical_state_ready():
            return
        now = time.monotonic()
        if now - self.last_command_monotonic < self.command_retry_seconds:
            return
        if self.auto_offboard and not self._is_offboard():
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if self.auto_arm and not self._is_armed():
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.last_command_monotonic = now

    def _search_enabled(self, now=None):
        """Return whether operator authorization and UGV head start are ready."""
        now = time.monotonic() if now is None else float(now)
        enabled = locating_gate_ready(
            self.manual_start_required,
            self.locating_requested,
            self.harvest_started_monotonic,
            now,
            self.search_start_delay,
        )
        if enabled and self.locating_started_monotonic is None:
            self.locating_started_monotonic = now
        return enabled

    def _waiting_mode(self):
        """Describe the condition currently holding the UAV stationary."""
        if self.manual_start_required and not self.locating_requested:
            return 'WAITING_FOR_OPERATOR'
        if self.harvest_started_monotonic is None:
            return 'WAITING_FOR_UGV'
        return 'UGV_HEAD_START'

    def _integrate_gazebo_motion(self, now):
        """Dead-reckon the deterministic Gazebo velocity controller."""
        dt = float(np.clip(
            now - self.last_motion_tick_monotonic, 0.0, 0.20))
        self.estimated_world_xy += dt * np.asarray(
            self.last_gazebo_xy_command, dtype=np.float64)
        self.last_motion_tick_monotonic = now

    def _publish_gazebo_velocity(self, velocity_x, velocity_y, velocity_z):
        command = Twist()
        command.linear.x = float(velocity_x)
        command.linear.y = float(velocity_y)
        command.linear.z = float(velocity_z)
        self.gazebo_velocity_pub.publish(command)
        self.last_gazebo_xy_command = (
            float(velocity_x), float(velocity_y))

    def _remember_marker_world_position(self, observation):
        """Estimate the marker XY from camera geometry for local reacquisition."""
        center = observation['center']
        height, width = observation['shape'][:2]
        error_u = float(center[0]) - 0.5 * width
        error_v = float(center[1]) - 0.5 * height
        depth = float(observation['depth'])
        offset = np.asarray((
            -error_v * depth / max(float(self.focal_xy[1]), 1.0),
            -error_u * depth / max(float(self.focal_xy[0]), 1.0),
        ))
        self.last_marker_world_xy = self.estimated_world_xy + offset

    def _update_panther_velocity_estimate(self, observation):
        """Estimate Panther XY velocity in the UAV-aligned camera frame."""
        observation_time = float(observation['received_monotonic'])
        relative_xy = marker_position_relative_to_uav(
            observation['center'],
            observation['shape'],
            observation['depth'],
            self.focal_xy,
        )
        self.marker_relative_xy = relative_xy
        if observation_time == self.last_velocity_observation_monotonic:
            return
        if (
                self.previous_marker_relative_xy is not None
                and self.previous_marker_observation_monotonic is not None):
            dt = (
                observation_time
                - self.previous_marker_observation_monotonic
            )
            if 0.03 <= dt <= 0.75:
                self.estimated_panther_velocity_xy = filtered_target_velocity(
                    relative_xy,
                    self.previous_marker_relative_xy,
                    dt,
                    self.last_gazebo_xy_command,
                    self.estimated_panther_velocity_xy,
                    self.target_velocity_filter_alpha,
                    self.max_estimated_target_speed,
                )
        self.previous_marker_relative_xy = relative_xy.copy()
        self.previous_marker_observation_monotonic = observation_time
        self.last_velocity_observation_monotonic = observation_time

    def _select_search_target(self, now):
        """Choose local reacquisition, route intercept, or coverage search."""
        mission_elapsed = max(
            now - self.harvest_started_monotonic, 0.0)
        search_elapsed = max(
            now - self.locating_started_monotonic, 0.0
        ) if self.locating_started_monotonic is not None else 0.0
        if (
                self.last_marker_world_xy is not None
                and self.last_detection_monotonic is not None
                and now - self.last_detection_monotonic
                <= self.local_reacquire_timeout):
            target = expanding_square_target(
                self.last_marker_world_xy,
                max(
                    now - self.last_detection_monotonic
                    - self.marker_timeout,
                    0.0,
                ),
                self.max_horizontal_speed,
                self.fallback_search_spacing,
            )
            return target, 'LOCAL_REACQUIRE_SEARCH'
        if self.planned_route:
            target = route_intercept_target(
                self.planned_route,
                mission_elapsed,
                search_elapsed,
                self.ugv_nominal_speed,
                self.route_sweep_distance,
                self.route_sweep_period,
            )
            return target, 'ROUTE_INTERCEPT_SEARCH'
        target = expanding_square_target(
            self.ugv_origin,
            search_elapsed,
            self.max_horizontal_speed,
            self.fallback_search_spacing,
        )
        return target, 'EXPANDING_SQUARE_SEARCH'

    def _fresh_observation(self):
        return bool(
            self.latest_observation is not None
            and self.last_detection_monotonic is not None
            and time.monotonic() - self.last_detection_monotonic
            <= self.marker_timeout)

    def _height_above_ground(self):
        message = self.local_position
        if (
                message is not None
                and message.dist_bottom_valid
                and math.isfinite(message.dist_bottom)):
            return max(float(message.dist_bottom), 0.0)
        if self._vertical_state_ready():
            return max(-float(message.z), 0.0)
        return 0.0

    def _update_readiness(self, pixel_error=None, depth=None):
        stable = False
        if pixel_error is not None and depth is not None:
            shape = self.latest_observation['shape']
            normalized_error = math.hypot(
                pixel_error[0] / max(shape[1] * 0.5, 1.0),
                pixel_error[1] / max(shape[0] * 0.5, 1.0),
            )
            stable = (
                normalized_error <= 0.12
                and abs(depth - self.desired_depth) <= 1.0
                and (
                    self.control_backend == 'gazebo_velocity'
                    or (self._is_armed() and self._is_offboard())
                )
            )
        if stable:
            if self.stable_since is None:
                self.stable_since = time.monotonic()
            elif time.monotonic() - self.stable_since >= 1.0:
                self.ever_ready = True
        else:
            self.stable_since = None
        critical_loss = bool(
            self.ever_ready
            and self.last_detection_monotonic is not None
            and time.monotonic() - self.last_detection_monotonic
            > self.critical_loss_timeout)
        ready = self.ever_ready and not critical_loss
        self.ready_pub.publish(Bool(data=ready))
        self.handoff_pub.publish(Bool(
            data=ready and self.route_complete and stable))
        return ready

    def _publish_status(self, mode, ready, depth=None, error=None):
        observation = (
            self.latest_observation if self._fresh_observation() else None)
        payload = {
            'mode': mode,
            'ready': ready,
            'gps_used': False,
            'tracking_origin': 'uav_camera',
            'harvest_state': self.harvest_state,
            'manual_start_required': self.manual_start_required,
            'locating_requested': self.locating_requested,
            'locating_active': self.locating_started_monotonic is not None,
            'search_delay_s': self.search_start_delay,
            'head_start_remaining_s': (
                round(max(
                    self.search_start_delay
                    - (time.monotonic() - self.harvest_started_monotonic),
                    0.0,
                ), 2)
                if self.harvest_started_monotonic is not None else None
            ),
            'estimated_uav_xy': [
                round(float(value), 2)
                for value in self.estimated_world_xy
            ],
            'search_target_xy': (
                [round(float(value), 2) for value in self.search_target_xy]
                if self.search_target_xy is not None else None
            ),
            'control_backend': self.control_backend,
            'armed': self._is_armed(),
            'offboard': self._is_offboard(),
            'local_position_valid': self._local_position_ready(),
            'vertical_state_valid': self._vertical_state_ready(),
            'tag_visible': self._fresh_observation(),
            'marker_relative_xy_m': (
                [round(float(value), 3) for value in self.marker_relative_xy]
                if observation is not None
                and self.marker_relative_xy is not None else None
            ),
            'estimated_panther_velocity_mps': [
                round(float(value), 3)
                for value in self.estimated_panther_velocity_xy
            ],
            'estimated_panther_speed_mps': round(float(np.linalg.norm(
                self.estimated_panther_velocity_xy)), 3),
            'detected_marker_id': (
                observation['id'] if observation is not None else None),
            'detection_confidence': (
                observation['confidence']
                if observation is not None else None),
            'detection_source': (
                observation['source'] if observation is not None else None),
            'depth_m': depth,
            'pixel_error': error,
            'route_complete': self.route_complete,
            'rl_landing_implemented': False,
        }
        text = json.dumps(payload)
        self.status_pub.publish(String(data=text))
        if text != self.last_status_text:
            self.get_logger().info(text, throttle_duration_sec=1.0)
            self.last_status_text = text

    def _fresh_ground_depth(self):
        return bool(
            self.latest_ground_depth is not None
            and self.last_ground_depth_monotonic is not None
            and time.monotonic() - self.last_ground_depth_monotonic
            <= self.ground_depth_timeout)

    def _tick_gazebo_velocity(self):
        """Run deterministic GPS-free visual servoing in Gazebo velocity mode."""
        now = time.monotonic()
        self._integrate_gazebo_motion(now)
        if not self._search_enabled(now):
            self.search_target_xy = None
            mode = self._waiting_mode()
            self.current_mode = mode
            self._publish_gazebo_velocity(0.0, 0.0, 0.0)
            ready = self._update_readiness()
            self._publish_status(mode, ready)
            return

        observation = (
            self.latest_observation if self._fresh_observation() else None)
        pixel_error = None
        tag_depth = (
            float(observation['depth'])
            if observation is not None else None)
        velocity_x = 0.0
        velocity_y = 0.0
        if observation is not None:
            self.search_target_xy = None
            self._update_panther_velocity_estimate(observation)
            self._remember_marker_world_position(observation)
            (velocity_x, velocity_y), pixel_error = (
                gazebo_visual_velocity(
                    observation['center'],
                    observation['shape'],
                    tag_depth,
                    self.focal_xy,
                    self.horizontal_gain,
                    self.max_horizontal_speed,
                    target_velocity=self.estimated_panther_velocity_xy,
                    feedforward_gain=self.velocity_feedforward_gain,
                )
            )
            height_feedback = tag_depth
            mode = 'VISUAL_FOLLOW_VELOCITY'
        elif self._fresh_ground_depth():
            height_feedback = float(self.latest_ground_depth)
            if height_feedback >= self.visual_engage_altitude:
                self.search_target_xy, mode = self._select_search_target(now)
                velocity_x, velocity_y = bounded_velocity_toward(
                    self.estimated_world_xy,
                    self.search_target_xy,
                    self.max_horizontal_speed,
                    self.search_arrival_radius,
                )
            else:
                self.search_target_xy = None
                mode = 'SEARCH_TAKEOFF'
        else:
            self.search_target_xy = None
            height_feedback = None
            mode = 'WAITING_FOR_DEPTH'

        velocity_z = 0.0
        if height_feedback is not None:
            velocity_z = float(np.clip(
                self.vertical_gain
                * (self.takeoff_altitude - height_feedback),
                -self.max_vertical_speed,
                self.max_vertical_speed,
            ))
        else:
            # The model is intentionally spawned without GPS or a world-pose
            # input.  A small, bounded upward command prevents it from settling
            # into uneven terrain before the first downward depth frame arrives.
            velocity_z = min(
                self.bootstrap_climb_speed,
                self.max_vertical_speed,
            )
        self.current_mode = mode
        self._publish_gazebo_velocity(
            velocity_x, velocity_y, velocity_z)
        ready = self._update_readiness(pixel_error, tag_depth)
        self._publish_status(mode, ready, tag_depth, pixel_error)

    def _tick(self):
        if self.control_backend == 'gazebo_velocity':
            self._tick_gazebo_velocity()
            return

        now = time.monotonic()
        if not self._search_enabled(now):
            mode = self._waiting_mode()
            self.current_mode = mode
            self.search_target_xy = None
            ready = self._update_readiness()
            self._publish_status(mode, ready)
            return

        heading = (
            float(self.local_position.heading)
            if self._vertical_state_ready() else 0.0)
        if self._vertical_state_ready() and self.yaw_target is None:
            self.yaw_target = heading
        yaw_target = (
            float(self.yaw_target)
            if self.yaw_target is not None else heading)
        altitude = self._height_above_ground()
        observation = (
            self.latest_observation if self._fresh_observation() else None)
        pixel_error = None
        depth = (
            float(observation['depth'])
            if observation is not None else None)

        # Use tag pixels directly for horizontal acceleration. This PX4 mode
        # requires attitude only, not GPS or a valid local XY estimate, while
        # PX4 safely generates the attitude and motor thrust internally.
        horizontal_acceleration = (0.0, 0.0)
        if (
                observation is not None
                and altitude >= self.visual_engage_altitude):
            center = observation['center']
            height, width = observation['shape'][:2]
            current_error = (
                float(center[0]) - 0.5 * width,
                float(center[1]) - 0.5 * height,
            )
            observation_time = float(
                observation['received_monotonic'])
            error_rate = (0.0, 0.0)
            if (
                    observation_time != self.last_processed_observation
                    and self.previous_pixel_error is not None
                    and self.previous_error_monotonic is not None):
                dt = max(
                    observation_time - self.previous_error_monotonic,
                    0.02,
                )
                error_rate = (
                    (current_error[0] - self.previous_pixel_error[0]) / dt,
                    (current_error[1] - self.previous_pixel_error[1]) / dt,
                )
            if observation_time != self.last_processed_observation:
                self.previous_pixel_error = current_error
                self.previous_error_monotonic = observation_time
                self.last_processed_observation = observation_time
                self.last_visual_command, pixel_error = visual_acceleration(
                    observation['center'],
                    observation['shape'],
                    yaw_target,
                    error_rate,
                    self.horizontal_acceleration_gain,
                    self.horizontal_damping_gain,
                    self.max_horizontal_acceleration,
                )
            else:
                pixel_error = current_error
            horizontal_acceleration = self.last_visual_command
            height_feedback = depth
            mode = 'VISUAL_FOLLOW_ACCELERATION'
        else:
            self.previous_pixel_error = None
            self.previous_error_monotonic = None
            self.last_processed_observation = None
            self.last_visual_command = (0.0, 0.0)
            height_feedback = altitude
            mode = (
                'BOOTSTRAP_CLIMB'
                if altitude < self.takeoff_altitude
                else 'TAG_SEARCH_HOVER')
        vertical_velocity = (
            float(self.local_position.vz)
            if self._vertical_state_ready()
            and math.isfinite(self.local_position.vz)
            else 0.0
        )
        acceleration_down = height_acceleration(
            height_feedback,
            self.takeoff_altitude,
            vertical_velocity,
            self.height_acceleration_gain,
            self.vertical_damping_gain,
            self.max_vertical_acceleration,
        )
        self._publish_control_mode(acceleration=True)
        self._publish_acceleration_setpoint(
            (
                horizontal_acceleration[0],
                horizontal_acceleration[1],
                acceleration_down,
            ),
            yaw_target,
        )
        self._maybe_arm_and_enter_offboard()
        self.current_mode = mode
        ready = self._update_readiness(pixel_error, depth)
        self._publish_status(mode, ready, depth, pixel_error)


def main(args=None):
    """Run the visual tag follower."""
    rclpy.init(args=args)
    node = VisualTagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
