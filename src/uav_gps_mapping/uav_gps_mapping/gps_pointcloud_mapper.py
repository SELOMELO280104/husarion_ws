"""Accumulate Gazebo LiDAR scans using PX4's GPS/INS pose."""

from collections import deque
from dataclasses import dataclass
from datetime import datetime
from itertools import pairwise
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
import numpy as np
from pymavlink import mavutil
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger

from uav_gps_mapping.pcd_to_occupancy import (
    save_traversability_products,
)


@dataclass
class IcpResult:
    """Bounded rigid correction estimated from a GPS-seeded scan."""

    rotation: np.ndarray
    translation: np.ndarray
    accepted: bool
    correspondences: int
    rmse_before: float
    rmse_after: float
    translation_correction: float
    rotation_correction_deg: float
    reason: str

    def transform(self, points):
        """Apply the estimated world-frame correction."""
        return points @ self.rotation.T + self.translation


def voxel_centroids(points, voxel_size):
    """Downsample points to one centroid per cubic voxel."""
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float64)
    keys = np.floor(points / voxel_size).astype(np.int64)
    _unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((int(inverse.max()) + 1, 3), dtype=np.float64)
    np.add.at(sums, inverse, points)
    counts = np.bincount(inverse)
    return sums / counts[:, np.newaxis]


def estimate_rigid_transform(source, target):
    """Return the proper rigid transform mapping source onto target."""
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (
        (source - source_center).T @ (target - target_center))
    left, _singular, right_transpose = np.linalg.svd(covariance)
    rotation = right_transpose.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1, :] *= -1.0
        rotation = right_transpose.T @ left.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def rotation_angle_degrees(rotation):
    """Return the unsigned angle represented by a rotation matrix."""
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(cosine))


def _icp_correspondences(tree, points, maximum_distance, trim_fraction):
    distances, indices = tree.query(
        points,
        k=1,
        distance_upper_bound=maximum_distance,
        workers=1,
    )
    valid = np.isfinite(distances)
    if not np.any(valid):
        return valid, indices, float('inf')
    finite_distances = distances[valid]
    trim_distance = min(
        maximum_distance,
        float(np.quantile(finite_distances, trim_fraction)),
    )
    valid &= distances <= trim_distance
    if not np.any(valid):
        return valid, indices, float('inf')
    rmse = float(np.sqrt(np.mean(np.square(distances[valid]))))
    return valid, indices, rmse


def gps_seeded_icp(
    source,
    target,
    *,
    target_tree=None,
    maximum_correspondence_distance=0.60,
    maximum_iterations=8,
    minimum_correspondences=150,
    maximum_translation=0.40,
    maximum_rotation_degrees=3.0,
    minimum_relative_improvement=0.08,
    trim_fraction=0.85,
    minimum_geometry_ratio=0.002,
    translation_only=False,
):
    """Align a GPS-seeded scan to a map while retaining a strong GPS bound."""
    identity = np.eye(3, dtype=np.float64)
    zero = np.zeros(3, dtype=np.float64)

    def rejected(reason, correspondences=0, before=float('inf'),
                 after=float('inf'), translation=0.0, rotation=0.0):
        return IcpResult(
            identity, zero, False, correspondences, before, after,
            translation, rotation, reason)

    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if len(source) < minimum_correspondences:
        return rejected('too few source points')
    if len(target) < minimum_correspondences:
        return rejected('reference map is still bootstrapping')
    tree = target_tree if target_tree is not None else cKDTree(target)
    transformed = source.copy()
    rotation_total = identity.copy()
    translation_total = zero.copy()
    initial_valid, _initial_indices, rmse_before = _icp_correspondences(
        tree, transformed, maximum_correspondence_distance, trim_fraction)
    initial_count = int(np.count_nonzero(initial_valid))
    if initial_count < minimum_correspondences:
        return rejected(
            'too few initial correspondences',
            initial_count,
            rmse_before,
        )

    final_count = initial_count
    rmse_after = rmse_before
    for _iteration in range(maximum_iterations):
        valid, indices, rmse = _icp_correspondences(
            tree,
            transformed,
            maximum_correspondence_distance,
            trim_fraction,
        )
        final_count = int(np.count_nonzero(valid))
        if final_count < minimum_correspondences:
            return rejected(
                'correspondences fell below threshold',
                final_count,
                rmse_before,
                rmse,
            )
        matched_source = transformed[valid]
        matched_target = target[indices[valid]]
        geometry = np.cov(matched_source, rowvar=False)
        eigenvalues = np.linalg.eigvalsh(geometry)
        if (
            eigenvalues[-1] <= 0.0
            or eigenvalues[0] / eigenvalues[-1] < minimum_geometry_ratio
        ):
            return rejected(
                'scan geometry is degenerate',
                final_count,
                rmse_before,
                rmse,
            )
        if translation_only:
            rotation_step = identity
            translation_step = (
                matched_target.mean(axis=0)
                - matched_source.mean(axis=0))
        else:
            rotation_step, translation_step = estimate_rigid_transform(
                matched_source, matched_target)
        transformed = (
            transformed @ rotation_step.T + translation_step)
        rotation_total = rotation_step @ rotation_total
        translation_total = (
            rotation_step @ translation_total + translation_step)
        rmse_after = rmse
        centroid_shift = np.linalg.norm(
            source.mean(axis=0) @ rotation_total.T
            + translation_total
            - source.mean(axis=0))
        rotation_degrees = rotation_angle_degrees(rotation_total)
        if (
            centroid_shift > maximum_translation
            or rotation_degrees > maximum_rotation_degrees
        ):
            return rejected(
                'correction exceeded the GPS safety bound',
                final_count,
                rmse_before,
                rmse_after,
                float(centroid_shift),
                float(rotation_degrees),
            )
        step_shift = np.linalg.norm(
            matched_source.mean(axis=0) @ rotation_step.T
            + translation_step
            - matched_source.mean(axis=0))
        if (
            step_shift < 0.002
            and rotation_angle_degrees(rotation_step) < 0.02
        ):
            break

    final_valid, _final_indices, rmse_after = _icp_correspondences(
        tree, transformed, maximum_correspondence_distance, trim_fraction)
    final_count = int(np.count_nonzero(final_valid))
    centroid_shift = float(np.linalg.norm(
        source.mean(axis=0) @ rotation_total.T
        + translation_total
        - source.mean(axis=0)))
    rotation_degrees = float(rotation_angle_degrees(rotation_total))
    improvement = (
        (rmse_before - rmse_after) / rmse_before
        if rmse_before > 0.0 else 0.0)
    if final_count < minimum_correspondences:
        return rejected(
            'too few final correspondences',
            final_count,
            rmse_before,
            rmse_after,
            centroid_shift,
            rotation_degrees,
        )
    if improvement < minimum_relative_improvement:
        return rejected(
            'insufficient fit improvement',
            final_count,
            rmse_before,
            rmse_after,
            centroid_shift,
            rotation_degrees,
        )
    return IcpResult(
        rotation_total,
        translation_total,
        True,
        final_count,
        rmse_before,
        rmse_after,
        centroid_shift,
        rotation_degrees,
        'accepted',
    )


def statistical_outlier_mask(
    points,
    *,
    neighbor_count=12,
    stddev_multiplier=1.5,
    chunk_size=100000,
):
    """Return a PCL-style statistical outlier mask with bounded memory use."""
    points = np.asarray(points, dtype=np.float64)
    if len(points) < 3:
        return np.ones(len(points), dtype=bool)
    query_count = min(len(points), max(2, int(neighbor_count) + 1))
    tree = cKDTree(points)
    mean_distances = np.empty(len(points), dtype=np.float64)
    for start in range(0, len(points), max(1, int(chunk_size))):
        stop = min(start + max(1, int(chunk_size)), len(points))
        distances, _indices = tree.query(
            points[start:stop],
            k=query_count,
            workers=1,
        )
        if distances.ndim == 1:
            distances = distances[:, np.newaxis]
        mean_distances[start:stop] = np.mean(
            distances[:, 1:], axis=1)
    finite = np.isfinite(mean_distances)
    if not np.any(finite):
        return np.zeros(len(points), dtype=bool)
    mean = float(np.mean(mean_distances[finite]))
    deviation = float(np.std(mean_distances[finite]))
    threshold = mean + max(0.0, float(stddev_multiplier)) * deviation
    return finite & (mean_distances <= threshold)


def write_colored_ascii_pcd(path, points, colors, registration_comment):
    """Write XYZRGB points as an ASCII PCD that pcl_viewer can open."""
    path = Path(path)
    packed_rgb = (
        colors[:, 0].astype(np.uint32) << 16
        | colors[:, 1].astype(np.uint32) << 8
        | colors[:, 2].astype(np.uint32))
    rgb_float = packed_rgb.view(np.float32)
    with path.open('x', encoding='ascii') as stream:
        stream.write(
            '# .PCD v0.7 - Point Cloud Data file format\n'
            f'# Registration: {registration_comment}\n'
            'VERSION 0.7\n'
            'FIELDS x y z rgb\n'
            'SIZE 4 4 4 4\n'
            'TYPE F F F F\n'
            'COUNT 1 1 1 1\n'
            f'WIDTH {len(points)}\n'
            'HEIGHT 1\n'
            'VIEWPOINT 0 0 0 1 0 0 0\n'
            f'POINTS {len(points)}\n'
            'DATA ascii\n')
        for (x, y, z), rgb in zip(points, rgb_float):
            stream.write(
                f'{x:.7g} {y:.7g} {z:.7g} {rgb:.9g}\n')


def quaternion_matrix(w, x, y, z):
    """Return the body-FRD to earth-NED rotation matrix."""
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        return np.eye(3)
    w, x, y, z = (value / norm for value in (w, x, y, z))
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def normalized_quaternion(values):
    quaternion = np.asarray(values, dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return quaternion / norm


def interpolate_vector(samples, timestamp, max_gap=0.25):
    """Linearly interpolate timestamped numpy vectors."""
    if not samples:
        return None
    if timestamp <= samples[0][0]:
        if samples[0][0] - timestamp <= max_gap:
            return samples[0][1].copy()
        return None
    if timestamp >= samples[-1][0]:
        if timestamp - samples[-1][0] <= max_gap:
            return samples[-1][1].copy()
        return None
    for (time_a, value_a), (time_b, value_b) in pairwise(samples):
        if time_a <= timestamp <= time_b:
            if time_b - time_a > max_gap:
                return None
            fraction = (
                0.0 if time_b == time_a
                else (timestamp - time_a) / (time_b - time_a))
            return value_a + fraction * (value_b - value_a)
    return None


def interpolate_quaternion(samples, timestamp, max_gap=0.25):
    """Spherically interpolate timestamped w,x,y,z quaternions."""
    if not samples:
        return None
    if timestamp <= samples[0][0]:
        if samples[0][0] - timestamp <= max_gap:
            return samples[0][1].copy()
        return None
    if timestamp >= samples[-1][0]:
        if timestamp - samples[-1][0] <= max_gap:
            return samples[-1][1].copy()
        return None
    for (time_a, value_a), (time_b, value_b) in pairwise(samples):
        if time_a <= timestamp <= time_b:
            if time_b - time_a > max_gap:
                return None
            fraction = (
                0.0 if time_b == time_a
                else (timestamp - time_a) / (time_b - time_a))
            quaternion_a = normalized_quaternion(value_a)
            quaternion_b = normalized_quaternion(value_b)
            dot = float(np.dot(quaternion_a, quaternion_b))
            if dot < 0.0:
                quaternion_b = -quaternion_b
                dot = -dot
            dot = float(np.clip(dot, -1.0, 1.0))
            if dot > 0.9995:
                return normalized_quaternion(
                    quaternion_a
                    + fraction * (quaternion_b - quaternion_a))
            angle = math.acos(dot)
            sine = math.sin(angle)
            return (
                math.sin((1.0 - fraction) * angle) / sine * quaternion_a
                + math.sin(fraction * angle) / sine * quaternion_b)
    return None


def message_stamp_seconds(message):
    return (
        float(message.header.stamp.sec)
        + float(message.header.stamp.nanosec) * 1e-9)


def decode_rgb_image(message):
    """Convert a ROS Image into an RGB uint8 numpy array."""
    encodings = {
        'rgb8': (3, (0, 1, 2)),
        'bgr8': (3, (2, 1, 0)),
        'rgba8': (4, (0, 1, 2)),
        'bgra8': (4, (2, 1, 0)),
    }
    if message.encoding.lower() not in encodings:
        raise ValueError(f'Unsupported RGB encoding: {message.encoding}')
    channels, order = encodings[message.encoding.lower()]
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step)
    pixels = rows[:, :message.width * channels].reshape(
        message.height, message.width, channels)
    return pixels[:, :, order].copy()


def project_rgb_to_lidar(
    points_lidar,
    image,
    intrinsics,
    rotation_sensor_to_flu,
    lidar_offset_flu,
    camera_offset_flu,
):
    """Project LiDAR-frame points into Gazebo's X-forward camera image."""
    points_flu = (
        points_lidar @ rotation_sensor_to_flu.T + lidar_offset_flu)
    points_camera = (
        points_flu - camera_offset_flu) @ rotation_sensor_to_flu
    forward = points_camera[:, 0]
    fx, fy, cx, cy = intrinsics
    valid = forward > 0.05
    u = np.zeros(len(points_lidar), dtype=np.int64)
    v = np.zeros(len(points_lidar), dtype=np.int64)
    u[valid] = np.rint(
        cx - fx * points_camera[valid, 1] / forward[valid]).astype(
            np.int64)
    v[valid] = np.rint(
        cy - fy * points_camera[valid, 2] / forward[valid]).astype(
            np.int64)
    valid &= (
        (u >= 0) & (u < image.shape[1])
        & (v >= 0) & (v < image.shape[0])
    )
    colors = np.full((len(points_lidar), 3), 128, dtype=np.uint8)
    colors[valid] = image[v[valid], u[valid]]
    return colors, valid


class GpsPointCloudMapper(Node):
    """Transform LiDAR scans with PX4 EKF GPS/INS position and attitude."""

    def __init__(self):
        super().__init__('gps_pointcloud_mapper')
        self.declare_parameter(
            'cloud_topic', '/uav/gps_mapping/lidar/points')
        self.declare_parameter('output_directory', 'maps')
        self.declare_parameter('mavlink_endpoint', 'udpin:127.0.0.1:14540')
        self.declare_parameter('voxel_size', 0.10)
        self.declare_parameter('max_points', 3000000)
        self.declare_parameter('scan_delay', 0.15)
        self.declare_parameter('max_lidar_range', 18.0)
        self.declare_parameter('min_downward_angle_deg', 35.0)
        self.declare_parameter(
            'rgb_topic', '/uav/gps_mapping/rgb/image')
        self.declare_parameter(
            'camera_info_topic', '/uav/gps_mapping/rgb/camera_info')
        self.declare_parameter('enable_rgb', True)
        self.declare_parameter('max_rgb_time_offset', 0.06)
        self.declare_parameter('enable_icp', True)
        self.declare_parameter('icp_translation_only', True)
        self.declare_parameter('icp_reference_voxel_size', 0.25)
        self.declare_parameter('icp_source_voxel_size', 0.20)
        self.declare_parameter('icp_max_correspondence_distance', 0.35)
        self.declare_parameter('icp_max_translation', 0.20)
        self.declare_parameter('icp_max_rotation_deg', 0.50)
        self.declare_parameter('icp_min_height', 0.35)
        self.declare_parameter('icp_max_height', 1.30)
        self.declare_parameter('icp_max_source_points', 3000)
        self.declare_parameter('icp_min_correspondences', 150)
        self.declare_parameter('icp_rebuild_interval', 5)
        self.declare_parameter('enable_statistical_filter', True)
        self.declare_parameter('outlier_mean_k', 12)
        self.declare_parameter('outlier_stddev_multiplier', 1.5)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', -5.0)
        self.declare_parameter('origin_z', 0.2)

        self.output_directory = Path(
            str(self.get_parameter('output_directory').value)).expanduser()
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.scan_delay = float(self.get_parameter('scan_delay').value)
        self.max_lidar_range = float(
            self.get_parameter('max_lidar_range').value)
        minimum_angle = math.radians(float(
            self.get_parameter('min_downward_angle_deg').value))
        self.minimum_downward_fraction = math.sin(minimum_angle)
        self.enable_rgb = bool(self.get_parameter('enable_rgb').value)
        self.max_rgb_time_offset = float(
            self.get_parameter('max_rgb_time_offset').value)
        self.enable_icp = bool(
            self.get_parameter('enable_icp').value)
        self.icp_translation_only = bool(
            self.get_parameter('icp_translation_only').value)
        self.icp_reference_voxel_size = float(
            self.get_parameter('icp_reference_voxel_size').value)
        self.icp_source_voxel_size = float(
            self.get_parameter('icp_source_voxel_size').value)
        self.icp_max_correspondence_distance = float(
            self.get_parameter(
                'icp_max_correspondence_distance').value)
        self.icp_max_translation = float(
            self.get_parameter('icp_max_translation').value)
        self.icp_max_rotation_deg = float(
            self.get_parameter('icp_max_rotation_deg').value)
        self.icp_min_height = float(
            self.get_parameter('icp_min_height').value)
        self.icp_max_height = float(
            self.get_parameter('icp_max_height').value)
        self.icp_max_source_points = int(
            self.get_parameter('icp_max_source_points').value)
        self.icp_min_correspondences = int(
            self.get_parameter('icp_min_correspondences').value)
        self.icp_rebuild_interval = int(
            self.get_parameter('icp_rebuild_interval').value)
        self.enable_statistical_filter = bool(
            self.get_parameter('enable_statistical_filter').value)
        self.outlier_mean_k = int(
            self.get_parameter('outlier_mean_k').value)
        self.outlier_stddev_multiplier = float(
            self.get_parameter('outlier_stddev_multiplier').value)
        self.origin = np.array([
            float(self.get_parameter('origin_x').value),
            float(self.get_parameter('origin_y').value),
            float(self.get_parameter('origin_z').value),
        ])
        self.position_ned = None
        self.rotation_body_to_ned = None
        self.position_samples = deque(maxlen=400)
        self.attitude_samples = deque(maxlen=400)
        self.pending_scans = deque(maxlen=20)
        self.rgb_images = deque(maxlen=12)
        self.camera_intrinsics = None
        self.mavlink_clock_offset = None
        self.last_mavlink_boot_ms = None
        self.mavlink_wraps = 0
        self.mission_active = False
        self.last_mode = None
        self.pose_stream_requested = False
        self.mavlink_message_count = 0
        self.last_mavlink_type = 'none'
        self.lidar_message_count = 0
        self.registered_scan_count = 0
        self.last_lidar_stamp = None
        self.rgb_message_count = 0
        self.colored_point_count = 0
        self.total_registered_point_count = 0
        self.voxels = {}
        self.icp_reference_voxels = {}
        self.icp_reference_points = np.empty(
            (0, 3), dtype=np.float64)
        self.icp_reference_tree = None
        self.icp_updates_since_rebuild = 0
        self.icp_attempt_count = 0
        self.icp_accepted_count = 0
        self.icp_skipped_count = 0
        self.icp_last_result = None
        self.map_dirty = False
        self.path = PathMessage()
        self.path.header.frame_id = 'world'

        # The simulated LiDAR link is pitched 45 degrees forward/down relative
        # to the X500 base FLU frame.
        angle = math.radians(45.0)
        self.rotation_lidar_to_flu = np.array([
            [math.cos(angle), 0.0, math.sin(angle)],
            [0.0, 1.0, 0.0],
            [-math.sin(angle), 0.0, math.cos(angle)],
        ])
        self.rotation_flu_to_frd = np.diag([1.0, -1.0, -1.0])
        self.rotation_ned_to_enu = np.array([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ])
        self.lidar_offset_flu = np.array([0.0, 0.0, -0.10])
        self.camera_offset_flu = np.array([0.0, 0.0, -0.05])

        endpoint = str(self.get_parameter('mavlink_endpoint').value)
        self.mavlink = mavutil.mavlink_connection(
            endpoint, source_system=254)
        self.mavlink_timer = self.create_timer(0.01, self._poll_mavlink)
        self.scan_timer = self.create_timer(0.02, self._process_pending_scans)
        self.status_timer = self.create_timer(5.0, self._report_status)
        self.map_timer = self.create_timer(1.0, self._publish_map)
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            str(self.get_parameter('cloud_topic').value),
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            str(self.get_parameter('rgb_topic').value),
            self._rgb_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.scan_publisher = self.create_publisher(
            PointCloud2, '/gps_pointcloud/registered_scan',
            qos_profile_sensor_data)
        self.map_publisher = self.create_publisher(
            PointCloud2, '/gps_pointcloud/map', qos_profile_sensor_data)
        self.path_publisher = self.create_publisher(
            PathMessage, '/gps_pointcloud/path', 10)
        self.save_service = self.create_service(
            Trigger, '~/save', self._save_callback)
        icp_state = 'enabled' if self.enable_icp else 'disabled'
        icp_mode = (
            'translation only'
            if self.icp_translation_only else '6-DoF')
        self.get_logger().info(
            'Waiting for PX4 GPS/INS pose and a QGroundControl MISSION. '
            f'Bounded scan-to-map ICP is {icp_state} ({icp_mode}).')

    def _poll_mavlink(self):
        # Keep this callback short so high-rate MAVLink traffic cannot starve
        # the LiDAR subscription and scan-processing timer.
        for _ in range(50):
            msg = self.mavlink.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            self.mavlink_message_count += 1
            self.last_mavlink_type = msg_type
            if msg_type == 'LOCAL_POSITION_NED':
                self.position_ned = np.array(
                    [msg.x, msg.y, msg.z], dtype=np.float64)
                timestamp = self._mavlink_timestamp(msg)
                self.position_samples.append(
                    (timestamp, self.position_ned.copy()))
            elif msg_type == 'ATTITUDE_QUATERNION':
                quaternion = normalized_quaternion(
                    [msg.q1, msg.q2, msg.q3, msg.q4])
                self.rotation_body_to_ned = quaternion_matrix(*quaternion)
                timestamp = self._mavlink_timestamp(msg)
                self.attitude_samples.append((timestamp, quaternion))
            elif (
                msg_type == 'HEARTBEAT'
                and msg.get_srcComponent() == 1
            ):
                if not self.pose_stream_requested:
                    self._request_pose_streams(
                        msg.get_srcSystem(), msg.get_srcComponent())
                self._handle_mode(mavutil.mode_string_v10(msg))

    def _mavlink_timestamp(self, message):
        """Map wrapped MAVLink boot time onto the ROS simulation clock."""
        now = self.get_clock().now().nanoseconds * 1e-9
        boot_ms = int(message.time_boot_ms)
        if (
            self.last_mavlink_boot_ms is not None
            and boot_ms < self.last_mavlink_boot_ms - (1 << 31)
        ):
            self.mavlink_wraps += 1
        self.last_mavlink_boot_ms = boot_ms
        unwrapped = (boot_ms + self.mavlink_wraps * (1 << 32)) * 1e-3
        candidate_offset = now - unwrapped
        if (
            self.mavlink_clock_offset is None
            or abs(candidate_offset - self.mavlink_clock_offset) > 5.0
        ):
            self.mavlink_clock_offset = candidate_offset
            self.position_samples.clear()
            self.attitude_samples.clear()
        else:
            # Reception latency is positive. Keeping the smallest observed
            # offset is a simple low-jitter estimate of the capture clock.
            self.mavlink_clock_offset = min(
                self.mavlink_clock_offset, candidate_offset)
        return unwrapped + self.mavlink_clock_offset

    def _report_status(self):
        if self.mavlink_message_count == 0:
            self.get_logger().warning(
                'No PX4 MAVLink messages received yet on UDP 14540.')
            return
        pose_ready = (
            self.position_ned is not None
            and self.rotation_body_to_ned is not None)
        self.get_logger().info(
            f'PX4 MAVLink messages={self.mavlink_message_count}, '
            f'last={self.last_mavlink_type}, pose_ready={pose_ready}, '
            f'lidar={self.lidar_message_count}, '
            f'rgb={self.rgb_message_count}, '
            f'registered={self.registered_scan_count}, '
            f'voxels={len(self.voxels)}, '
            f'colored={self.colored_point_count}/'
            f'{self.total_registered_point_count}, '
            f'icp={self.icp_accepted_count}/{self.icp_attempt_count} '
            f'accepted, skipped={self.icp_skipped_count}, '
            f'icp_last={self._icp_status()}, '
            f'clock={self.get_clock().now().nanoseconds * 1e-9:.3f}, '
            f'lidar_stamp={self.last_lidar_stamp}, '
            f'position_span={self._sample_span(self.position_samples)}, '
            f'attitude_span={self._sample_span(self.attitude_samples)}.')

    @staticmethod
    def _sample_span(samples):
        if not samples:
            return 'none'
        return f'{samples[0][0]:.3f}..{samples[-1][0]:.3f}'

    def _icp_status(self):
        if self.icp_last_result is None:
            return 'none'
        result = self.icp_last_result
        return (
            f'{result.reason} '
            f'{result.rmse_before:.3f}->{result.rmse_after:.3f}m, '
            f'd={result.translation_correction:.3f}m, '
            f'r={result.rotation_correction_deg:.2f}deg')

    def _request_pose_streams(self, system_id, component_id):
        """Ask PX4 for position and quaternion at 20 Hz."""
        for message_id in (31, 32):
            self.mavlink.mav.command_long_send(
                system_id,
                component_id,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                50000,
                0, 0, 0, 0, 0,
            )
        self.pose_stream_requested = True
        self.get_logger().info(
            'Requested PX4 ATTITUDE_QUATERNION and LOCAL_POSITION_NED at '
            '20 Hz.')

    def _handle_mode(self, mode):
        if mode != self.last_mode:
            self.get_logger().info(f'PX4 flight mode: {mode}')
            self.last_mode = mode
        in_mission = mode in ('MISSION', 'AUTO.MISSION')
        if in_mission and not self.mission_active:
            self.voxels.clear()
            self.icp_reference_voxels.clear()
            self.icp_reference_points = np.empty(
                (0, 3), dtype=np.float64)
            self.icp_reference_tree = None
            self.icp_updates_since_rebuild = 0
            self.icp_attempt_count = 0
            self.icp_accepted_count = 0
            self.icp_skipped_count = 0
            self.icp_last_result = None
            self.map_dirty = False
            self.path.poses.clear()
            self.pending_scans.clear()
            self.mission_active = True
            self.get_logger().info(
                'QGC mission started: GPS/INS 3D mapping enabled.')
        elif not in_mission and self.mission_active:
            self.mission_active = False
            success, message = self._save_map()
            logger = (
                self.get_logger().info if success
                else self.get_logger().warning)
            logger(message)

    def _cloud_callback(self, msg):
        self.lidar_message_count += 1
        self.last_lidar_stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9)
        if not self.mission_active:
            return
        self.pending_scans.append(msg)

    def _rgb_callback(self, msg):
        try:
            image = decode_rgb_image(msg)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self.rgb_message_count += 1
        self.rgb_images.append((message_stamp_seconds(msg), image))

    def _camera_info_callback(self, msg):
        self.camera_intrinsics = (
            float(msg.k[0]), float(msg.k[4]),
            float(msg.k[2]), float(msg.k[5]))

    def _rgb_at(self, timestamp):
        if not self.rgb_images or self.camera_intrinsics is None:
            return None
        sample = min(
            self.rgb_images, key=lambda item: abs(item[0] - timestamp))
        if abs(sample[0] - timestamp) > self.max_rgb_time_offset:
            return None
        return sample[1]

    def _process_pending_scans(self):
        if not self.mission_active:
            self.pending_scans.clear()
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.pending_scans:
            msg = self.pending_scans[0]
            scan_time = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1e-9)
            age = now - scan_time
            if age < self.scan_delay:
                return
            position_ned = interpolate_vector(
                self.position_samples, scan_time)
            attitude = interpolate_quaternion(
                self.attitude_samples, scan_time)
            rgb_image = self._rgb_at(scan_time) if self.enable_rgb else None
            synchronized = (
                position_ned is not None
                and attitude is not None
                and (not self.enable_rgb or rgb_image is not None)
            )
            if not synchronized:
                if age < 1.0:
                    return
                self.pending_scans.popleft()
                self.get_logger().warning(
                    'Dropped a LiDAR scan: synchronized pose/RGB unavailable.')
                continue
            self.pending_scans.popleft()
            self._register_scan(
                msg, position_ned, attitude, rgb_image)

    def _register_scan(self, msg, position_ned, attitude, rgb_image):
        source = point_cloud2.read_points(
            msg, field_names=['x', 'y', 'z'], skip_nans=False)
        points_lidar = np.column_stack(
            (source['x'], source['y'], source['z'])).astype(np.float64)
        valid = np.all(np.isfinite(points_lidar), axis=1)
        points_lidar = points_lidar[valid]
        if points_lidar.size == 0:
            return

        ranges = np.linalg.norm(points_lidar, axis=1)
        points_flu = points_lidar @ self.rotation_lidar_to_flu.T
        downward_fraction = np.divide(
            -points_flu[:, 2],
            ranges,
            out=np.zeros_like(ranges),
            where=ranges > 0.0,
        )
        valid = (
            (ranges >= 0.5)
            & (ranges <= self.max_lidar_range)
            & (downward_fraction >= self.minimum_downward_fraction)
        )
        points_lidar = points_lidar[valid]
        if points_lidar.size == 0:
            return
        if rgb_image is not None:
            colors, colored = project_rgb_to_lidar(
                points_lidar,
                rgb_image,
                self.camera_intrinsics,
                self.rotation_lidar_to_flu,
                self.lidar_offset_flu,
                self.camera_offset_flu,
            )
        else:
            colors = np.full(
                (len(points_lidar), 3), 128, dtype=np.uint8)
            colored = np.zeros(len(points_lidar), dtype=bool)
        self.registered_scan_count += 1
        self.colored_point_count += int(np.count_nonzero(colored))
        self.total_registered_point_count += len(points_lidar)

        rotation_body_to_ned = quaternion_matrix(*attitude)
        rotation_flu_to_enu = (
            self.rotation_ned_to_enu
            @ rotation_body_to_ned
            @ self.rotation_flu_to_frd
        )
        rotation_lidar_to_enu = (
            rotation_flu_to_enu
            @ self.rotation_lidar_to_flu
        )
        position_enu = (
            self.rotation_ned_to_enu @ position_ned
            + self.origin
            + rotation_flu_to_enu @ self.lidar_offset_flu)
        points_world = (
            points_lidar @ rotation_lidar_to_enu.T + position_enu)
        points_world = self._align_scan_with_icp(points_world)
        self._update_icp_reference(points_world)

        self._publish_cloud(
            points_world, colors, msg.header.stamp,
            self.scan_publisher)
        keys = np.floor(points_world / self.voxel_size).astype(np.int64)
        unique_keys, inverse = np.unique(
            keys, axis=0, return_inverse=True)
        sums = np.zeros((len(unique_keys), 3), dtype=np.float64)
        np.add.at(sums, inverse, points_world)
        color_sums = np.zeros((len(unique_keys), 3), dtype=np.float64)
        np.add.at(color_sums, inverse, colors)
        counts = np.bincount(inverse)
        for key_array, point_sum, color_sum, count in zip(
            unique_keys, sums, color_sums, counts
        ):
            key = tuple(key_array)
            if key in self.voxels:
                self.voxels[key][:3] += point_sum
                self.voxels[key][3:6] += color_sum
                self.voxels[key][6] += count
            elif len(self.voxels) < self.max_points:
                self.voxels[key] = np.array([
                    point_sum[0], point_sum[1], point_sum[2],
                    color_sum[0], color_sum[1], color_sum[2], count],
                    dtype=np.float64)
        if len(unique_keys):
            self.map_dirty = True
        self._publish_path(position_enu, msg.header.stamp)

    def _align_scan_with_icp(self, points_world):
        if not self.enable_icp:
            return points_world
        structural = points_world[
            (points_world[:, 2] >= self.icp_min_height)
            & (points_world[:, 2] <= self.icp_max_height)
        ]
        source = voxel_centroids(
            structural, self.icp_source_voxel_size)
        if len(source) > self.icp_max_source_points:
            indices = np.linspace(
                0,
                len(source) - 1,
                self.icp_max_source_points,
                dtype=np.int64,
            )
            source = source[indices]
        self._maybe_rebuild_icp_reference()
        if (
            len(source) < self.icp_min_correspondences
            or self.icp_reference_tree is None
        ):
            self.icp_skipped_count += 1
            return points_world
        self.icp_attempt_count += 1
        result = gps_seeded_icp(
            source,
            self.icp_reference_points,
            target_tree=self.icp_reference_tree,
            maximum_correspondence_distance=(
                self.icp_max_correspondence_distance),
            minimum_correspondences=self.icp_min_correspondences,
            maximum_translation=self.icp_max_translation,
            maximum_rotation_degrees=self.icp_max_rotation_deg,
            translation_only=self.icp_translation_only,
        )
        self.icp_last_result = result
        if not result.accepted:
            return points_world
        self.icp_accepted_count += 1
        if self.icp_accepted_count == 1:
            self.get_logger().info(
                'GPS-seeded ICP is active: first correction accepted '
                f'({self._icp_status()}).')
        return result.transform(points_world)

    def _update_icp_reference(self, points_world):
        if not self.enable_icp:
            return
        structural = points_world[
            (points_world[:, 2] >= self.icp_min_height)
            & (points_world[:, 2] <= self.icp_max_height)
        ]
        if len(structural) == 0:
            return
        keys = np.floor(
            structural / self.icp_reference_voxel_size).astype(
                np.int64)
        unique_keys, inverse = np.unique(
            keys, axis=0, return_inverse=True)
        sums = np.zeros((len(unique_keys), 3), dtype=np.float64)
        np.add.at(sums, inverse, structural)
        counts = np.bincount(inverse)
        for key_array, point_sum, count in zip(
            unique_keys, sums, counts
        ):
            key = tuple(key_array)
            if key in self.icp_reference_voxels:
                self.icp_reference_voxels[key][:3] += point_sum
                self.icp_reference_voxels[key][3] += count
            elif len(self.icp_reference_voxels) < self.max_points:
                self.icp_reference_voxels[key] = np.array([
                    point_sum[0], point_sum[1], point_sum[2], count],
                    dtype=np.float64,
                )
        if len(unique_keys):
            self.icp_updates_since_rebuild += 1

    def _maybe_rebuild_icp_reference(self):
        enough_points = (
            len(self.icp_reference_voxels)
            >= self.icp_min_correspondences)
        rebuild_due = (
            self.icp_reference_tree is None
            or self.icp_updates_since_rebuild
            >= self.icp_rebuild_interval)
        if not enough_points or not rebuild_due:
            return
        values = np.asarray(
            list(self.icp_reference_voxels.values()),
            dtype=np.float64,
        )
        self.icp_reference_points = (
            values[:, :3] / values[:, 3, np.newaxis])
        self.icp_reference_tree = cKDTree(
            self.icp_reference_points)
        self.icp_updates_since_rebuild = 0

    def _publish_path(self, position, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.w = 1.0
        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

    def _publish_map(self):
        if not self.mission_active or not self.voxels:
            return
        points, colors = self._voxel_points_and_colors()
        self._publish_cloud(
            points,
            colors,
            self.get_clock().now().to_msg(),
            self.map_publisher,
        )

    def _voxel_points_and_colors(self):
        values = np.asarray(list(self.voxels.values()), dtype=np.float64)
        counts = values[:, 6, np.newaxis]
        points = values[:, :3] / counts
        colors = np.clip(
            np.rint(values[:, 3:6] / counts), 0, 255).astype(np.uint8)
        return points, colors

    @staticmethod
    def _publish_cloud(points, colors, stamp, publisher):
        from std_msgs.msg import Header
        header = Header(stamp=stamp, frame_id='world')
        fields = [
            PointField(
                name='x', offset=0,
                datatype=PointField.FLOAT32, count=1),
            PointField(
                name='y', offset=4,
                datatype=PointField.FLOAT32, count=1),
            PointField(
                name='z', offset=8,
                datatype=PointField.FLOAT32, count=1),
            PointField(
                name='rgb', offset=12,
                datatype=PointField.FLOAT32, count=1),
        ]
        values = np.empty(
            len(points),
            dtype=point_cloud2.dtype_from_fields(fields))
        values['x'] = points[:, 0]
        values['y'] = points[:, 1]
        values['z'] = points[:, 2]
        packed_rgb = (
            colors[:, 0].astype(np.uint32) << 16
            | colors[:, 1].astype(np.uint32) << 8
            | colors[:, 2].astype(np.uint32))
        values['rgb'] = packed_rgb.view(np.float32)
        publisher.publish(point_cloud2.create_cloud(header, fields, values))

    def _save_callback(self, _request, response):
        response.success, response.message = self._save_map()
        return response

    def _save_map(self):
        if not self.voxels:
            return False, 'No GPS/INS mission map points are available.'
        self.output_directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path = self.output_directory / f'uav_gps_map_{stamp}.pcd'
        points, colors = self._voxel_points_and_colors()
        icp_mode = (
            'translation-only ICP'
            if self.icp_translation_only else 'bounded 6-DoF ICP')
        registration_comment = (
            f'PX4 GPS/INS + {icp_mode} '
            f'({self.icp_accepted_count}/{self.icp_attempt_count} accepted)')
        write_colored_ascii_pcd(
            path, points, colors, registration_comment)
        navigation_points = points
        filtered_message = ''
        if self.enable_statistical_filter:
            try:
                keep = statistical_outlier_mask(
                    points,
                    neighbor_count=self.outlier_mean_k,
                    stddev_multiplier=self.outlier_stddev_multiplier,
                )
                filtered_points = points[keep]
                filtered_colors = colors[keep]
                if len(filtered_points) == 0:
                    raise ValueError(
                        'statistical filter rejected every point')
                filtered_path = (
                    self.output_directory
                    / f'uav_gps_map_{stamp}_filtered.pcd')
                write_colored_ascii_pcd(
                    filtered_path,
                    filtered_points,
                    filtered_colors,
                    registration_comment
                    + f'; statistical outlier filter k={self.outlier_mean_k}, '
                    f'stddev={self.outlier_stddev_multiplier:g}',
                )
                navigation_points = filtered_points
                filtered_message = (
                    f'; filtered map has {len(filtered_points)} points: '
                    f'{filtered_path}')
            except (MemoryError, OSError, RuntimeError, ValueError) as error:
                self.get_logger().warning(
                    f'Could not generate filtered PCD: {error}')
        mask_message = ''
        try:
            mask_pcd, mask_ppm, reachable = save_traversability_products(
                navigation_points,
                self.output_directory / 'latest_panther_traversability',
                resolution=0.10,
            )
            mask_message = (
                f'; Panther mask has {reachable} reachable cells: '
                f'{mask_pcd} and {mask_ppm}')
        except (OSError, ValueError) as error:
            self.get_logger().warning(
                f'Could not generate Panther traversability mask: {error}')
        self.map_dirty = False
        return (
            True,
            f'Saved {len(points)} GPS/INS+ICP map points to {path}; '
            f'ICP accepted {self.icp_accepted_count}/'
            f'{self.icp_attempt_count} attempted corrections'
            f'{filtered_message}'
            f'{mask_message}')


def main(args=None):
    rclpy.init(args=args)
    node = GpsPointCloudMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.map_dirty:
            success, message = node._save_map()
            logger = (
                node.get_logger().info if success
                else node.get_logger().warning)
            logger(f'Shutdown save: {message}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
