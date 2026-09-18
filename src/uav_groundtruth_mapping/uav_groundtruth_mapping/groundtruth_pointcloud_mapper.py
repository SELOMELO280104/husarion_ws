"""Accumulate RGB LiDAR scans with the exact Gazebo model pose."""

from collections import deque
from datetime import datetime
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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from uav_gps_mapping.gps_pointcloud_mapper import (
    decode_rgb_image,
    interpolate_quaternion,
    interpolate_vector,
    message_stamp_seconds,
    normalized_quaternion,
    project_rgb_to_lidar,
    quaternion_matrix,
    statistical_outlier_mask,
    write_colored_ascii_pcd,
)
from uav_gps_mapping.pcd_to_occupancy import (
    save_traversability_products,
)
from uav_groundtruth_mapping.row_corridor_mask import (
    save_row_corridor_products,
)


def transform_lidar_points(
    points_lidar,
    model_position,
    model_quaternion,
    rotation_lidar_to_model,
    lidar_offset_model,
):
    """Transform sensor-frame points with an exact Gazebo model pose."""
    rotation_model_to_world = quaternion_matrix(*model_quaternion)
    rotation_lidar_to_world = (
        rotation_model_to_world @ rotation_lidar_to_model)
    lidar_position_world = (
        model_position
        + rotation_model_to_world @ lidar_offset_model)
    return (
        points_lidar @ rotation_lidar_to_world.T
        + lidar_position_world
    ), lidar_position_world


class GroundTruthPointCloudMapper(Node):
    """Register simulated RGB LiDAR scans using exact Gazebo poses."""

    def __init__(self):
        super().__init__('groundtruth_pointcloud_mapper')
        self.declare_parameter(
            'cloud_topic', '/uav/groundtruth/lidar/points')
        self.declare_parameter(
            'pose_topic', '/uav/groundtruth/pose')
        self.declare_parameter(
            'rgb_topic', '/uav/groundtruth/rgb/image')
        self.declare_parameter(
            'camera_info_topic', '/uav/groundtruth/rgb/camera_info')
        self.declare_parameter('output_directory', 'maps')
        self.declare_parameter('mavlink_endpoint', 'udpin:127.0.0.1:14540')
        self.declare_parameter('voxel_size', 0.10)
        self.declare_parameter('max_points', 3000000)
        self.declare_parameter('scan_delay', 0.10)
        self.declare_parameter('max_lidar_range', 18.0)
        self.declare_parameter('min_downward_angle_deg', 35.0)
        self.declare_parameter('enable_rgb', True)
        self.declare_parameter('max_rgb_time_offset', 0.06)
        self.declare_parameter('enable_statistical_filter', True)
        self.declare_parameter('outlier_mean_k', 12)
        self.declare_parameter('outlier_stddev_multiplier', 1.5)

        self.output_directory = Path(
            str(self.get_parameter('output_directory').value)).expanduser()
        self.voxel_size = float(
            self.get_parameter('voxel_size').value)
        self.max_points = int(
            self.get_parameter('max_points').value)
        self.scan_delay = float(
            self.get_parameter('scan_delay').value)
        self.max_lidar_range = float(
            self.get_parameter('max_lidar_range').value)
        minimum_angle = math.radians(float(
            self.get_parameter('min_downward_angle_deg').value))
        self.minimum_downward_fraction = math.sin(minimum_angle)
        self.enable_rgb = bool(
            self.get_parameter('enable_rgb').value)
        self.max_rgb_time_offset = float(
            self.get_parameter('max_rgb_time_offset').value)
        self.enable_statistical_filter = bool(
            self.get_parameter('enable_statistical_filter').value)
        self.outlier_mean_k = int(
            self.get_parameter('outlier_mean_k').value)
        self.outlier_stddev_multiplier = float(
            self.get_parameter('outlier_stddev_multiplier').value)

        self.pose_positions = deque(maxlen=500)
        self.pose_attitudes = deque(maxlen=500)
        self.rgb_images = deque(maxlen=16)
        self.pending_scans = deque(maxlen=30)
        self.camera_intrinsics = None
        self.mission_active = False
        self.last_mode = None
        self.map_dirty = False
        self.voxels = {}
        self.path = PathMessage()
        self.path.header.frame_id = 'world'
        self.mavlink_message_count = 0
        self.last_mavlink_type = 'none'
        self.pose_message_count = 0
        self.lidar_message_count = 0
        self.rgb_message_count = 0
        self.registered_scan_count = 0
        self.colored_point_count = 0
        self.total_registered_point_count = 0
        self.last_lidar_stamp = None

        # The mapping LiDAR and RGB camera are fixed 45 degrees forward/down
        # in the Gazebo model's FLU frame.
        angle = math.radians(45.0)
        self.rotation_lidar_to_model = np.array([
            [math.cos(angle), 0.0, math.sin(angle)],
            [0.0, 1.0, 0.0],
            [-math.sin(angle), 0.0, math.cos(angle)],
        ])
        self.lidar_offset_model = np.array([0.0, 0.0, -0.10])
        self.camera_offset_model = np.array([0.0, 0.0, -0.05])

        self.mavlink = mavutil.mavlink_connection(
            str(self.get_parameter('mavlink_endpoint').value),
            source_system=254,
        )
        self.mavlink_timer = self.create_timer(0.02, self._poll_mavlink)
        self.scan_timer = self.create_timer(
            0.02, self._process_pending_scans)
        self.status_timer = self.create_timer(5.0, self._report_status)
        self.map_timer = self.create_timer(1.0, self._publish_map)
        self.pose_subscription = self.create_subscription(
            TFMessage,
            str(self.get_parameter('pose_topic').value),
            self._pose_callback,
            qos_profile_sensor_data,
        )
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
            PointCloud2,
            '/groundtruth_pointcloud/registered_scan',
            qos_profile_sensor_data,
        )
        self.map_publisher = self.create_publisher(
            PointCloud2,
            '/groundtruth_pointcloud/map',
            qos_profile_sensor_data,
        )
        self.path_publisher = self.create_publisher(
            PathMessage, '/groundtruth_pointcloud/path', 10)
        self.save_service = self.create_service(
            Trigger, '~/save', self._save_callback)
        self.get_logger().info(
            'Waiting for Gazebo ground-truth pose and a QGroundControl '
            'MISSION. PX4 still controls the flight.')

    def _poll_mavlink(self):
        for _ in range(50):
            message = self.mavlink.recv_match(blocking=False)
            if message is None:
                break
            self.mavlink_message_count += 1
            self.last_mavlink_type = message.get_type()
            if (
                self.last_mavlink_type == 'HEARTBEAT'
                and message.get_srcComponent() == 1
            ):
                self._handle_mode(mavutil.mode_string_v10(message))

    def _handle_mode(self, mode):
        if mode != self.last_mode:
            self.get_logger().info(f'PX4 flight mode: {mode}')
            self.last_mode = mode
        in_mission = mode in ('MISSION', 'AUTO.MISSION')
        if in_mission and not self.mission_active:
            self.voxels.clear()
            self.path.poses.clear()
            self.pending_scans.clear()
            self.registered_scan_count = 0
            self.colored_point_count = 0
            self.total_registered_point_count = 0
            self.map_dirty = False
            self.mission_active = True
            self.get_logger().info(
                'QGC mission started: Gazebo ground-truth mapping enabled.')
        elif not in_mission and self.mission_active:
            self.mission_active = False
            success, message = self._save_map()
            logger = (
                self.get_logger().info if success
                else self.get_logger().warning)
            logger(message)

    def _pose_callback(self, message):
        if not message.transforms:
            return
        transform = message.transforms[0]
        timestamp = (
            float(transform.header.stamp.sec)
            + float(transform.header.stamp.nanosec) * 1e-9)
        if timestamp <= 0.0:
            timestamp = self.get_clock().now().nanoseconds * 1e-9
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        position = np.array([
            translation.x, translation.y, translation.z,
        ], dtype=np.float64)
        quaternion = normalized_quaternion([
            rotation.w, rotation.x, rotation.y, rotation.z,
        ])
        self.pose_positions.append((timestamp, position))
        self.pose_attitudes.append((timestamp, quaternion))
        self.pose_message_count += 1

    def _cloud_callback(self, message):
        self.lidar_message_count += 1
        self.last_lidar_stamp = message_stamp_seconds(message)
        if self.mission_active:
            self.pending_scans.append(message)

    def _rgb_callback(self, message):
        try:
            image = decode_rgb_image(message)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self.rgb_images.append((message_stamp_seconds(message), image))
        self.rgb_message_count += 1

    def _camera_info_callback(self, message):
        self.camera_intrinsics = (
            float(message.k[0]),
            float(message.k[4]),
            float(message.k[2]),
            float(message.k[5]),
        )

    def _rgb_at(self, timestamp):
        if not self.rgb_images or self.camera_intrinsics is None:
            return None
        sample = min(
            self.rgb_images,
            key=lambda item: abs(item[0] - timestamp),
        )
        if abs(sample[0] - timestamp) > self.max_rgb_time_offset:
            return None
        return sample[1]

    def _process_pending_scans(self):
        if not self.mission_active:
            self.pending_scans.clear()
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.pending_scans:
            message = self.pending_scans[0]
            scan_time = message_stamp_seconds(message)
            age = now - scan_time
            if age < self.scan_delay:
                return
            position = interpolate_vector(
                self.pose_positions, scan_time)
            attitude = interpolate_quaternion(
                self.pose_attitudes, scan_time)
            if position is None or attitude is None:
                if age < 1.0:
                    return
                self.pending_scans.popleft()
                self.get_logger().warning(
                    'Dropped a LiDAR scan: ground-truth pose unavailable.')
                continue
            rgb_image = (
                self._rgb_at(scan_time) if self.enable_rgb else None)
            self.pending_scans.popleft()
            self._register_scan(
                message, position, attitude, rgb_image)

    def _register_scan(self, message, position, attitude, rgb_image):
        source = point_cloud2.read_points(
            message,
            field_names=['x', 'y', 'z'],
            skip_nans=False,
        )
        points_lidar = np.column_stack(
            (source['x'], source['y'], source['z'])).astype(np.float64)
        finite = np.all(np.isfinite(points_lidar), axis=1)
        points_lidar = points_lidar[finite]
        if points_lidar.size == 0:
            return

        ranges = np.linalg.norm(points_lidar, axis=1)
        points_model = (
            points_lidar @ self.rotation_lidar_to_model.T)
        downward_fraction = np.divide(
            -points_model[:, 2],
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
                self.rotation_lidar_to_model,
                self.lidar_offset_model,
                self.camera_offset_model,
            )
        else:
            colors = np.full(
                (len(points_lidar), 3), 128, dtype=np.uint8)
            colored = np.zeros(len(points_lidar), dtype=bool)

        points_world, lidar_position = transform_lidar_points(
            points_lidar,
            position,
            attitude,
            self.rotation_lidar_to_model,
            self.lidar_offset_model,
        )
        self.registered_scan_count += 1
        self.colored_point_count += int(np.count_nonzero(colored))
        self.total_registered_point_count += len(points_lidar)
        self._publish_cloud(
            points_world,
            colors,
            message.header.stamp,
            self.scan_publisher,
        )
        self._accumulate(points_world, colors)
        self._publish_path(lidar_position, message.header.stamp)

    def _accumulate(self, points, colors):
        keys = np.floor(points / self.voxel_size).astype(np.int64)
        unique_keys, inverse = np.unique(
            keys, axis=0, return_inverse=True)
        sums = np.zeros((len(unique_keys), 3), dtype=np.float64)
        np.add.at(sums, inverse, points)
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
                    color_sum[0], color_sum[1], color_sum[2], count,
                ], dtype=np.float64)
        if len(unique_keys):
            self.map_dirty = True

    def _voxel_points_and_colors(self):
        values = np.asarray(
            list(self.voxels.values()), dtype=np.float64)
        counts = values[:, 6, np.newaxis]
        points = values[:, :3] / counts
        colors = np.clip(
            np.rint(values[:, 3:6] / counts),
            0,
            255,
        ).astype(np.uint8)
        return points, colors

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
            dtype=point_cloud2.dtype_from_fields(fields),
        )
        values['x'] = points[:, 0]
        values['y'] = points[:, 1]
        values['z'] = points[:, 2]
        packed_rgb = (
            colors[:, 0].astype(np.uint32) << 16
            | colors[:, 1].astype(np.uint32) << 8
            | colors[:, 2].astype(np.uint32))
        values['rgb'] = packed_rgb.view(np.float32)
        publisher.publish(
            point_cloud2.create_cloud(header, fields, values))

    def _report_status(self):
        pose_ready = (
            bool(self.pose_positions)
            and bool(self.pose_attitudes))
        colored_ratio = (
            100.0 * self.colored_point_count
            / self.total_registered_point_count
            if self.total_registered_point_count else 0.0)
        self.get_logger().info(
            f'ground_truth_pose={self.pose_message_count}, '
            f'pose_ready={pose_ready}, '
            f'PX4_messages={self.mavlink_message_count}, '
            f'last={self.last_mavlink_type}, '
            f'lidar={self.lidar_message_count}, '
            f'rgb={self.rgb_message_count}, '
            f'registered={self.registered_scan_count}, '
            f'voxels={len(self.voxels)}, '
            f'colored={colored_ratio:.1f}%, '
            f'lidar_stamp={self.last_lidar_stamp}.')

    def _save_callback(self, _request, response):
        response.success, response.message = self._save_map()
        return response

    def _save_map(self):
        if not self.voxels:
            return False, 'No ground-truth mission map points are available.'
        self.output_directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        raw_path = (
            self.output_directory
            / f'uav_groundtruth_map_{stamp}.pcd')
        points, colors = self._voxel_points_and_colors()
        write_colored_ascii_pcd(
            raw_path,
            points,
            colors,
            'Gazebo exact model pose + synchronized RGB LiDAR',
        )

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
                    / f'uav_groundtruth_map_{stamp}_filtered.pcd')
                write_colored_ascii_pcd(
                    filtered_path,
                    filtered_points,
                    filtered_colors,
                    'Gazebo exact model pose + synchronized RGB LiDAR; '
                    f'statistical outlier filter k={self.outlier_mean_k}, '
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

        corridor_message = ''
        try:
            corridor_products = save_row_corridor_products(
                navigation_points,
                self.output_directory
                / 'latest_panther_row_corridors',
                resolution=0.10,
                source_name=str(raw_path),
            )
            corridor_message = (
                '; row mask detected '
                f'{corridor_products["row_count"]} vegetation rows, '
                f'{corridor_products["accessible_corridor_count"]} '
                'accessible corridors and '
                f'{corridor_products["reachable_cells"]} reachable cells: '
                f'{corridor_products["nav_yaml"]}')
        except (MemoryError, OSError, RuntimeError, ValueError) as error:
            self.get_logger().warning(
                f'Could not generate Panther row-corridor mask: {error}')
        self.map_dirty = False
        return (
            True,
            f'Saved {len(points)} Gazebo ground-truth points to {raw_path}'
            f'{filtered_message}{mask_message}{corridor_message}',
        )


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPointCloudMapper()
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
