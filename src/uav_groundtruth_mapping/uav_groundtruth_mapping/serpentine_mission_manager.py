"""Run a guarded, GPS-denied Panther mission through orchard row corridors."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path as PathMessage
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import label as connected_labels
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml


def load_corridors(csv_path):
    """Load and validate corridor centerlines from the row-mask CSV."""
    corridors = []
    with Path(csv_path).open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            corridor = {
                'id': int(row['corridor_id']),
                'start': (
                    float(row['start_x']),
                    float(row['start_y']),
                ),
                'end': (
                    float(row['end_x']),
                    float(row['end_y']),
                ),
                'width': float(row['nominal_width_m']),
            }
            if corridor['width'] <= 0.0:
                raise ValueError(
                    f'Corridor {corridor["id"]} has an invalid width')
            if math.dist(corridor['start'], corridor['end']) < 0.5:
                raise ValueError(
                    f'Corridor {corridor["id"]} is too short')
            corridors.append(corridor)
    if not corridors:
        raise ValueError(f'No corridors found in {csv_path}')
    corridors.sort(key=lambda item: item['id'])
    return corridors


def sample_line(start, end, spacing):
    """Return evenly spaced XY points, excluding start and including end."""
    length = math.dist(start, end)
    count = max(1, int(math.ceil(length / spacing)))
    return [
        (
            start[0] + (end[0] - start[0]) * index / count,
            start[1] + (end[1] - start[1]) * index / count,
        )
        for index in range(1, count + 1)
    ]


def build_serpentine_segments(
    corridors,
    robot_xy,
    waypoint_spacing=1.0,
    headland_clearance=2.25,
):
    """Build approach, row, and headland segments for full field coverage."""
    if waypoint_spacing <= 0.0:
        raise ValueError('waypoint_spacing must be positive')
    if headland_clearance <= 0.0:
        raise ValueError('headland_clearance must be positive')

    # Enter from the nearer outer row. This avoids crossing unharvested rows
    # merely to reach the beginning of the boustrophedon pattern.
    low_distance = min(
        math.dist(robot_xy, corridors[0]['start']),
        math.dist(robot_xy, corridors[0]['end']),
    )
    high_distance = min(
        math.dist(robot_xy, corridors[-1]['start']),
        math.dist(robot_xy, corridors[-1]['end']),
    )
    ordered = corridors if low_distance <= high_distance else list(
        reversed(corridors))

    first = ordered[0]
    first_forward = (
        math.dist(robot_xy, first['start'])
        <= math.dist(robot_xy, first['end'])
    )
    segments = []
    previous_exit = None

    for index, corridor in enumerate(ordered):
        forward = first_forward if index % 2 == 0 else not first_forward
        entry = corridor['start'] if forward else corridor['end']
        exit_point = corridor['end'] if forward else corridor['start']

        if index == 0:
            segments.append({
                'name': f'approach_row_{corridor["id"]}',
                'kind': 'approach',
                'points': [entry],
            })
        else:
            # Do not cut diagonally around the vegetation at a row end. Move
            # fully into the headland, shift to the next row there, and then
            # re-enter. This three-leg manoeuvre leaves turning room for the
            # Panther's footprint and for the local LiDAR safety inflation.
            all_x = [
                point[0]
                for item in ordered
                for point in (item['start'], item['end'])
            ]
            field_mid_x = 0.5 * (min(all_x) + max(all_x))
            right_headland = (
                0.5 * (previous_exit[0] + entry[0]) > field_mid_x
            )
            outside_x = (
                max(previous_exit[0], entry[0]) + headland_clearance
                if right_headland
                else min(previous_exit[0], entry[0]) - headland_clearance
            )
            outer_exit = (outside_x, previous_exit[1])
            outer_entry = (outside_x, entry[1])
            previous_id = ordered[index - 1]['id']
            next_id = corridor['id']
            # Keep each straight part of the U-turn in its own Nav2 action.
            # A single NavigateThroughPoses action spanning both 90-degree
            # corners can briefly hand the controller an empty pruned path
            # while changing legs (FollowPath INVALID_PATH=103). Reaching and
            # validating each leg separately also prevents a recovery from
            # cutting diagonally across the vegetation.
            headland_legs = (
                (
                    'exit',
                    previous_exit,
                    outer_exit,
                ),
                (
                    'shift',
                    outer_exit,
                    outer_entry,
                ),
                (
                    'enter',
                    outer_entry,
                    entry,
                ),
            )
            for leg_name, leg_start, leg_end in headland_legs:
                segments.append({
                    'name': (
                        f'headland_{leg_name}_row_{previous_id}'
                        f'_to_{next_id}'
                    ),
                    'kind': f'headland_{leg_name}',
                    'points': sample_line(
                        leg_start,
                        leg_end,
                        max(0.5, waypoint_spacing),
                    ),
                })

        segments.append({
            'name': f'harvest_row_{corridor["id"]}',
            'kind': 'row',
            'points': sample_line(entry, exit_point, waypoint_spacing),
        })
        previous_exit = exit_point

    return segments


def _read_binary_pgm(path):
    """Read the P5 PGM format emitted by the mapping package."""
    with Path(path).open('rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError(f'{path} is not a binary P5 PGM')
        tokens = []
        while len(tokens) < 3:
            line = stream.readline()
            if not line:
                raise ValueError(f'{path} has an incomplete PGM header')
            line = line.split(b'#', 1)[0]
            tokens.extend(line.split())
        width, height, maximum = (int(token) for token in tokens[:3])
        if maximum != 255:
            raise ValueError(
                f'{path} uses unsupported PGM maximum {maximum}')
        data = np.frombuffer(stream.read(), dtype=np.uint8)
    if data.size != width * height:
        raise ValueError(
            f'{path} contains {data.size} pixels, expected '
            f'{width * height}')
    return data.reshape((height, width))


def load_traversability_map(yaml_path):
    """Load Nav2 map geometry and build a nearest-free-cell index."""
    yaml_path = Path(yaml_path)
    metadata = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    image_path = Path(metadata['image'])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image = _read_binary_pgm(image_path)
    free_rows, free_columns = np.nonzero(image >= 250)
    if len(free_rows) == 0:
        raise ValueError(f'{image_path} contains no free cells')
    resolution = float(metadata['resolution'])
    origin = metadata['origin']
    height = image.shape[0]
    free_xy = np.column_stack((
        float(origin[0]) + (free_columns + 0.5) * resolution,
        float(origin[1])
        + (height - 1 - free_rows + 0.5) * resolution,
    ))
    components, component_count = connected_labels(
        image >= 250,
        structure=np.ones((3, 3), dtype=np.uint8),
    )
    return {
        'image': image,
        'origin': (float(origin[0]), float(origin[1])),
        'resolution': resolution,
        'free_xy': free_xy,
        'free_tree': cKDTree(free_xy),
        'free_rows': free_rows,
        'free_columns': free_columns,
        'components': components,
        'component_count': int(component_count),
    }


def select_start_component(traversability_map, robot_xy, max_distance=1.0):
    """Restrict a map index to free space connected to the robot start."""
    distance, index = traversability_map['free_tree'].query(
        np.asarray(robot_xy, dtype=np.float64), k=1)
    if distance > max_distance:
        raise ValueError(
            f'Robot start {robot_xy} is {distance:.2f} m from '
            'known traversable space')
    row = traversability_map['free_rows'][index]
    column = traversability_map['free_columns'][index]
    component = traversability_map['components'][row, column]
    selected = (
        traversability_map['components'][
            traversability_map['free_rows'],
            traversability_map['free_columns'],
        ] == component
    )
    connected_xy = traversability_map['free_xy'][selected]
    return {
        **traversability_map,
        'free_xy': connected_xy,
        'free_tree': cKDTree(connected_xy),
        'selected_component': int(component),
    }


def snap_segments_to_free(segments, traversability_map, max_distance=0.25):
    """Validate goals; skip blocked midpoints and snap goals to free cells."""
    tree = traversability_map['free_tree']
    adjusted_count = 0
    result = []
    for segment in segments:
        points = []
        last_index = len(segment['points']) - 1
        for point_index, point in enumerate(segment['points']):
            distance, index = tree.query(
                np.asarray(point, dtype=np.float64), k=1)
            if distance > max_distance:
                # A sampled midpoint may land on a mapped trunk or row edge.
                # It is not a mandatory goal; Nav2 will route between the
                # surrounding valid points. Segment endpoints are mandatory.
                if point_index != last_index:
                    adjusted_count += 1
                    continue
                raise ValueError(
                    f'Segment endpoint {point} is {distance:.2f} m from '
                    'start-connected traversable space')
            snapped = tuple(
                float(value)
                for value in traversability_map['free_xy'][index])
            if math.dist(point, snapped) > (
                0.75 * traversability_map['resolution']
            ):
                adjusted_count += 1
            if not points or math.dist(points[-1], snapped) > 1e-6:
                points.append(snapped)
        if not points:
            raise ValueError(
                f'Snapping removed every point in {segment["name"]}')
        result.append({**segment, 'points': points})
    return result, adjusted_count


def yaw_between(start, end):
    return math.atan2(end[1] - start[1], end[0] - start[0])


def pose_from_xy(xy, yaw, frame_id, stamp):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(xy[0])
    pose.pose.position.y = float(xy[1])
    pose.pose.orientation.z = math.sin(0.5 * yaw)
    pose.pose.orientation.w = math.cos(0.5 * yaw)
    return pose


class SerpentineMissionManager(Node):
    """Send safe row-by-row NavigateThroughPoses goals to Nav2."""

    def __init__(self):
        super().__init__('panther_serpentine_mission')
        self.declare_parameter(
            'corridors_csv',
            str(
                Path.home() / 'husarion_ws' / 'maps'
                / 'latest_panther_row_corridors_centerlines.csv'
            ),
        )
        self.declare_parameter(
            'map_yaml',
            str(
                Path.home() / 'husarion_ws' / 'maps'
                / 'latest_panther_row_corridors_nav.yaml'
            ),
        )
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('waypoint_spacing', 1.0)
        self.declare_parameter('headland_clearance', 2.25)
        self.declare_parameter('waypoint_snap_distance', 0.25)
        self.declare_parameter('scan_timeout', 2.0)
        self.declare_parameter('max_segment_retries', 1)
        self.declare_parameter('autostart', False)
        self.declare_parameter('auto_resume_sensor_loss', True)
        self.declare_parameter('initialize_amcl', True)
        self.declare_parameter('initial_pose_x', -6.0)
        self.declare_parameter('initial_pose_y', -8.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('required_readiness_topic', '')
        self.declare_parameter(
            'required_readiness_label', 'coordinated vehicle')

        csv_path = self.get_parameter(
            'corridors_csv').get_parameter_value().string_value
        map_yaml = self.get_parameter(
            'map_yaml').get_parameter_value().string_value
        self.map_frame = self.get_parameter(
            'map_frame').get_parameter_value().string_value
        self.waypoint_spacing = self.get_parameter(
            'waypoint_spacing').get_parameter_value().double_value
        self.headland_clearance = self.get_parameter(
            'headland_clearance').get_parameter_value().double_value
        self.snap_distance = self.get_parameter(
            'waypoint_snap_distance').get_parameter_value().double_value
        self.scan_timeout = self.get_parameter(
            'scan_timeout').get_parameter_value().double_value
        self.max_retries = self.get_parameter(
            'max_segment_retries').get_parameter_value().integer_value
        self.autostart = self.get_parameter(
            'autostart').get_parameter_value().bool_value
        self.auto_resume_sensor_loss = self.get_parameter(
            'auto_resume_sensor_loss').get_parameter_value().bool_value
        self.initialize_amcl = self.get_parameter(
            'initialize_amcl').get_parameter_value().bool_value
        self.initial_pose_xy = (
            self.get_parameter(
                'initial_pose_x').get_parameter_value().double_value,
            self.get_parameter(
                'initial_pose_y').get_parameter_value().double_value,
        )
        self.initial_pose_yaw = self.get_parameter(
            'initial_pose_yaw').get_parameter_value().double_value
        self.required_readiness_topic = self.get_parameter(
            'required_readiness_topic').get_parameter_value().string_value
        self.required_readiness_label = self.get_parameter(
            'required_readiness_label').get_parameter_value().string_value

        self.corridors = load_corridors(csv_path)
        self.traversability_map = load_traversability_map(map_yaml)
        self.action_client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses')
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(
            String, '/panther/harvest/status', latched_qos)
        self.path_pub = self.create_publisher(
            PathMessage, '/panther/harvest/planned_path', latched_qos)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', latched_qos)
        self.stop_pub = self.create_publisher(
            TwistStamped, '/panther/autonomous/cmd_vel', 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_callback,
            # AMCL publishes its initialized pose with transient-local
            # durability.  The mission manager starts a few seconds after
            # AMCL, so it must request the cached pose or it can wait forever
            # while a stationary robot produces no new AMCL update.
            latched_qos,
        )
        self.create_subscription(
            LaserScan,
            '/panther/main_lidar/scan',
            self._scan_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/panther/hardware/e_stop',
            self._estop_callback,
            latched_qos,
        )
        self.external_ready = (
            None if self.required_readiness_topic else True)
        if self.required_readiness_topic:
            self.create_subscription(
                Bool,
                self.required_readiness_topic,
                self._external_readiness_callback,
                latched_qos,
            )
        self.create_service(
            Trigger, '/panther/harvest/start', self._start_service)
        self.create_service(
            Trigger, '/panther/harvest/pause', self._pause_service)
        self.create_service(
            Trigger, '/panther/harvest/resume', self._resume_service)
        self.create_service(
            Trigger, '/panther/harvest/cancel', self._cancel_service)

        self.current_pose = None
        self.initial_pose_publish_count = 0
        self.localization_initialized = not self.initialize_amcl
        self.last_scan_monotonic = None
        self.estop_active = None
        self.state = 'WAITING_FOR_SENSORS'
        self.detail = 'Waiting for AMCL, LiDAR, E-stop, and Nav2'
        self.segments = []
        self.segment_index = 0
        self.segment_retry = 0
        self.segment_waypoint_offset = 0
        self.active_pose_offset = 0
        self.active_pose_count = 0
        self.goal_handle = None
        self.goal_token = 0
        self.autostart_attempted = False
        self.initial_pose_timer = self.create_timer(
            0.5, self._initial_pose_timer)
        self.readiness_timer = self.create_timer(
            0.5, self._readiness_timer)
        self._publish_status()
        self.get_logger().info(
            f'Loaded {len(self.corridors)} safe row corridors from {csv_path}.')
        self.get_logger().info(
            f'Validating mission goals against Nav2 map {map_yaml}.')

    def _pose_callback(self, message):
        self.current_pose = message
        if (
            self.initialize_amcl
            and self.initial_pose_publish_count >= 3
            and math.dist(self._current_xy(), self.initial_pose_xy) <= 0.5
        ):
            self.localization_initialized = True

    def _initial_pose_timer(self):
        """Seed AMCL tightly once its Nav2 action server is available."""
        if (
            not self.initialize_amcl
            or self.initial_pose_publish_count >= 3
            or not self.action_client.server_is_ready()
        ):
            return
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = self.initial_pose_xy[0]
        message.pose.pose.position.y = self.initial_pose_xy[1]
        message.pose.pose.orientation.z = math.sin(
            0.5 * self.initial_pose_yaw)
        message.pose.pose.orientation.w = math.cos(
            0.5 * self.initial_pose_yaw)
        # 5 cm planar standard deviation and 5 degree yaw standard
        # deviation. The simulator spawn is known, while subsequent motion
        # remains GPS-denied wheel odometry plus LiDAR localization.
        message.pose.covariance[0] = 0.0025
        message.pose.covariance[7] = 0.0025
        message.pose.covariance[35] = math.radians(5.0) ** 2
        self.current_pose = None
        self.localization_initialized = False
        self.initial_pose_publish_count += 1
        self.initial_pose_pub.publish(message)
        if self.initial_pose_publish_count == 3:
            self.get_logger().info(
                'Sent tight GPS-denied AMCL initialization at '
                f'({self.initial_pose_xy[0]:.2f}, '
                f'{self.initial_pose_xy[1]:.2f}).')

    def _scan_callback(self, message):
        if message.ranges:
            self.last_scan_monotonic = time.monotonic()

    def _estop_callback(self, message):
        was_active = self.estop_active
        self.estop_active = bool(message.data)
        if self.estop_active and not was_active and self.state == 'RUNNING':
            self._invalidate_and_cancel_goal()
            self._publish_zero()
            self.state = 'PAUSED_ESTOP'
            self.detail = (
                'E-stop activated; reset E-stop, then call harvest/resume')
            self._publish_status()
            self.get_logger().error(self.detail)

    def _external_readiness_callback(self, message):
        was_ready = self.external_ready
        self.external_ready = bool(message.data)
        if (
            was_ready
            and not self.external_ready
            and self.state == 'RUNNING'
        ):
            self._invalidate_and_cancel_goal()
            self._publish_zero()
            self.state = 'PAUSED_SENSOR_LOSS'
            self.detail = (
                f'Navigation paused: waiting for '
                f'{self.required_readiness_label}')
            self._publish_status()
            self.get_logger().error(self.detail)

    def _ready_reason(self):
        missing = []
        if not self.localization_initialized:
            missing.append('tight AMCL initialization')
        if self.current_pose is None:
            missing.append('AMCL pose')
        if (
            self.last_scan_monotonic is None
            or time.monotonic() - self.last_scan_monotonic > self.scan_timeout
        ):
            missing.append('fresh LiDAR scan')
        if self.estop_active is None:
            missing.append('E-stop state')
        elif self.estop_active:
            missing.append('E-stop reset')
        if not self.action_client.server_is_ready():
            missing.append('Nav2 action server')
        if not self.external_ready:
            missing.append(self.required_readiness_label)
        return ', '.join(missing)

    def _readiness_timer(self):
        if self.state == 'WAITING_FOR_SENSORS':
            reason = self._ready_reason()
            if not reason:
                self.state = 'READY'
                self.detail = 'All safety and localization inputs are ready'
                self._publish_status()
                self.get_logger().info(
                    'GPS-denied navigation READY. Call '
                    '/panther/harvest/start.')
        elif self.state == 'RUNNING':
            reason = self._ready_reason()
            if reason:
                self._invalidate_and_cancel_goal()
                self._publish_zero()
                self.state = 'PAUSED_SENSOR_LOSS'
                self.detail = (
                    f'Navigation paused: waiting for {reason}')
                self._publish_status()
                self.get_logger().error(self.detail)
        elif (
            self.state == 'PAUSED_SENSOR_LOSS'
            and self.autostart
            and self.auto_resume_sensor_loss
        ):
            reason = self._ready_reason()
            if not reason:
                self.state = 'RUNNING'
                self.detail = (
                    'Visual/safety inputs recovered; resuming current segment')
                self._publish_status()
                self.get_logger().info(self.detail)
                self._send_current_segment()
        if (
            self.autostart
            and not self.autostart_attempted
            and self.state == 'READY'
        ):
            self.autostart_attempted = True
            success, message = self._start_mission()
            if not success:
                self.get_logger().error(message)

    def _current_xy(self):
        position = self.current_pose.pose.pose.position
        return (float(position.x), float(position.y))

    def _start_service(self, _, response):
        response.success, response.message = self._start_mission()
        return response

    def _start_mission(self):
        if self.state == 'RUNNING':
            return False, 'Mission is already running'
        reason = self._ready_reason()
        if reason:
            return False, f'Cannot start: waiting for {reason}'
        raw_segments = build_serpentine_segments(
            self.corridors,
            self._current_xy(),
            self.waypoint_spacing,
            self.headland_clearance,
        )
        connected_map = select_start_component(
            self.traversability_map,
            self._current_xy(),
        )
        self.segments, adjusted = snap_segments_to_free(
            raw_segments,
            connected_map,
            self.snap_distance,
        )
        self.segment_index = 0
        self.segment_retry = 0
        self.segment_waypoint_offset = 0
        self.state = 'RUNNING'
        self.detail = 'Starting full serpentine mission'
        self.get_logger().info(
            f'Map validation passed in connected component '
            f'{connected_map["selected_component"]}; adjusted or skipped '
            f'{adjusted} blocked waypoint(s).')
        self._publish_planned_path()
        self._publish_status()
        self._send_current_segment()
        return True, (
            f'Started {len(self.corridors)}-row mission with '
            f'{len(self.segments)} guarded segments'
        )

    def _pause_service(self, _, response):
        if self.state != 'RUNNING':
            response.success = False
            response.message = f'Cannot pause from state {self.state}'
            return response
        self._invalidate_and_cancel_goal()
        self._publish_zero()
        self.state = 'PAUSED'
        self.detail = 'Paused by operator; current segment will restart'
        self._publish_status()
        response.success = True
        response.message = self.detail
        return response

    def _resume_service(self, _, response):
        if self.state not in (
            'PAUSED', 'PAUSED_ESTOP', 'PAUSED_SENSOR_LOSS'
        ):
            response.success = False
            response.message = f'Cannot resume from state {self.state}'
            return response
        reason = self._ready_reason()
        if reason:
            response.success = False
            response.message = f'Cannot resume: waiting for {reason}'
            return response
        self.state = 'RUNNING'
        self.detail = 'Resuming current segment'
        self._publish_status()
        self._send_current_segment()
        response.success = True
        response.message = self.detail
        return response

    def _cancel_service(self, _, response):
        if self.state in ('WAITING_FOR_SENSORS', 'READY', 'CANCELLED'):
            response.success = False
            response.message = f'No active mission in state {self.state}'
            return response
        self._invalidate_and_cancel_goal()
        self._publish_zero()
        self.state = 'CANCELLED'
        self.detail = 'Mission cancelled by operator'
        self._publish_status()
        response.success = True
        response.message = self.detail
        return response

    def _invalidate_and_cancel_goal(self):
        self.goal_token += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None

    def _publish_zero(self):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'panther/base_link'
        self.stop_pub.publish(message)

    def _segment_poses(self, segment):
        all_points = segment['points']
        offset = min(
            self.segment_waypoint_offset,
            max(0, len(all_points) - 1),
        )
        points = all_points[offset:]
        if not points:
            raise RuntimeError(
                f'Segment {segment["name"]} contains no waypoints')
        poses = []
        previous = self._current_xy()
        stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(points):
            next_point = (
                points[index + 1]
                if index + 1 < len(points)
                else point
            )
            yaw = (
                yaw_between(previous, point)
                if math.dist(previous, point) > 1e-6
                else yaw_between(previous, next_point)
                if math.dist(previous, next_point) > 1e-6
                else 0.0
            )
            poses.append(
                pose_from_xy(point, yaw, self.map_frame, stamp))
            previous = point
        return poses

    def _send_current_segment(self):
        if self.state != 'RUNNING':
            return
        if self.segment_index >= len(self.segments):
            self.state = 'COMPLETE'
            self.detail = (
                f'Harvest mission complete: {len(self.corridors)} rows')
            self._publish_zero()
            self._publish_status()
            self.get_logger().info(self.detail)
            return

        reason = self._ready_reason()
        if reason:
            self.state = 'PAUSED_SENSOR_LOSS'
            self.detail = f'Navigation paused: waiting for {reason}'
            self._publish_zero()
            self._publish_status()
            self.get_logger().error(self.detail)
            return

        segment = self.segments[self.segment_index]
        goal = NavigateThroughPoses.Goal()
        goal.poses = self._segment_poses(segment)
        self.active_pose_offset = self.segment_waypoint_offset
        self.active_pose_count = len(goal.poses)
        token = self.goal_token = self.goal_token + 1
        self.detail = (
            f'{segment["name"]} '
            f'({self.segment_index + 1}/{len(self.segments)})'
        )
        self._publish_status()
        self.get_logger().info(self.detail)
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback, active_token=token:
            self._feedback_callback(feedback, active_token),
        )
        future.add_done_callback(
            lambda result, active_token=token:
            self._goal_response_callback(result, active_token))

    def _goal_response_callback(self, future, token):
        if token != self.goal_token or self.state != 'RUNNING':
            return
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self._segment_failed(f'Goal transport error: {error}')
            return
        if not goal_handle.accepted:
            self._segment_failed('Nav2 rejected the segment')
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, active_token=token:
            self._result_callback(result, active_token))

    def _feedback_callback(self, feedback_message, token):
        if token != self.goal_token or self.state != 'RUNNING':
            return
        feedback = feedback_message.feedback
        completed = max(
            0,
            self.active_pose_count - feedback.number_of_poses_remaining,
        )
        self.segment_waypoint_offset = max(
            self.segment_waypoint_offset,
            self.active_pose_offset + completed,
        )
        self.detail = (
            f'{self.segments[self.segment_index]["name"]}: '
            f'{feedback.distance_remaining:.1f} m remaining, '
            f'{feedback.number_of_poses_remaining} waypoints'
        )
        self._publish_status()

    def _result_callback(self, future, token):
        if token != self.goal_token or self.state != 'RUNNING':
            return
        self.goal_handle = None
        wrapped = future.result()
        result = wrapped.result
        if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == NavigateThroughPoses.Result.NONE
        ):
            self.segment_index += 1
            self.segment_retry = 0
            self.segment_waypoint_offset = 0
            self._send_current_segment()
            return
        self._segment_failed(
            f'Nav2 status={wrapped.status}, error={result.error_code}: '
            f'{result.error_msg or "no detail"}'
        )

    def _segment_failed(self, reason):
        if self.segment_retry < self.max_retries:
            self.segment_retry += 1
            self.get_logger().warning(
                f'{reason}; retrying segment '
                f'{self.segment_retry}/{self.max_retries}')
            self._send_current_segment()
            return
        self.state = 'ERROR'
        self.detail = (
            f'{self.segments[self.segment_index]["name"]} failed: {reason}')
        self._publish_zero()
        self._publish_status()
        self.get_logger().error(self.detail)

    def _publish_planned_path(self):
        path = PathMessage()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        previous = self._current_xy()
        for segment in self.segments:
            for point in segment['points']:
                yaw = yaw_between(previous, point)
                path.poses.append(
                    pose_from_xy(
                        point, yaw, self.map_frame, path.header.stamp))
                previous = point
        self.path_pub.publish(path)

    def _publish_status(self):
        message = String()
        message.data = json.dumps({
            'state': self.state,
            'detail': self.detail,
            'segment': self.segment_index,
            'segment_count': len(self.segments),
            'estop': self.estop_active,
        })
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SerpentineMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node._publish_zero()
            except Exception:
                # SIGINT can invalidate the rcl context between the ok()
                # check and publish during a multi-process launch shutdown.
                if rclpy.ok():
                    raise
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
