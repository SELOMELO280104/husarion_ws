"""Display the UAV detector image in a reliable, immediately visible window."""

from __future__ import annotations

import json
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


class UavCameraViewer(Node):
    """Show the annotated camera stream using matching sensor-data QoS."""

    def __init__(self):
        super().__init__('uav_marker_camera_viewer')
        self.declare_parameter(
            'image_topic', '/uav/tag_follower/debug_image')
        self.declare_parameter(
            'window_title', 'UAV Camera - Marker Detection')
        self.declare_parameter(
            'status_topic', '/uav/tag_follower/status')
        self.declare_parameter(
            'start_service', '/uav/tag_follower/start_locating')
        self.declare_parameter('window_width', 800)
        self.declare_parameter('window_height', 600)
        self.declare_parameter('always_on_top', True)

        self.image_topic = str(
            self.get_parameter('image_topic').value)
        self.window_title = str(
            self.get_parameter('window_title').value)
        self.status_topic = str(
            self.get_parameter('status_topic').value)
        self.start_service = str(
            self.get_parameter('start_service').value)
        self.window_width = int(
            self.get_parameter('window_width').value)
        self.window_height = int(
            self.get_parameter('window_height').value)
        self.always_on_top = bool(
            self.get_parameter('always_on_top').value)

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
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_status = {}
        self.received_frames = 0
        self.user_closed = False
        self.window_created = False
        self.created_monotonic = None
        self.button_rect = None
        self.start_request_pending = False
        self.start_future = None
        self.start_feedback = ''
        self.create_subscription(
            Image, self.image_topic, self._image_callback, sensor_qos)
        self.create_subscription(
            String, self.status_topic, self._status_callback, latched_qos)
        self.start_client = self.create_client(Trigger, self.start_service)
        self.timer = self.create_timer(1.0 / 30.0, self._display)
        self._create_window()
        self._display()
        self.get_logger().info(
            f'Opened "{self.window_title}"; waiting for {self.image_topic}.')

    def _waiting_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            frame,
            'Waiting for UAV camera...',
            (105, 215),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            self.image_topic,
            (100, 260),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (210, 210, 210),
            1,
        )
        return frame

    def _create_window(self):
        if self.window_created:
            return
        cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(
            self.window_title, self.window_width, self.window_height)
        cv2.moveWindow(self.window_title, 20, 20)
        cv2.setMouseCallback(self.window_title, self._mouse_callback)
        if self.always_on_top and hasattr(cv2, 'WND_PROP_TOPMOST'):
            cv2.setWindowProperty(
                self.window_title, cv2.WND_PROP_TOPMOST, 1)
        self.window_created = True
        self.created_monotonic = time.monotonic()

    def _status_callback(self, message):
        try:
            self.latest_status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning(
                'Cannot decode follower status for the camera controls.',
                throttle_duration_sec=2.0,
            )

    def _request_start(self):
        if bool(self.latest_status.get('locating_requested', False)):
            self.start_feedback = 'Locating is already authorized.'
            return
        if self.start_request_pending:
            return
        self.start_request_pending = True
        self.start_feedback = 'Waiting for follower service...'
        self.get_logger().info('Start Locating pressed by operator.')
        self._try_send_start_request()

    def _try_send_start_request(self):
        if not self.start_request_pending or self.start_future is not None:
            return
        if bool(self.latest_status.get('locating_requested', False)):
            self.start_request_pending = False
            return
        if not self.start_client.service_is_ready():
            return
        self.start_feedback = 'Sending locating authorization...'
        self.start_future = self.start_client.call_async(Trigger.Request())
        self.start_future.add_done_callback(self._start_response)

    def _start_response(self, future):
        try:
            response = future.result()
            self.start_feedback = str(response.message)
            if response.success:
                self.get_logger().info(response.message)
                self.start_request_pending = False
            else:
                self.get_logger().error(response.message)
                self.start_request_pending = False
        except Exception as error:
            self.start_feedback = f'Start request failed: {error}'
            self.start_request_pending = False
            self.get_logger().error(self.start_feedback)
        finally:
            self.start_future = None

    def _mouse_callback(self, event, x, y, _flags, _parameter):
        if event != cv2.EVENT_LBUTTONUP or self.button_rect is None:
            return
        left, top, right, bottom = self.button_rect
        if left <= x <= right and top <= y <= bottom:
            self._request_start()

    def _button_content(self):
        mode = str(self.latest_status.get('mode', 'WAITING_FOR_FOLLOWER'))
        requested = bool(
            self.latest_status.get('locating_requested', False))
        active = bool(self.latest_status.get('locating_active', False))
        if active:
            return 'LOCATING ACTIVE', (45, 150, 45), mode
        if requested:
            remaining = self.latest_status.get('head_start_remaining_s')
            detail = mode
            if remaining is not None and float(remaining) > 0.0:
                detail = f'Panther head start: {float(remaining):.1f} s remaining'
            return 'LOCATING ARMED', (0, 155, 230), detail
        if self.start_request_pending:
            return 'START REQUESTED', (0, 155, 230), self.start_feedback
        return 'START LOCATING', (45, 175, 70), 'Click here or press S'

    def _compose_display(self, frame):
        """Fit the camera above a persistent, clickable control bar."""
        output_width = max(self.window_width, 320)
        output_height = max(self.window_height, 240)
        control_height = min(96, max(output_height // 5, 72))
        camera_height = output_height - control_height
        canvas = np.full(
            (output_height, output_width, 3), (25, 25, 25), dtype=np.uint8)

        frame_height, frame_width = frame.shape[:2]
        scale = min(
            output_width / max(frame_width, 1),
            camera_height / max(frame_height, 1),
        )
        resized_width = max(int(round(frame_width * scale)), 1)
        resized_height = max(int(round(frame_height * scale)), 1)
        resized = cv2.resize(
            frame, (resized_width, resized_height), interpolation=cv2.INTER_AREA)
        offset_x = (output_width - resized_width) // 2
        offset_y = (camera_height - resized_height) // 2
        canvas[
            offset_y:offset_y + resized_height,
            offset_x:offset_x + resized_width,
        ] = resized

        margin = 10
        self.button_rect = (
            margin,
            camera_height + margin,
            output_width - margin,
            output_height - margin,
        )
        label, color, detail = self._button_content()
        left, top, right, bottom = self.button_rect
        cv2.rectangle(canvas, (left, top), (right, bottom), color, -1)
        cv2.rectangle(canvas, (left, top), (right, bottom), (235, 235, 235), 2)
        label_size = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.78, 2)[0]
        label_x = max((output_width - label_size[0]) // 2, left + 6)
        label_y = top + 30
        cv2.putText(
            canvas, label, (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.78, (255, 255, 255), 2)
        detail_text = str(detail)[:92]
        detail_size = cv2.getTextSize(
            detail_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0]
        detail_x = max((output_width - detail_size[0]) // 2, left + 6)
        cv2.putText(
            canvas, detail_text, (detail_x, min(label_y + 24, bottom - 7)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (245, 245, 245), 1)
        return canvas

    def _image_callback(self, message):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8').copy()
            self.received_frames += 1
            if self.received_frames == 1:
                self.get_logger().info(
                    'Displaying annotated UAV marker detections.')
        except Exception as error:
            self.get_logger().warning(
                f'Cannot decode annotated camera image: {error}',
                throttle_duration_sec=2.0,
            )

    def _display(self):
        if self.user_closed:
            return
        self._create_window()
        self._try_send_start_request()
        frame = (
            self.latest_frame
            if self.latest_frame is not None
            else self._waiting_frame()
        )
        cv2.imshow(self.window_title, self._compose_display(frame))
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('s'), ord('S')):
            self._request_start()
        if key in (27, ord('q')):
            self.user_closed = True
            cv2.destroyWindow(self.window_title)
            self.get_logger().info('UAV camera window closed by operator.')
            return
        try:
            visible = cv2.getWindowProperty(
                self.window_title, cv2.WND_PROP_VISIBLE)
            if visible < 1.0:
                self.user_closed = True
                self.get_logger().info(
                    'UAV camera window closed by operator.')
        except cv2.error:
            self.user_closed = True

    def destroy_node(self):
        if self.window_created:
            try:
                cv2.destroyWindow(self.window_title)
                cv2.waitKey(1)
            except cv2.error:
                pass
        return super().destroy_node()


def main(args=None):
    """Run the annotated UAV camera viewer."""
    rclpy.init(args=args)
    node = UavCameraViewer()
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
