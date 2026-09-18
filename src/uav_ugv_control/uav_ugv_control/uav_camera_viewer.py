#!/usr/bin/env python3
"""Display the UAV detector image with fallback to raw camera."""

from __future__ import annotations
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
import time

class UavCameraViewer(Node):
    def __init__(self):
        super().__init__('uav_marker_camera_viewer')
        
        self.declare_parameter('window_title', 'UAV Camera - Hybrid Tracking')
        self.declare_parameter('start_service', '/uav/tag_follower/start_locating')
        self.declare_parameter('landing_service', '/uav/tag_follower/request_rl_landing')
        self.declare_parameter('window_width', 800)
        self.declare_parameter('window_height', 600)

        self.window_title = str(self.get_parameter('window_title').value)
        self.start_service = str(self.get_parameter('start_service').value)
        self.landing_service = str(self.get_parameter('landing_service').value)
        self.window_width = int(self.get_parameter('window_width').value)
        self.window_height = int(self.get_parameter('window_height').value)

        self.bridge = CvBridge()
        
        # Çift Kanal Değişkenleri
        self.latest_ai_frame = None
        self.latest_raw_frame = None
        self.last_ai_time = 0.0

        self.window_created = False
        self.button_rect = None
        self.landing_rect = None
        self.tracking_active = False
        self.landing_active = False

        # 1. ÖNCELİKLİ KANAL: Yapay Zekanın Çizimli Görüntüsü
        self.create_subscription(Image, '/uav/ai_vision', self._ai_callback, 10)
        
        # 2. YEDEK KANAL: Dronun Ham Kamerası (Siyah ekranı önlemek için)
        self.create_subscription(Image, '/uav/tag_camera/image', self._raw_callback, qos_profile_sensor_data)

        self.start_client = self.create_client(Trigger, self.start_service)
        self.landing_client = self.create_client(Trigger, self.landing_service)

        self.timer = self.create_timer(1.0 / 30.0, self._display)
        self._create_window()
        self._display()
        self.get_logger().info('Akıllı Arayüz başlatıldı. Hem AI hem de Ham Kamera dinleniyor.')

    def _waiting_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, 'Waiting for UAV camera...', (105, 215),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        return frame

    def _create_window(self):
        if self.window_created:
            return
        cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_title, self.window_width, self.window_height)
        cv2.setMouseCallback(self.window_title, self._mouse_callback)
        self.window_created = True

    def _request_start(self):
        self.tracking_active = True
        if self.start_client.service_is_ready():
            self.start_client.call_async(Trigger.Request())
            self.get_logger().info('Takip tetiklendi!')

    def _request_landing(self):
        self.landing_active = True
        if self.landing_client.service_is_ready():
            self.landing_client.call_async(Trigger.Request())
            self.get_logger().info('İniş tetiklendi!')

    def _mouse_callback(self, event, x, y, _flags, _parameter):
        if event != cv2.EVENT_LBUTTONUP:
            return
        if self.button_rect:
            left, top, right, bottom = self.button_rect
            if left <= x <= right and top <= y <= bottom:
                self._request_start()
        if self.landing_rect:
            left, top, right, bottom = self.landing_rect
            if left <= x <= right and top <= y <= bottom:
                self._request_landing()

    def _compose_display(self, frame):
        output_width = max(self.window_width, 320)
        output_height = max(self.window_height, 240)
        control_height = min(150, max(output_height // 4, 120))
        camera_height = output_height - control_height
        canvas = np.full((output_height, output_width, 3), (25, 25, 25), dtype=np.uint8)

        frame_height, frame_width = frame.shape[:2]
        scale = min(output_width / max(frame_width, 1), camera_height / max(frame_height, 1))
        resized_width = max(int(round(frame_width * scale)), 1)
        resized_height = max(int(round(frame_height * scale)), 1)
        resized = cv2.resize(frame, (resized_width, resized_height), interpolation=cv2.INTER_AREA)

        offset_x = (output_width - resized_width) // 2
        offset_y = (camera_height - resized_height) // 2
        canvas[offset_y:offset_y + resized_height, offset_x:offset_x + resized_width] = resized

        margin = 10
        self.button_rect = (margin, camera_height + margin, output_width // 2 - 5, output_height - margin)
        self.landing_rect = (output_width // 2 + 5, camera_height + margin, output_width - margin, output_height - margin)

        left, top, right, bottom = self.button_rect
        if self.tracking_active:
            cv2.rectangle(canvas, (left, top), (right, bottom), (0, 100, 0), -1)
            text = 'TRACKING ACTIVE'
        else:
            cv2.rectangle(canvas, (left, top), (right, bottom), (45, 175, 70), -1)
            text = 'START HYBRID TRACKING'
        cv2.rectangle(canvas, (left, top), (right, bottom), (235, 235, 235), 2)
        cv2.putText(canvas, text, (left + 15, top + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        ll, lt, lr, lb = self.landing_rect
        if self.landing_active:
            cv2.rectangle(canvas, (ll, lt), (lr, lb), (0, 0, 139), -1)
            text = 'LANDING IN PROGRESS'
        else:
            cv2.rectangle(canvas, (ll, lt), (lr, lb), (35, 75, 180), -1)
            text = 'START RL LANDING'
        cv2.rectangle(canvas, (ll, lt), (lr, lb), (235, 235, 235), 2)
        cv2.putText(canvas, text, (ll + 15, lt + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        return canvas

    def _ai_callback(self, message):
        try:
            self.latest_ai_frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8').copy()
            self.last_ai_time = time.time()
        except Exception:
            pass

    def _raw_callback(self, message):
        try:
            self.latest_raw_frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8').copy()
        except Exception:
            pass

    def _display(self):
        self._create_window()
        
        # AKILLI SEÇİCİ: Son 1 saniyede AI'dan görüntü geldiyse onu göster, yoksa ham kamerayı göster.
        if time.time() - self.last_ai_time < 1.0 and self.latest_ai_frame is not None:
            frame = self.latest_ai_frame
        elif self.latest_raw_frame is not None:
            frame = self.latest_raw_frame
            # Yapay zeka kapalıysa ekrana uyarı bas
            cv2.putText(frame, "AI OFFLINE - SHOWING RAW CAMERA", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            frame = self._waiting_frame()

        cv2.imshow(self.window_title, self._compose_display(frame))
        
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            self.user_closed = True
            cv2.destroyWindow(self.window_title)

def main(args=None):
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
