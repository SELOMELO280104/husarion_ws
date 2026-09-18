#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point 
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from ultralytics import YOLO
import time
import cv2

class HybridTracker(Node):
    def __init__(self):
        super().__init__('hybrid_tracker_node')
        self.bridge = CvBridge()
        
        # YOLO 26 Modelin
        model_path = '/home/robocare/Desktop/selim/runs/detect/uav_landing_project/yolo26_aruco_fallback-2/weights/best.pt'
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO 26 ağırlıkları yüklendi. (Engel algılama DEVRE DIŞI)')

        self.last_aruco_time = 0.0
        self.timeout_threshold = 0.5 
        self.tracking_active = False 
        
        self.aruco_x = -1.0
        self.aruco_y = -1.0

        # Sadece ArUco ve RGB Görüntü kanalları kaldı, Depth silindi
        self.aruco_sub = self.create_subscription(Point, '/uav/tag_follower/detection', self.aruco_callback, 10)
        self.image_sub = self.create_subscription(Image, '/uav/tag_camera/image', self.image_callback, qos_profile_sensor_data)

        self.unified_pub = self.create_publisher(Point, '/uav/tag_follower/unified_target', 10)
        self.debug_image_pub = self.create_publisher(Image, '/uav/ai_vision', 10)
        
        self.start_srv = self.create_service(Trigger, '/uav/tag_follower/start_locating', self.start_callback)

    def start_callback(self, request, response):
        self.tracking_active = True
        response.success = True
        return response

    def aruco_callback(self, msg):
        if not self.tracking_active:
            return
        self.last_aruco_time = time.time()
        self.aruco_x = msg.x
        self.aruco_y = msg.y
        self.unified_pub.publish(msg)

    def image_callback(self, msg):
        current_time = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            if self.tracking_active:
                if (current_time - self.last_aruco_time) > self.timeout_threshold:
                    results = self.model(cv_image, verbose=False)
                    if len(results[0].boxes) > 0:
                        box = results[0].boxes[0]
                        x_center = float((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                        y_center = float((box.xyxy[0][1] + box.xyxy[0][3]) / 2)

                        yolo_msg = Point()
                        yolo_msg.x, yolo_msg.y = x_center, y_center
                        self.unified_pub.publish(yolo_msg)
                        
                        cv2.rectangle(cv_image, (int(box.xyxy[0][0]), int(box.xyxy[0][1])), (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (0, 0, 255), 2)
                        cv2.putText(cv_image, "YOLO 26 LOCKED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    cx, cy = int(self.aruco_x), int(self.aruco_y)
                    cv2.circle(cv_image, (cx, cy), 20, (0, 255, 0), 2)
                    cv2.line(cv_image, (cx-30, cy), (cx+30, cy), (0, 255, 0), 2)
                    cv2.line(cv_image, (cx, cy-30), (cx, cy+30), (0, 255, 0), 2)
                    cv2.putText(cv_image, "ArUco LOCKED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(cv_image, "READY: Press 'START HYBRID TRACKING'", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HybridTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
