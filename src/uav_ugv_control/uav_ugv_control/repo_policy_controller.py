#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode
from std_msgs.msg import Bool
import time

class RepoPolicyController(Node):
    def __init__(self):
        super().__init__('repo_policy_controller')

        self.fx = 554.256
        self.fy = 554.256
        self.cx = 320.0
        self.cy = 240.0
        self.current_z = 2.0 

        self.landing_active = False
        self.blind_landing_engaged = False 
        self.last_target_time = 0.0
        self.last_callback_time = time.time()
        self.landing_start_time = 0.0  # YENİ: İnişin başladığı anı tutacak
        
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0

        # KONTROL KAZANÇLARI (Sanal Hedefli)
        self.Kp = 3.0    
        self.Ki = 1.5    
        self.Kd = 0.2
        
        self.pixel_lookahead_time = 1
        self.alpha = 0.2  
        
        self.prev_u = 320.0
        self.prev_v = 240.0
        self.du_filtered = 0.0
        self.dv_filtered = 0.0

        self.err_x_sum = 0.0
        self.err_y_sum = 0.0
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0

        self.get_logger().info('Zaman Ayarlı Kör İniş ve Sanal Hedef Aktif!')

        self.target_sub = self.create_subscription(Point, '/uav/tag_follower/unified_target', self.target_callback, 10)
        self.handoff_sub = self.create_subscription(Bool, '/uav/tag_follower/landing_handoff_ready', self.handoff_callback, 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def handoff_callback(self, msg):
        if msg.data and not self.landing_active:
            self.landing_active = True
            self.landing_start_time = time.time()  # İnişin başladığı kronometreyi başlat
            self.get_logger().info('İniş Yetkisi Alındı! 3 Saniye Sonra Zorunlu İniş (Touchdown) Başlayacak...')

    def target_callback(self, msg):
        if self.blind_landing_engaged:
            return 

        current_time = time.time()
        dt = current_time - self.last_callback_time
        if dt <= 0.001:
            dt = 0.05

        self.last_target_time = current_time
        self.last_callback_time = current_time

        u = msg.x
        v = msg.y

        du = (u - self.prev_u) / dt
        dv = (v - self.prev_v) / dt

        self.du_filtered = (self.alpha * du) + ((1.0 - self.alpha) * self.du_filtered)
        self.dv_filtered = (self.alpha * dv) + ((1.0 - self.alpha) * self.dv_filtered)

        self.prev_u = u
        self.prev_v = v

        u_virtual = u + (self.du_filtered * self.pixel_lookahead_time)
        v_virtual = v + (self.dv_filtered * self.pixel_lookahead_time)

        rel_x = (u_virtual - self.cx) * self.current_z / self.fx
        rel_y = (v_virtual - self.cy) * self.current_z / self.fy
        
        self.err_x_sum += rel_x * dt
        self.err_y_sum += rel_y * dt
        
        max_i = 2.0
        self.err_x_sum = max(-max_i, min(max_i, self.err_x_sum))
        self.err_y_sum = max(-max_i, min(max_i, self.err_y_sum))
        
        d_err_x = (rel_x - self.prev_err_x) / dt
        d_err_y = (rel_y - self.prev_err_y) / dt
        self.prev_err_x = rel_x
        self.prev_err_y = rel_y

        self.cmd_vx = (self.Kp * rel_x) + (self.Ki * self.err_x_sum) + (self.Kd * d_err_x)
        self.cmd_vy = (self.Kp * rel_y) + (self.Ki * self.err_y_sum) + (self.Kd * d_err_y)
        
        if self.landing_active:
            self.cmd_vz = 0.6  
        else:
            self.cmd_vz = 0.0

    def timer_callback(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

        current_time = time.time()
        
        if self.landing_active:
            # YENİ: Kamera hedefi kaybetse de kaybetmese de, iniş başlayalı 3 saniye olduysa kör inişi ZORLA!
            if (current_time - self.last_target_time) > 0.6 or (current_time - self.landing_start_time) > 3.0:
                self.blind_landing_engaged = True
            
            if self.blind_landing_engaged:
                # X ve Y'deki son hızı korur (target_callback pas geçildiği için cmd_vx/vy sabit kalır)
                self.cmd_vz = 1.0  # Yere daha sert oturması için 0.8'den 1.0'a çıkarıldı
                self.get_logger().info('Kör İniş Devrede! PID Donduruldu, Mevcut Hızla Araç Tavanına İniş Yapılıyor...', throttle_duration_sec=1.0)
            
            self.publish_velocity_command(self.cmd_vx, self.cmd_vy, self.cmd_vz)
        else:
            if (current_time - self.last_target_time) > 0.6:
                self.cmd_vx = 0.0
                self.cmd_vy = 0.0
                self.cmd_vz = 0.0
            
            self.publish_velocity_command(self.cmd_vx, self.cmd_vy, self.cmd_vz)

    def publish_velocity_command(self, vx, vy, vz):
        max_vel = 6.0
        vx = max(-max_vel, min(max_vel, vx))
        vy = max(-max_vel, min(max_vel, vy))

        msg = TrajectorySetpoint()
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan') 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RepoPolicyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
