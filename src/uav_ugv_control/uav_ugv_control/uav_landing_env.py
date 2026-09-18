import rclpy
from rclpy.node import Node
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time

from geometry_msgs.msg import Point
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

class UAVLandingEnv(gym.Env, Node):
    def __init__(self):
        gym.Env.__init__(self)
        Node.__init__(self, 'rl_landing_env')

        self.action_space = spaces.Box(low=-3.0, high=3.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-100.0, high=100.0, shape=(5,), dtype=np.float32)

        self.rel_x = 0.0
        self.rel_y = 0.0
        self.rel_z = 2.0
        self.target_vel_x = 0.0
        self.target_vel_y = 0.0
        
        self.step_count = 0
        self.max_steps = 500

        self.target_sub = self.create_subscription(Point, '/uav/tag_follower/unified_target', self.target_callback, 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        self.set_pose_client = self.create_client(SetEntityPose, '/world/empty/set_pose')

    def target_callback(self, msg):
        self.rel_x = msg.x
        self.rel_y = msg.y
        self.rel_z = msg.z 

    def step(self, action):
        self.step_count += 1
        
        self._publish_action(action[0], action[1], action[2])
        time.sleep(0.1) 
        
        obs = np.array([self.rel_x, self.rel_y, self.rel_z, self.target_vel_x, self.target_vel_y], dtype=np.float32)
        
        distance = np.sqrt(self.rel_x**2 + self.rel_y**2)
        reward = -distance
        
        terminated = False
        truncated = False
        
        if distance < 0.2 and self.rel_z < 0.3:
            reward += 1000.0
            terminated = True
            print("BAŞARILI İNİŞ!")
        elif distance > 5.0 or self.rel_z < -0.5:
            reward -= 500.0
            terminated = True
            print("ÇAKILMA VEYA UZAKLAŞMA!")
            
        if self.step_count >= self.max_steps:
            truncated = True

        return obs, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        print("Bölüm Sıfırlanıyor (Işınlanma Başladı)...")
        
        self._publish_action(0.0, 0.0, 0.0)
        
        self._teleport_entity("x500", -6.0, -9.0, 1.5)
        self._teleport_entity("panther", -6.0, -8.0, 0.1)
        
        time.sleep(2.0)
        obs = np.array([0.0, 0.0, 2.0, 0.0, 0.0], dtype=np.float32)
        return obs, {}

    def _teleport_entity(self, name, x, y, z):
        if not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            return
            
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z
        self.set_pose_client.call_async(req)

    def _publish_action(self, vx, vy, vz):
        mode_msg = OffboardControlMode()
        mode_msg.position = False
        mode_msg.velocity = True
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(mode_msg)
        
        traj_msg = TrajectorySetpoint()
        traj_msg.velocity = [float(vx), float(vy), float(vz)]
        traj_msg.position = [float('nan'), float('nan'), float('nan')]
        traj_msg.yaw = float('nan')
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(traj_msg)
