import rclpy
import threading
import sys
import os

# Ortam dosyasının bulunduğu klasörü Python'a tanıtıyoruz
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from stable_baselines3 import PPO
from uav_landing_env import UAVLandingEnv

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    
    env = UAVLandingEnv()
    
    spin_thread = threading.Thread(target=ros_spin_thread, args=(env,), daemon=True)
    spin_thread.start()
    
    print("PPO Modeli Yükleniyor...")
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_uav_landing_tensorboard/")
    
    print("Eğitim Başlıyor! (Ctrl+C ile durdurabilirsiniz)")
    try:
        model.learn(total_timesteps=500000)
        model.save("ppo_uav_landing_model")
        print("Model başarıyla kaydedildi: ppo_uav_landing_model.zip")
    except KeyboardInterrupt:
        print("Eğitim manuel olarak durduruldu, mevcut ağırlıklar kaydediliyor...")
        model.save("ppo_uav_landing_model_interrupted")
        
    env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
