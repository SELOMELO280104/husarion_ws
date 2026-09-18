"""OpenCV popup dashboard for RL search training metrics."""

import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TrainingDashboard(Node):
    def __init__(self):
        super().__init__('rl_training_dashboard')
        self.data = {}
        self.create_subscription(String, '/rl_search/training_status',
                                 self._status, 10)
        self.create_timer(0.2, self._draw)

    def _status(self, message):
        try:
            self.data = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def _draw(self):
        image = np.zeros((620, 900, 3), dtype=np.uint8)
        cv2.putText(image, 'RL UAV SEARCH TRAINING', (30, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 255), 2)
        rows = [('Episodes', self.data.get('episodes', 0)),
                ('Training time', self._time_text('training_elapsed_seconds',
                                                   'training_target_seconds')),
                ('Current episode', self._time_text('episode_elapsed_seconds',
                                                    'training_target_seconds')),
                ('Success rate', f"{self.data.get('success_rate', 0):.1%}"),
                ('Average reward/step', f"{self.data.get('average_reward_per_step', 0):.3f}"),
                ('Cumulative reward', f"{self.data.get('cumulative_reward', 0):.2f}"),
                ('Cumulative penalty', f"{self.data.get('cumulative_penalty', 0):.2f}"),
                ('Net reward', f"{self.data.get('net_reward', 0):.2f}"),
                ('Reward rate/min', f"{self.data.get('reward_rate_per_minute', 0):.2f}"),
                ('Penalty rate/min', f"{self.data.get('penalty_rate_per_minute', 0):.2f}"),
                ('Exploration epsilon', f"{self.data.get('epsilon', 0):.3f}"),
                ('Learned Q states', self.data.get('q_states', 0))]
        for index, (label, value) in enumerate(rows):
            cv2.putText(image, f'{label}: {value}', (45, 95 + index * 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.68, (230, 230, 230), 2)
        cv2.putText(image, 'Close window or press q to stop dashboard',
                    (30, 590), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 1)
        cv2.imshow('RL Training Dashboard', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def _time_text(self, elapsed_key, target_key):
        elapsed = float(self.data.get(elapsed_key, 0.0))
        target = max(1.0, float(self.data.get(target_key, 600.0)))
        return f'{elapsed / 60.0:.1f}/{target / 60.0:.0f} min'


def main(args=None):
    rclpy.init(args=args)
    node = TrainingDashboard()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
