"""Tabular Q-learning trainer for the safe UAV search environment."""

from __future__ import annotations

from collections import defaultdict
import json
import random


class QSearchAgent:
    def __init__(self, action_count=7, alpha=0.2, gamma=0.95,
                 epsilon=1.0, epsilon_decay=0.995, seed=None):
        self.action_count = action_count
        self.alpha, self.gamma = alpha, gamma
        self.epsilon, self.epsilon_decay = epsilon, epsilon_decay
        self.rng = random.Random(seed)
        self.q = defaultdict(lambda: [0.0] * action_count)

    @staticmethod
    def key(state):
        return (round(max(-20, min(20, state.dx)), 1),
                round(max(-20, min(20, state.dy)), 1),
                round(max(0, min(20, state.altitude)), 1),
                int(state.marker_visible))

    def choose(self, state, training=True):
        key = self.key(state)
        if training and self.rng.random() < self.epsilon:
            return self.rng.randrange(self.action_count)
        values = self.q[key]
        return max(range(self.action_count), key=values.__getitem__)

    def update(self, state, action, reward, next_state, done):
        key, next_key = self.key(state), self.key(next_state)
        target = reward if done else reward + self.gamma * max(self.q[next_key])
        self.q[key][action] += self.alpha * (target - self.q[key][action])

    def end_episode(self):
        self.epsilon = max(0.05, self.epsilon * self.epsilon_decay)

    def save(self, path):
        data = {','.join(map(str, key)): values for key, values in self.q.items()}
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2)

    def load(self, path):
        with open(path, encoding='utf-8') as stream:
            data = json.load(stream)
        self.q.clear()
        for key, values in data.items():
            self.q[tuple(float(v) if '.' in v else int(v)
                         for v in key.split(','))] = list(values)


def train(agent, reset_episode, step, episodes=1000):
    """Train against callbacks: ``reset_episode()`` and ``step(action)``."""
    for _ in range(episodes):
        state = reset_episode()
        for _ in range(2000):
            action = agent.choose(state)
            next_state, reward, done = step(action)
            agent.update(state, action, reward, next_state, done)
            state = next_state
            if done:
                break
        agent.end_episode()
    return agent
