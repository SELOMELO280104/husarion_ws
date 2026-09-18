from uav_groundtruth_mapping.rl_search_environment import SearchEnvironment
from uav_groundtruth_mapping.safe_spawn import SpawnPose


def test_reset_selects_safe_poses_and_observes():
    env = SearchEnvironment(bounds=(-5, 5, -5, 5), seed=1)
    state = env.reset(
        [SpawnPose(0, 0, 5, 0.5)], [SpawnPose(3, 0, 0, 0.5)])
    assert state.altitude == 5
    assert state.dx == 3


def test_visible_reward_is_positive():
    env = SearchEnvironment()
    env.reset([SpawnPose(0, 0, 5, .5)], [SpawnPose(3, 0, 0, .5)])
    assert env.reward(env.observe(True), 0) == 100.0
