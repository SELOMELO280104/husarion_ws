from uav_groundtruth_mapping.rl_search_trainer import QSearchAgent
from uav_groundtruth_mapping.rl_search_environment import SearchState


def test_agent_learns_and_saves(tmp_path):
    agent = QSearchAgent(seed=1)
    state = SearchState(1, 1, 5, False, 10)
    next_state = SearchState(0, 0, 5, True, 9)
    agent.update(state, 0, 100, next_state, True)
    assert agent.choose(state, training=False) == 0
    path = tmp_path / 'q.json'
    agent.save(path)
    assert path.exists()
