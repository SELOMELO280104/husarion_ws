"""Regression tests for the repository-compatible landing port."""

import math

import pytest

from uav_ugv_control.repo_landing_core import (
    CascadedPIAxis,
    DoubleQLearner,
    FilteredDerivative,
    MultiResolutionDiscretizer,
    RepoObservation,
    apply_increment,
    rotate_to_stability,
    rotate_to_world,
)


def test_stability_rotation_round_trip():
    stable = rotate_to_stability(2.0, -0.4, 0.73)
    world = rotate_to_world(*stable, 0.73)
    assert world == pytest.approx((2.0, -0.4))


def test_observation_wire_format_round_trip():
    original = RepoObservation(*[float(index) for index in range(18)])
    decoded = RepoObservation.from_array(original.to_array())
    assert decoded == original
    with pytest.raises(ValueError):
        RepoObservation.from_array([1.0])


def test_filtered_derivative_converges_to_ramp_slope():
    derivative = FilteredDerivative(cutoff_hz=2.0)
    value = 0.0
    result = 0.0
    for _ in range(100):
        value += 0.01
        result = derivative.update(value, 0.01)
    assert result == pytest.approx(1.0, abs=0.05)


def test_cascaded_axis_moves_toward_platform():
    controller = CascadedPIAxis()
    correction, relative_velocity_setpoint = controller.update(1.0, 0.0, 0.02)
    assert relative_velocity_setpoint < 0.0
    assert correction > 0.0
    assert abs(correction) <= 2.0


def test_repository_discretization_and_action_semantics():
    discretizer = MultiResolutionDiscretizer(curriculum_step=4)
    center = discretizer.state(0.0, 0.0, 0.0, 0.0,
                               math.radians(21.37723),
                               math.radians(7.12574))
    assert center[:4] == (4, 1, 1, 1)
    increased = apply_increment(
        0.0, 0, math.radians(7.12574), math.radians(21.37723))
    assert increased > 0.0
    unchanged = apply_increment(
        increased, 2, math.radians(7.12574), math.radians(21.37723))
    assert unchanged == increased


def test_double_q_update_and_persistence(tmp_path):
    learner = DoubleQLearner(seed=4)
    state = (0, 0, 1, 1, 3)
    next_state = (0, 1, 1, 1, 4)
    learner.update(state, 0, 2.0, next_state, terminal=True)
    assert learner.visits[state][0] == 1
    assert learner.q_a[state][0] != 0.0 or learner.q_b[state][0] != 0.0
    learner.end_episode()
    target = tmp_path / 'policy.json'
    learner.save(str(target))
    restored = DoubleQLearner(seed=8)
    restored.load(str(target))
    assert restored.episode == 1
    assert restored.visits[state][0] == 1
    assert restored.q_a[state] == learner.q_a[state]
    assert restored.q_b[state] == learner.q_b[state]


def test_upstream_epsilon_schedule():
    assert DoubleQLearner.epsilon_for_episode(0) == 1.0
    assert DoubleQLearner.epsilon_for_episode(800) == 1.0
    assert DoubleQLearner.epsilon_for_episode(2000) == pytest.approx(0.01)
