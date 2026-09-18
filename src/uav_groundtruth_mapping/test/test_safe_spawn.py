from uav_groundtruth_mapping.safe_spawn import SpawnPose, is_safe_spawn


def test_rejects_obstacle_and_accepts_clear_pose():
    obstacle = ((0.0, 1.0, 0.0, 1.0),)
    assert not is_safe_spawn(SpawnPose(0.5, 0.5, 0.0, 0.2), obstacle)
    assert is_safe_spawn(SpawnPose(2.0, 2.0, 0.0, 0.2), obstacle)


def test_rejects_uav_ugv_overlap():
    ugv = SpawnPose(0.0, 0.0, 0.0, 0.8)
    assert not is_safe_spawn(SpawnPose(0.5, 0.0, 2.0, 0.8), other_poses=(ugv,))
