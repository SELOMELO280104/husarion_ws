import csv
import math

from uav_groundtruth_mapping.serpentine_mission_manager import (
    build_serpentine_segments,
    load_corridors,
    load_traversability_map,
    sample_line,
    select_start_component,
    snap_segments_to_free,
)


def test_sample_line_excludes_start_and_includes_end():
    points = sample_line((0.0, 0.0), (2.2, 0.0), 1.0)
    assert len(points) == 3
    assert points[0] != (0.0, 0.0)
    assert points[-1] == (2.2, 0.0)
    assert all(
        math.dist(first, second) <= 1.0
        for first, second in zip([(0.0, 0.0)] + points, points)
    )


def test_serpentine_alternates_and_connects_rows():
    corridors = [
        {'id': 1, 'start': (0.0, 0.0), 'end': (10.0, 0.0), 'width': 1.1},
        {'id': 2, 'start': (0.0, 3.0), 'end': (10.0, 3.0), 'width': 1.1},
        {'id': 3, 'start': (0.0, 6.0), 'end': (10.0, 6.0), 'width': 1.1},
    ]
    segments = build_serpentine_segments(
        corridors, (-2.0, -1.0), waypoint_spacing=1.0)

    assert [segment['kind'] for segment in segments] == [
        'approach',
        'row',
        'headland_exit',
        'headland_shift',
        'headland_enter',
        'row',
        'headland_exit',
        'headland_shift',
        'headland_enter',
        'row',
    ]
    assert segments[0]['points'][-1] == (0.0, 0.0)
    assert segments[1]['points'][-1] == (10.0, 0.0)
    assert max(point[0] for point in segments[2]['points']) == 12.25
    assert segments[3]['points'][-1] == (12.25, 3.0)
    assert segments[4]['points'][-1] == (10.0, 3.0)
    assert segments[5]['points'][-1] == (0.0, 3.0)
    assert min(point[0] for point in segments[6]['points']) == -2.25
    assert segments[7]['points'][-1] == (-2.25, 6.0)
    assert segments[8]['points'][-1] == (0.0, 6.0)
    assert segments[9]['points'][-1] == (10.0, 6.0)


def test_load_corridors_validates_csv(tmp_path):
    path = tmp_path / 'corridors.csv'
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'corridor_id', 'start_x', 'start_y',
            'end_x', 'end_y', 'nominal_width_m'])
        writer.writerow([2, 0, 3, 10, 3, 1.1])
        writer.writerow([1, 0, 0, 10, 0, 1.1])

    corridors = load_corridors(path)
    assert [corridor['id'] for corridor in corridors] == [1, 2]


def test_waypoints_are_snapped_to_free_cells(tmp_path):
    pgm = tmp_path / 'map.pgm'
    pgm.write_bytes(
        b'P5\n4 3\n255\n'
        + bytes([
            0, 0, 0, 0,
            0, 254, 254, 0,
            0, 0, 0, 0,
        ])
    )
    yaml_path = tmp_path / 'map.yaml'
    yaml_path.write_text(
        'image: map.pgm\n'
        'resolution: 1.0\n'
        'origin: [0.0, 0.0, 0.0]\n',
        encoding='utf-8',
    )
    nav_map = load_traversability_map(yaml_path)
    nav_map = select_start_component(nav_map, (1.5, 1.5))
    segments, snapped = snap_segments_to_free(
        [{'name': 'test', 'kind': 'row', 'points': [(1.0, 1.0)]}],
        nav_map,
        max_distance=1.0,
    )
    assert snapped == 0
    assert segments[0]['points'] == [(1.5, 1.5)]
