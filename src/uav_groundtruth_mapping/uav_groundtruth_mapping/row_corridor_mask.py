"""Extract Panther-safe orchard row corridors from a UAV 3D point cloud."""

import argparse
import csv
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation, gaussian_filter1d
from scipy.signal import find_peaks

from uav_gps_mapping.pcd_to_occupancy import (
    is_ascii_pcd,
    packed_rgb_float,
    read_xyz,
    reachable_component,
    save_map,
)


# Physical collision / LiDAR walls in orchard_safety_walls.sdf. Keeping the
# same coordinates here makes the static localization map and live scan agree.
ORCHARD_WALL_BOUNDS = {
    'west': -9.05,
    'east': 37.83,
    'south': -9.90,
    'north': 27.74,
}
ORCHARD_WALL_HALF_THICKNESS = 0.15


def orchard_boundary_wall_mask(shape, origin, resolution):
    """Rasterize the four physical orchard walls into a grid."""
    grid_y, grid_x = np.indices(shape, dtype=np.float64)
    world_x = origin[0] + (grid_x + 0.5) * resolution
    world_y = origin[1] + (grid_y + 0.5) * resolution
    half = ORCHARD_WALL_HALF_THICKNESS
    return (
        (np.abs(world_x - ORCHARD_WALL_BOUNDS['west']) <= half)
        | (np.abs(world_x - ORCHARD_WALL_BOUNDS['east']) <= half)
        | (np.abs(world_y - ORCHARD_WALL_BOUNDS['south']) <= half)
        | (np.abs(world_y - ORCHARD_WALL_BOUNDS['north']) <= half)
    )


def newest_groundtruth_map(map_directory):
    """Return the newest nonempty ASCII ground-truth PCD."""
    candidates = [
        path
        for path in Path(map_directory).glob(
            'uav_groundtruth_map_*.pcd')
        if path.stat().st_size > 1024 and is_ascii_pcd(path)
    ]
    if not candidates:
        raise FileNotFoundError(
            'No uav_groundtruth_map_*.pcd found in '
            f'{map_directory}')
    return max(candidates, key=lambda path: path.stat().st_mtime)


def circular_structure(radius_cells):
    """Return a disk-shaped structuring element."""
    radius_cells = max(0, int(radius_cells))
    axis = np.arange(-radius_cells, radius_cells + 1)
    yy, xx = np.meshgrid(axis, axis, indexing='ij')
    return xx * xx + yy * yy <= radius_cells * radius_cells


def estimate_ground_height(points, quantile=0.15):
    """Estimate the orchard ground plane height from the lower cloud mode."""
    points = np.asarray(points)
    if len(points) == 0:
        raise ValueError('Point cloud contains no finite points')
    return float(np.quantile(points[:, 2], quantile))


def _unique_xy_cells(points_xy, resolution):
    cells = np.unique(
        np.floor(points_xy / resolution).astype(np.int32),
        axis=0,
    )
    return (cells.astype(np.float64) + 0.5) * resolution


def _profile_peaks(profile, distance, prominence):
    """Find peaks while allowing the first and last row to be maxima."""
    padding = max(2, int(distance))
    padded = np.pad(
        profile,
        (padding, padding),
        mode='constant',
        constant_values=0.0,
    )
    peaks, properties = find_peaks(
        padded,
        distance=distance,
        prominence=prominence,
    )
    peaks = peaks - padding
    valid = (peaks >= 0) & (peaks < len(profile))
    return (
        peaks[valid],
        {
            name: values[valid]
            for name, values in properties.items()
        },
    )


def estimate_row_geometry(
    obstacle_xy,
    *,
    profile_resolution=0.15,
    angle_step_deg=0.5,
    minimum_row_spacing=1.5,
):
    """Estimate row direction, row centers, spacing, and along-row extents."""
    obstacle_xy = np.asarray(obstacle_xy, dtype=np.float64)
    if len(obstacle_xy) < 100:
        raise ValueError(
            'Too few vegetation points to detect orchard rows')
    sampled = _unique_xy_cells(obstacle_xy, profile_resolution)
    center = np.mean(sampled, axis=0)
    centered = sampled - center

    best_score = -math.inf
    best_angle = None
    for angle_deg in np.arange(0.0, 180.0, angle_step_deg):
        angle = math.radians(float(angle_deg))
        normal = np.array([-math.sin(angle), math.cos(angle)])
        cross_row = centered @ normal
        edges = np.arange(
            cross_row.min(),
            cross_row.max() + 2.0 * profile_resolution,
            profile_resolution,
        )
        profile, _ = np.histogram(cross_row, bins=edges)
        smooth = gaussian_filter1d(
            profile.astype(np.float64), 1.5)
        score = float(
            np.std(smooth) / (np.mean(smooth) + 1e-9))
        if score > best_score:
            best_score = score
            best_angle = float(angle_deg)

    angle = math.radians(best_angle)
    direction = np.array([math.cos(angle), math.sin(angle)])
    normal = np.array([-math.sin(angle), math.cos(angle)])
    along = sampled @ direction
    cross = sampled @ normal
    edges = np.arange(
        cross.min(),
        cross.max() + 2.0 * profile_resolution,
        profile_resolution,
    )
    profile, _ = np.histogram(cross, bins=edges)
    smooth = gaussian_filter1d(
        profile.astype(np.float64), 2.0)
    peak_distance = max(
        1,
        int(round(minimum_row_spacing / profile_resolution)),
    )
    prominence = max(2.0, 0.10 * float(np.max(smooth)))
    peaks, properties = _profile_peaks(
        smooth,
        peak_distance,
        prominence,
    )
    if len(peaks) < 2:
        peaks, properties = _profile_peaks(
            smooth,
            peak_distance,
            max(1.0, 0.05 * float(np.max(smooth))),
        )
    if len(peaks) < 2:
        raise ValueError(
            'Could not detect at least two vegetation rows')

    cross_centers = 0.5 * (edges[:-1] + edges[1:])
    row_centers = cross_centers[peaks]
    order = np.argsort(row_centers)
    row_centers = row_centers[order]
    prominences = properties['prominences'][order]
    refinement_radius = 0.45 * minimum_row_spacing
    row_centers = np.asarray([
        float(np.median(
            cross[np.abs(cross - row_center) <= refinement_radius]))
        if np.any(
            np.abs(cross - row_center) <= refinement_radius)
        else float(row_center)
        for row_center in row_centers
    ])
    spacing = float(np.median(np.diff(row_centers)))
    if spacing <= 2.0 * profile_resolution:
        raise ValueError(
            f'Detected invalid row spacing: {spacing:.3f} m')

    row_band = min(0.75, 0.30 * spacing)
    row_extents = []
    for row_center in row_centers:
        selected = along[np.abs(cross - row_center) <= row_band]
        if len(selected) < 20:
            row_extents.append((float(along.min()), float(along.max())))
        else:
            low, high = np.quantile(selected, [0.05, 0.95])
            row_extents.append((float(low), float(high)))
    row_extents = np.asarray(row_extents, dtype=np.float64)

    return {
        'angle_deg': best_angle,
        'direction': direction,
        'normal': normal,
        'row_centers': row_centers,
        'row_prominences': prominences,
        'row_extents': row_extents,
        'row_spacing': spacing,
        'orientation_score': best_score,
        'profile': smooth,
    }


def _raster_cells(points_xy, origin, resolution, shape):
    cells = np.floor(
        (points_xy - origin) / resolution).astype(np.int64)
    valid = (
        (cells[:, 0] >= 0)
        & (cells[:, 0] < shape[1])
        & (cells[:, 1] >= 0)
        & (cells[:, 1] < shape[0])
    )
    return cells[valid]


def make_row_corridor_mask(
    points,
    *,
    resolution=0.10,
    robot_radius=0.65,
    safety_margin=0.10,
    obstacle_min_height=0.35,
    obstacle_max_height=0.80,
    min_obstacle_points_per_cell=3,
    ground_band=0.25,
    observation_fill_radius=0.35,
    corridor_half_width=0.55,
    lane_core_half_width=0.45,
    headland_depth=1.25,
    padding=0.75,
    start_location=(-6.0, -8.0),
):
    """Build a connected, footprint-safe mask between vegetation rows."""
    points = np.asarray(points, dtype=np.float64)
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) == 0:
        raise ValueError('Point cloud contains no finite points')
    if resolution <= 0.0:
        raise ValueError('resolution must be positive')

    ground_z = estimate_ground_height(points)
    relative_height = points[:, 2] - ground_z
    vegetation = points[
        (relative_height >= obstacle_min_height)
        & (relative_height <= obstacle_max_height)
    ]
    geometry = estimate_row_geometry(vegetation[:, :2])

    lower = np.quantile(points[:, :2], 0.002, axis=0)
    upper = np.quantile(points[:, :2], 0.998, axis=0)
    start_xy = np.asarray(start_location, dtype=np.float64)
    lower = np.minimum(lower, start_xy - 1.0)
    upper = np.maximum(upper, start_xy + 1.0)
    origin = (
        np.floor((lower - padding) / resolution) * resolution)
    maximum = (
        np.ceil((upper + padding) / resolution) * resolution)
    width, height = (
        np.ceil((maximum - origin) / resolution).astype(int) + 1)
    if width * height > 25_000_000:
        raise ValueError(
            f'Refusing unreasonable grid {width}x{height}; '
            'check PCD bounds')
    shape = (int(height), int(width))

    observed = np.zeros(shape, dtype=bool)
    ground_points = points[
        np.abs(relative_height) <= ground_band]
    cells = _raster_cells(
        ground_points[:, :2], origin, resolution, shape)
    if len(cells):
        observed[cells[:, 1], cells[:, 0]] = True
    observed = binary_dilation(
        observed,
        structure=circular_structure(
            math.ceil(observation_fill_radius / resolution)),
    )

    obstacle_counts = np.zeros(shape, dtype=np.uint16)
    cells = _raster_cells(
        vegetation[:, :2], origin, resolution, shape)
    if len(cells):
        np.add.at(
            obstacle_counts,
            (cells[:, 1], cells[:, 0]),
            1,
        )
    obstacles = (
        obstacle_counts >= int(min_obstacle_points_per_cell))
    inflated = binary_dilation(
        obstacles,
        structure=circular_structure(math.ceil(
            (robot_radius + safety_margin) / resolution)),
    )

    grid_y, grid_x = np.indices(shape, dtype=np.float64)
    world_x = origin[0] + (grid_x + 0.5) * resolution
    world_y = origin[1] + (grid_y + 0.5) * resolution
    direction = geometry['direction']
    normal = geometry['normal']
    along_grid = world_x * direction[0] + world_y * direction[1]
    cross_grid = world_x * normal[0] + world_y * normal[1]

    row_centers = geometry['row_centers']
    corridor_centers = 0.5 * (
        row_centers[:-1] + row_centers[1:])
    row_extents = geometry['row_extents']
    row_start = float(np.median(row_extents[:, 0]))
    row_end = float(np.median(row_extents[:, 1]))
    spacing = geometry['row_spacing']
    corridor_width = min(
        float(corridor_half_width),
        max(0.20, 0.35 * spacing),
    )

    corridor_zone = np.zeros(shape, dtype=bool)
    for center in corridor_centers:
        corridor_zone |= (
            np.abs(cross_grid - center) <= corridor_width)
    corridor_zone &= (
        (along_grid >= row_start - headland_depth)
        & (along_grid <= row_end + headland_depth)
    )
    # The UAV cloud contains low canopy and accumulated depth streaks that
    # can cross an otherwise driveable aisle. They are useful for locating
    # the vegetation rows but must not split the semantic lane into isolated
    # Nav2 islands. Preserve a continuous Panther-width core along each
    # detected inter-row aisle. Live 2-D LiDAR, the local costmap, and the
    # independent collision monitor still mark or stop for any real object.
    lane_core_width = min(
        float(lane_core_half_width),
        corridor_width,
    )
    lane_core = np.zeros(shape, dtype=bool)
    for center in corridor_centers:
        lane_core |= (
            np.abs(cross_grid - center) <= lane_core_width)
    lane_core &= (
        (along_grid >= row_start)
        & (along_grid <= row_end)
    )

    start_along = float(start_xy @ direction)
    start_cross = float(start_xy @ normal)
    cross_low = min(
        float(row_centers[0] - 0.75 * spacing),
        start_cross - 0.75,
    )
    cross_high = max(
        float(row_centers[-1] + 0.75 * spacing),
        start_cross + 0.75,
    )
    headland_zone = (
        (
            (along_grid <= row_start + headland_depth)
            | (along_grid >= row_end - headland_depth)
        )
        & (cross_grid >= cross_low)
        & (cross_grid <= cross_high)
    )
    # Include a narrow approach from the parked Panther to the nearest
    # headland. A downward/forward UAV sensor often has no returns directly
    # under the takeoff / UGV parking area, so requiring observed ground here
    # would leave the robot isolated in unknown space.
    if start_along <= row_start:
        approach_end = row_start + headland_depth
    elif start_along >= row_end:
        approach_end = row_end - headland_depth
    elif abs(start_along - row_start) <= abs(start_along - row_end):
        approach_end = row_start + headland_depth
    else:
        approach_end = row_end - headland_depth
    approach_low = min(start_along, approach_end) - 0.75
    approach_high = max(start_along, approach_end) + 0.75
    approach_zone = (
        (along_grid >= approach_low)
        & (along_grid <= approach_high)
        & (np.abs(cross_grid - start_cross) <= 0.75)
    )

    spawn_x = (start_xy[0] - origin[0]) / resolution
    spawn_y = (start_xy[1] - origin[1]) / resolution
    spawn_clearance = (
        robot_radius + safety_margin) / resolution
    spawn_area = (
        (grid_x - spawn_x) ** 2
        + (grid_y - spawn_y) ** 2
    ) <= spawn_clearance ** 2
    inflated[spawn_area] = False
    observed[spawn_area] = True

    route_support = observed | approach_zone | spawn_area
    candidate = (
        route_support
        & (corridor_zone | headland_zone | approach_zone)
        & (~inflated | lane_core)
    )
    candidate[[0, -1], :] = False
    candidate[:, [0, -1]] = False
    start_cell = (
        int(round(spawn_y)),
        int(round(spawn_x)),
    )
    reachable, selected_start = reachable_component(
        candidate, start_cell)

    centerlines = np.zeros(shape, dtype=bool)
    for center in corridor_centers:
        centerlines |= (
            (np.abs(cross_grid - center) <= 0.55 * resolution)
            & (along_grid >= row_start)
            & (along_grid <= row_end)
        )
    centerlines &= reachable

    geometry.update({
        'ground_z': ground_z,
        'corridor_centers': corridor_centers,
        'corridor_half_width': corridor_width,
        'lane_core_half_width': lane_core_width,
        'row_start': row_start,
        'row_end': row_end,
        'start_along': start_along,
        'start_cross': start_cross,
    })
    return (
        reachable,
        inflated,
        observed,
        centerlines,
        origin,
        selected_start,
        geometry,
    )


def _write_mask_pcd(path, xyz, colors):
    rgb = packed_rgb_float(colors)
    with Path(path).open('w', encoding='ascii') as stream:
        stream.write(
            '# .PCD v0.7 - Point Cloud Data file format\n'
            '# Green: Panther corridor; red: inflated vegetation; '
            'blue: corridor centerline\n'
            'VERSION 0.7\n'
            'FIELDS x y z rgb\n'
            'SIZE 4 4 4 4\n'
            'TYPE F F F F\n'
            'COUNT 1 1 1 1\n'
            f'WIDTH {len(xyz)}\n'
            'HEIGHT 1\n'
            'VIEWPOINT 0 0 0 1 0 0 0\n'
            f'POINTS {len(xyz)}\n'
            'DATA ascii\n')
        for (x, y, z), color in zip(xyz, rgb):
            stream.write(
                f'{x:.7g} {y:.7g} {z:.7g} {color:.9g}\n')


def _world_from_row_coordinates(along, cross, direction, normal):
    return along * direction + cross * normal


def save_row_corridor_products(
    points,
    output_base,
    *,
    resolution=0.10,
    source_name='',
    **kwargs,
):
    """Save a colored 3D overlay and Nav2 row-corridor occupancy map."""
    (
        reachable,
        occupied,
        observed,
        centerlines,
        origin,
        start,
        geometry,
    ) = make_row_corridor_mask(
        points, resolution=resolution, **kwargs)

    image = np.zeros((*reachable.shape, 3), dtype=np.uint8)
    image[observed] = [65, 65, 65]
    image[occupied & observed] = [225, 45, 45]
    image[reachable] = [35, 220, 75]
    image[centerlines] = [40, 100, 255]
    wall_cells = orchard_boundary_wall_mask(
        reachable.shape, origin, resolution)
    image[wall_cells] = [245, 170, 20]
    if start is not None:
        start_y, start_x = start
        image[
            max(0, start_y - 2):start_y + 3,
            max(0, start_x - 2):start_x + 3,
        ] = [255, 210, 40]

    output_base = Path(output_base)
    output_base.parent.mkdir(parents=True, exist_ok=True)
    ppm_path = output_base.with_suffix('.ppm')
    with ppm_path.open('wb') as stream:
        stream.write(
            f'P6\n{image.shape[1]} {image.shape[0]}\n255\n'.encode(
                'ascii'))
        stream.write(np.flipud(image).tobytes())

    nav_image = np.full(reachable.shape, 128, dtype=np.uint8)
    nav_image[occupied] = 0
    nav_image[reachable] = 254
    nav_image[wall_cells] = 0
    nav_pgm, nav_yaml = save_map(
        np.flipud(nav_image),
        (float(origin[0]), float(origin[1])),
        output_base.with_name(f'{output_base.name}_nav'),
        resolution,
    )

    overlay = reachable | (occupied & observed)
    cell_y, cell_x = np.nonzero(overlay)
    xyz = np.column_stack((
        origin[0] + (cell_x + 0.5) * resolution,
        origin[1] + (cell_y + 0.5) * resolution,
        np.full(
            len(cell_x),
            geometry['ground_z'] + 0.08,
            dtype=np.float64,
        ),
    ))
    pcd_path = output_base.with_suffix('.pcd')
    _write_mask_pcd(
        pcd_path, xyz, image[cell_y, cell_x])

    csv_path = output_base.with_name(
        f'{output_base.name}_centerlines.csv')
    direction = geometry['direction']
    normal = geometry['normal']
    grid_y, grid_x = np.indices(reachable.shape, dtype=np.float64)
    world_x = origin[0] + (grid_x + 0.5) * resolution
    world_y = origin[1] + (grid_y + 0.5) * resolution
    along_grid = (
        world_x * direction[0] + world_y * direction[1])
    cross_grid = world_x * normal[0] + world_y * normal[1]
    accessible_corridors = 0
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'corridor_id',
            'start_x', 'start_y',
            'end_x', 'end_y',
            'nominal_width_m',
        ])
        for index, center in enumerate(
            geometry['corridor_centers'], start=1
        ):
            corridor_cells = (
                reachable
                & (
                    np.abs(cross_grid - center)
                    <= geometry['corridor_half_width'])
                & (along_grid >= geometry['row_start'])
                & (along_grid <= geometry['row_end'])
            )
            if np.count_nonzero(corridor_cells) < 10:
                continue
            accessible_corridors += 1
            first_along, last_along = np.quantile(
                along_grid[corridor_cells], [0.01, 0.99])
            first = _world_from_row_coordinates(
                first_along, center, direction, normal)
            last = _world_from_row_coordinates(
                last_along, center, direction, normal)
            writer.writerow([
                index,
                f'{first[0]:.3f}', f'{first[1]:.3f}',
                f'{last[0]:.3f}', f'{last[1]:.3f}',
                f'{2.0 * geometry["corridor_half_width"]:.3f}',
            ])

    metadata_path = output_base.with_name(
        f'{output_base.name}_metadata.yaml')
    row_values = ', '.join(
        f'{value:.3f}' for value in geometry['row_centers'])
    corridor_values = ', '.join(
        f'{value:.3f}'
        for value in geometry['corridor_centers'])
    metadata_path.write_text(
        f'source: "{source_name}"\n'
        f'ground_z: {geometry["ground_z"]:.4f}\n'
        f'row_angle_deg: {geometry["angle_deg"]:.3f}\n'
        f'row_spacing_m: {geometry["row_spacing"]:.3f}\n'
        f'row_count: {len(geometry["row_centers"])}\n'
        f'corridor_count: {len(geometry["corridor_centers"])}\n'
        f'accessible_corridor_count: {accessible_corridors}\n'
        f'row_centers_m: [{row_values}]\n'
        f'corridor_centers_m: [{corridor_values}]\n'
        f'corridor_half_width_m: '
        f'{geometry["corridor_half_width"]:.3f}\n'
        f'lane_core_half_width_m: '
        f'{geometry["lane_core_half_width"]:.3f}\n'
        f'resolution: {resolution:.3f}\n'
        f'reachable_cells: {int(np.count_nonzero(reachable))}\n',
        encoding='utf-8',
    )
    return {
        'pcd': pcd_path,
        'ppm': ppm_path,
        'nav_pgm': nav_pgm,
        'nav_yaml': nav_yaml,
        'centerlines_csv': csv_path,
        'metadata_yaml': metadata_path,
        'reachable_cells': int(np.count_nonzero(reachable)),
        'row_count': int(len(geometry['row_centers'])),
        'corridor_count': int(len(geometry['corridor_centers'])),
        'accessible_corridor_count': accessible_corridors,
        'row_angle_deg': float(geometry['angle_deg']),
        'row_spacing': float(geometry['row_spacing']),
    }


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Extract Panther-safe paths between vegetation rows from '
            'a Gazebo ground-truth PCD.'))
    parser.add_argument(
        '--input',
        help='ASCII PCD; default is newest ground-truth map')
    parser.add_argument(
        '--map-directory',
        default=str(Path.home() / 'husarion_ws' / 'maps'))
    parser.add_argument(
        '--output-base',
        default=str(
            Path.home() / 'husarion_ws' / 'maps'
            / 'latest_panther_row_corridors'))
    parser.add_argument('--resolution', type=float, default=0.10)
    parser.add_argument('--robot-radius', type=float, default=0.65)
    parser.add_argument('--safety-margin', type=float, default=0.10)
    parser.add_argument(
        '--obstacle-min-height', type=float, default=0.35)
    parser.add_argument(
        '--obstacle-max-height', type=float, default=0.80)
    parser.add_argument(
        '--min-obstacle-points', type=int, default=3)
    parser.add_argument(
        '--corridor-half-width', type=float, default=0.55)
    parser.add_argument(
        '--lane-core-half-width',
        type=float,
        default=0.45,
        help=(
            'Semantic half-width kept continuous through sparse UAV-cloud '
            'canopy artifacts; live LiDAR remains authoritative'))
    args = parser.parse_args()

    source = (
        Path(args.input).expanduser()
        if args.input
        else newest_groundtruth_map(args.map_directory)
    )
    points = read_xyz(source)
    products = save_row_corridor_products(
        points,
        Path(args.output_base).expanduser(),
        resolution=args.resolution,
        robot_radius=args.robot_radius,
        safety_margin=args.safety_margin,
        obstacle_min_height=args.obstacle_min_height,
        obstacle_max_height=args.obstacle_max_height,
        min_obstacle_points_per_cell=args.min_obstacle_points,
        corridor_half_width=args.corridor_half_width,
        lane_core_half_width=args.lane_core_half_width,
        source_name=str(source),
    )
    print(f'Input: {source}')
    print(
        f'Detected {products["row_count"]} vegetation rows and '
        f'{products["corridor_count"]} corridors '
        f'({products["accessible_corridor_count"]} accessible) at '
        f'{products["row_angle_deg"]:.1f} degrees; median spacing '
        f'{products["row_spacing"]:.2f} m.')
    print(
        f'Reachable Panther cells: {products["reachable_cells"]}')
    print(f'3D mask: {products["pcd"]}')
    print(f'Preview: {products["ppm"]}')
    print(f'Nav2 map: {products["nav_yaml"]}')
    print(f'Centerlines: {products["centerlines_csv"]}')


if __name__ == '__main__':
    main()
