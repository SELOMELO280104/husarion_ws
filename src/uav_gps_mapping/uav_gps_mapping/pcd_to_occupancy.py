"""Convert the newest PX4 GPS point cloud into a Nav2 occupancy map."""

import argparse
from collections import deque
from pathlib import Path

import numpy as np


def pcd_data_header(path):
    """Return (encoding, first data-line index) without decoding binary data."""
    with Path(path).open('rb') as stream:
        for index in range(100):
            line = stream.readline()
            if not line:
                break
            if line.startswith(b'DATA '):
                try:
                    encoding = line.decode('ascii').strip().split()[1]
                except (UnicodeDecodeError, IndexError) as error:
                    raise ValueError(
                        f'{path} has an invalid PCD DATA header') from error
                return encoding, index + 1
    raise ValueError(f'{path} has no PCD DATA header')


def is_ascii_pcd(path):
    """Return true only for a readable ASCII PCD."""
    try:
        encoding, _data_line = pcd_data_header(path)
        return encoding == 'ascii'
    except (OSError, ValueError):
        return False


def newest_gps_map(map_directory):
    """Return the newest nonempty ASCII GPS PCD."""
    candidates = [
        path for path in Path(map_directory).glob('uav_gps_map_*.pcd')
        if path.stat().st_size > 1024 and is_ascii_pcd(path)
    ]
    if not candidates:
        raise FileNotFoundError(
            f'No uav_gps_map_*.pcd found in {map_directory}')
    return max(candidates, key=lambda path: path.stat().st_mtime)


def read_xyz(path):
    """Read XYZ columns from an ASCII PCD file."""
    encoding, data_line = pcd_data_header(path)
    if encoding != 'ascii':
        raise ValueError(
            f'Only ASCII PCD files are supported, got DATA {encoding}')
    points = np.loadtxt(path, skiprows=data_line, usecols=(0, 1, 2))
    points = np.atleast_2d(points)
    return points[np.all(np.isfinite(points), axis=1)]


def dilate(mask, radius_cells):
    """Binary dilation using a circular footprint."""
    if radius_cells <= 0:
        return mask
    result = mask.copy()
    height, width = mask.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            source_y0 = max(0, -dy)
            source_y1 = min(height, height - dy)
            source_x0 = max(0, -dx)
            source_x1 = min(width, width - dx)
            target_y0 = source_y0 + dy
            target_y1 = source_y1 + dy
            target_x0 = source_x0 + dx
            target_x1 = source_x1 + dx
            result[target_y0:target_y1, target_x0:target_x1] |= (
                mask[source_y0:source_y1, source_x0:source_x1])
    return result


def reachable_component(free, start_cell):
    """Return the 8-connected free region reachable from a grid cell."""
    reachable = np.zeros_like(free, dtype=bool)
    free_cells = np.argwhere(free)
    if len(free_cells) == 0:
        return reachable, None
    start_y, start_x = start_cell
    if (
        start_y < 0 or start_y >= free.shape[0]
        or start_x < 0 or start_x >= free.shape[1]
        or not free[start_y, start_x]
    ):
        distances = (
            (free_cells[:, 0] - start_y) ** 2
            + (free_cells[:, 1] - start_x) ** 2)
        start_y, start_x = free_cells[np.argmin(distances)]
    queue = deque([(int(start_y), int(start_x))])
    reachable[start_y, start_x] = True
    while queue:
        cell_y, cell_x = queue.popleft()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                next_y = cell_y + dy
                next_x = cell_x + dx
                if (
                    0 <= next_y < free.shape[0]
                    and 0 <= next_x < free.shape[1]
                    and free[next_y, next_x]
                    and not reachable[next_y, next_x]
                ):
                    reachable[next_y, next_x] = True
                    queue.append((next_y, next_x))
    return reachable, (int(start_y), int(start_x))


def make_traversability(
    points,
    resolution=0.10,
    ground_max_z=0.30,
    obstacle_min_z=0.35,
    obstacle_max_z=0.65,
    min_obstacle_points_per_cell=4,
    robot_radius=0.65,
    safety_margin=0.10,
    observation_fill_radius=0.30,
    padding=1.0,
    start_location=(-6.0, -8.0),
):
    """Build reachable free-space and footprint-inflated obstacle masks."""
    if len(points) == 0:
        raise ValueError('Point cloud contains no finite points')
    minimum = np.floor(
        (points[:, :2].min(axis=0) - padding) / resolution) * resolution
    maximum = np.ceil(
        (points[:, :2].max(axis=0) + padding) / resolution) * resolution
    width, height = np.ceil(
        (maximum - minimum) / resolution).astype(int) + 1
    if width * height > 25_000_000:
        raise ValueError(
            f'Refusing unreasonable grid {width}x{height}; check PCD bounds')

    def cells_for(selected_points):
        return np.floor(
            (selected_points[:, :2] - minimum) / resolution).astype(int)

    observed = np.zeros((height, width), dtype=bool)
    ground = points[points[:, 2] <= ground_max_z]
    if len(ground):
        cells = cells_for(ground)
        observed[cells[:, 1], cells[:, 0]] = True
    observed = dilate(
        observed,
        int(np.ceil(observation_fill_radius / resolution)))

    obstacle_counts = np.zeros((height, width), dtype=np.uint16)
    obstacle_points = points[
        (points[:, 2] >= obstacle_min_z)
        & (points[:, 2] <= obstacle_max_z)
    ]
    if len(obstacle_points):
        cells = cells_for(obstacle_points)
        np.add.at(
            obstacle_counts,
            (cells[:, 1], cells[:, 0]),
            1,
        )
    obstacles = obstacle_counts >= min_obstacle_points_per_cell
    inflated = dilate(
        obstacles,
        int(np.ceil((robot_radius + safety_margin) / resolution)))

    # Do not bake the parked robot into the mask at its known spawn.
    cell_y, cell_x = np.ogrid[:height, :width]
    spawn_x = (start_location[0] - minimum[0]) / resolution
    spawn_y = (start_location[1] - minimum[1]) / resolution
    spawn_clearance = (robot_radius + safety_margin) / resolution
    spawn_area = (
        (cell_x - spawn_x) ** 2 + (cell_y - spawn_y) ** 2
    ) <= spawn_clearance ** 2
    inflated[spawn_area] = False
    observed[spawn_area] = True

    free = observed & ~inflated
    free[[0, -1], :] = False
    free[:, [0, -1]] = False
    start_cell = (
        int(round(spawn_y)),
        int(round(spawn_x)),
    )
    reachable, selected_start = reachable_component(free, start_cell)
    return reachable, inflated, observed, minimum, selected_start


def packed_rgb_float(colors):
    """Pack uint8 RGB triplets into PCL's FLOAT32 rgb representation."""
    packed = (
        colors[:, 0].astype(np.uint32) << 16
        | colors[:, 1].astype(np.uint32) << 8
        | colors[:, 2].astype(np.uint32))
    return packed.view(np.float32)


def save_traversability_products(
    points,
    output_base,
    resolution=0.10,
    **kwargs,
):
    """Write a top-down PPM and colored PCD robot traversability overlay."""
    reachable, occupied, observed, origin, start = make_traversability(
        points, resolution=resolution, **kwargs)
    image = np.zeros((*reachable.shape, 3), dtype=np.uint8)
    image[observed] = [65, 65, 65]
    image[occupied] = [220, 45, 45]
    image[reachable] = [35, 220, 75]
    if start is not None:
        start_y, start_x = start
        image[
            max(0, start_y - 2):start_y + 3,
            max(0, start_x - 2):start_x + 3,
        ] = [40, 100, 255]

    output_base = Path(output_base)
    output_base.parent.mkdir(parents=True, exist_ok=True)
    ppm_path = output_base.with_suffix('.ppm')
    display_image = np.flipud(image)
    with ppm_path.open('wb') as stream:
        stream.write(
            f'P6\n{image.shape[1]} {image.shape[0]}\n255\n'.encode('ascii'))
        stream.write(display_image.tobytes())

    # A second black/free/unknown image is directly consumable by Nav2.
    # Unknown space remains blocked; only the connected green region is free.
    nav_image = np.full(reachable.shape, 128, dtype=np.uint8)
    nav_image[occupied] = 0
    nav_image[reachable] = 254
    save_map(
        np.flipud(nav_image),
        (float(origin[0]), float(origin[1])),
        output_base.with_name(f'{output_base.name}_nav'),
        resolution,
    )

    mask = reachable | occupied
    cell_y, cell_x = np.nonzero(mask)
    xyz = np.column_stack((
        origin[0] + (cell_x + 0.5) * resolution,
        origin[1] + (cell_y + 0.5) * resolution,
        np.full(len(cell_x), 0.08),
    ))
    colors = image[cell_y, cell_x]
    rgb = packed_rgb_float(colors)
    pcd_path = output_base.with_suffix('.pcd')
    with pcd_path.open('w', encoding='ascii') as stream:
        stream.write(
            '# .PCD v0.7 - Point Cloud Data file format\n'
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
    return pcd_path, ppm_path, int(np.count_nonzero(reachable))


def make_occupancy(
    points,
    resolution=0.10,
    obstacle_min_z=0.35,
    obstacle_max_z=1.20,
    padding=2.0,
    inflation_radius=0.15,
    min_points_per_cell=3,
    clear_locations=((-6.0, -8.0),),
    clearance_radius=2.5,
):
    """Rasterize obstacle-height points and return image plus map origin."""
    if len(points) == 0:
        raise ValueError('Point cloud contains no finite points')
    minimum = np.floor(
        (points[:, :2].min(axis=0) - padding) / resolution) * resolution
    maximum = np.ceil(
        (points[:, :2].max(axis=0) + padding) / resolution) * resolution
    width, height = np.ceil(
        (maximum - minimum) / resolution).astype(int) + 1
    if width * height > 100_000_000:
        raise ValueError(
            f'Refusing unreasonable grid {width}x{height}; check PCD bounds')

    obstacle_points = points[
        (points[:, 2] >= obstacle_min_z)
        & (points[:, 2] <= obstacle_max_z)
    ]
    counts = np.zeros((height, width), dtype=np.uint16)
    if len(obstacle_points):
        cells = np.floor(
            (obstacle_points[:, :2] - minimum) / resolution).astype(int)
        np.add.at(counts, (cells[:, 1], cells[:, 0]), 1)
    occupied = counts >= min_points_per_cell
    occupied = dilate(
        occupied, int(np.ceil(inflation_radius / resolution)))

    # The UAV survey can see the parked Panther and otherwise bake it into
    # the static map. Remove its known spawn footprint so Nav2 does not start
    # with the robot trapped inside an obstacle.
    if clearance_radius > 0:
        cell_y, cell_x = np.ogrid[:height, :width]
        radius_cells = clearance_radius / resolution
        for location_x, location_y in clear_locations:
            center_x = (location_x - minimum[0]) / resolution
            center_y = (location_y - minimum[1]) / resolution
            clear = (
                (cell_x - center_x) ** 2
                + (cell_y - center_y) ** 2
            ) <= radius_cells ** 2
            occupied[clear] = False

    # PGM values used by Nav2: 0 occupied, 254 free. Keep a hard boundary so
    # the planner cannot leave the surveyed orchard footprint.
    image = np.full((height, width), 254, dtype=np.uint8)
    image[occupied] = 0
    border = max(1, int(np.ceil(0.35 / resolution)))
    image[:border, :] = 0
    image[-border:, :] = 0
    image[:, :border] = 0
    image[:, -border:] = 0
    return np.flipud(image), (float(minimum[0]), float(minimum[1]))


def save_map(image, origin, output_base, resolution):
    """Write binary PGM and Nav2 YAML files."""
    output_base = Path(output_base)
    output_base.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_base.with_suffix('.pgm')
    yaml_path = output_base.with_suffix('.yaml')
    with pgm_path.open('wb') as stream:
        stream.write(
            f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode('ascii'))
        stream.write(image.tobytes())
    yaml_path.write_text(
        f'image: {pgm_path.name}\n'
        f'resolution: {resolution}\n'
        f'origin: [{origin[0]}, {origin[1]}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
        'mode: trinary\n',
        encoding='utf-8',
    )
    return pgm_path, yaml_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', help='ASCII PCD; default is newest GPS map')
    parser.add_argument(
        '--map-directory',
        default=str(Path.home() / 'husarion_ws' / 'maps'))
    parser.add_argument(
        '--output-base',
        default=str(
            Path.home() / 'husarion_ws' / 'maps'
            / 'latest_orchard_nav'))
    parser.add_argument('--resolution', type=float, default=0.10)
    parser.add_argument('--obstacle-min-z', type=float, default=0.35)
    parser.add_argument('--obstacle-max-z', type=float, default=1.20)
    parser.add_argument('--inflation-radius', type=float, default=0.15)
    parser.add_argument(
        '--traversability-resolution', type=float, default=0.10)
    parser.add_argument('--robot-radius', type=float, default=0.65)
    parser.add_argument('--safety-margin', type=float, default=0.10)
    parser.add_argument(
        '--min-obstacle-points', type=int, default=4,
        help='minimum returns in a grid cell before footprint inflation')
    args = parser.parse_args()

    source = (
        Path(args.input).expanduser()
        if args.input else newest_gps_map(args.map_directory))
    points = read_xyz(source)
    image, origin = make_occupancy(
        points,
        resolution=args.resolution,
        obstacle_min_z=args.obstacle_min_z,
        obstacle_max_z=args.obstacle_max_z,
        inflation_radius=args.inflation_radius,
    )
    pgm_path, yaml_path = save_map(
        image, origin, args.output_base, args.resolution)
    occupied = int(np.count_nonzero(image == 0))
    print(f'Input: {source}')
    print(
        f'Map: {image.shape[1]}x{image.shape[0]}, '
        f'occupied cells: {occupied}')
    print(f'PGM: {pgm_path}')
    print(f'YAML: {yaml_path}')
    mask_pcd, mask_ppm, reachable_cells = save_traversability_products(
        points,
        Path(args.map_directory) / 'latest_panther_traversability',
        resolution=args.traversability_resolution,
        robot_radius=args.robot_radius,
        safety_margin=args.safety_margin,
        min_obstacle_points_per_cell=args.min_obstacle_points,
    )
    print(
        f'Traversability: {reachable_cells} reachable cells, '
        f'PCD: {mask_pcd}, PPM: {mask_ppm}')


if __name__ == '__main__':
    main()
