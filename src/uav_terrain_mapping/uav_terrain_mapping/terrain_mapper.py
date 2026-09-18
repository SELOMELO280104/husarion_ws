"""Build a slope-following 2D map from a georeferenced UAV point cloud."""

import argparse
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, median_filter


def newest_cloud(directory):
    candidates = [
        path for path in Path(directory).glob('uav_gps_map_*.pcd')
        if path.stat().st_size > 1024
    ]
    if not candidates:
        raise FileNotFoundError(f'No UAV GPS PCD found in {directory}')
    return max(candidates, key=lambda path: path.stat().st_mtime)


def read_xyz(path):
    skip = None
    with Path(path).open('r', encoding='ascii') as stream:
        for index, line in enumerate(stream):
            if line.startswith('DATA '):
                if line.strip() != 'DATA ascii':
                    raise ValueError('Only ASCII PCD input is supported')
                skip = index + 1
                break
    if skip is None:
        raise ValueError('PCD DATA header is missing')
    points = np.atleast_2d(np.loadtxt(path, skiprows=skip, usecols=(0, 1, 2)))
    return points[np.isfinite(points).all(axis=1)]


def _cell_percentile(values, keys, size, percentile):
    """Calculate one percentile per occupied flattened grid cell."""
    order = np.argsort(keys)
    sorted_keys = keys[order]
    sorted_values = values[order]
    unique, starts = np.unique(sorted_keys, return_index=True)
    ends = np.r_[starts[1:], len(sorted_keys)]
    result = np.full(size, np.nan, dtype=np.float32)
    for key, start, end in zip(unique, starts, ends):
        result[key] = np.percentile(sorted_values[start:end], percentile)
    return result


def build_terrain_map(
    points,
    resolution=0.15,
    padding=2.0,
    obstacle_height=0.40,
    min_obstacle_points=3,
    obstacle_inflation=0.15,
):
    """Return a PGM image using height above locally estimated ground."""
    if not len(points):
        raise ValueError('Point cloud is empty')

    minimum = np.floor(
        (points[:, :2].min(axis=0) - padding) / resolution) * resolution
    maximum = np.ceil(
        (points[:, :2].max(axis=0) + padding) / resolution) * resolution
    width, height = (
        np.ceil((maximum - minimum) / resolution).astype(int) + 1
    )
    cells = np.floor((points[:, :2] - minimum) / resolution).astype(int)
    cells[:, 0] = np.clip(cells[:, 0], 0, width - 1)
    cells[:, 1] = np.clip(cells[:, 1], 0, height - 1)
    keys = cells[:, 1] * width + cells[:, 0]

    counts = np.bincount(keys, minlength=width * height).reshape(height, width)
    observed = counts > 0
    low_surface = _cell_percentile(
        points[:, 2], keys, width * height, 10.0
    ).reshape(height, width)

    # Fill cells with no direct return from the nearest surveyed cell and
    # smooth only the ground model. Absolute hills remain ground; isolated
    # vertical objects do not raise the reference surface.
    missing = ~np.isfinite(low_surface)
    nearest = distance_transform_edt(
        missing, return_distances=False, return_indices=True)
    ground = low_surface[tuple(nearest)]
    ground = median_filter(ground, size=5, mode='nearest')

    point_ground = ground[cells[:, 1], cells[:, 0]]
    elevated = points[:, 2] > point_ground + obstacle_height
    elevated_counts = np.bincount(
        keys[elevated], minlength=width * height).reshape(height, width)
    obstacles = elevated_counts >= min_obstacle_points

    inflate_cells = int(np.ceil(obstacle_inflation / resolution))
    if inflate_cells:
        yy, xx = np.ogrid[
            -inflate_cells:inflate_cells + 1,
            -inflate_cells:inflate_cells + 1]
        footprint = xx * xx + yy * yy <= inflate_cells * inflate_cells
        obstacles = binary_dilation(obstacles, structure=footprint)

    # Close small scan-line gaps, but retain large unsurveyed regions as
    # unknown instead of falsely declaring the whole bounding box free.
    survey = binary_dilation(observed, iterations=2)
    image = np.full((height, width), 205, dtype=np.uint8)
    image[survey] = 254
    image[obstacles & survey] = 0
    border = max(1, int(np.ceil(0.30 / resolution)))
    image[:border, :] = 0
    image[-border:, :] = 0
    image[:, :border] = 0
    image[:, -border:] = 0
    return np.flipud(image), (float(minimum[0]), float(minimum[1])), ground


def save_map(image, origin, output_base, resolution):
    base = Path(output_base)
    base.parent.mkdir(parents=True, exist_ok=True)
    pgm = base.with_suffix('.pgm')
    yaml = base.with_suffix('.yaml')
    with pgm.open('wb') as stream:
        stream.write(f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode())
        stream.write(image.tobytes())
    yaml.write_text(
        f'image: {pgm.name}\n'
        f'resolution: {resolution}\n'
        f'origin: [{origin[0]}, {origin[1]}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
        'mode: trinary\n',
        encoding='utf-8',
    )
    return pgm, yaml


def main():
    default_maps = Path.home() / 'husarion_ws' / 'maps'
    parser = argparse.ArgumentParser()
    parser.add_argument('--input')
    parser.add_argument('--map-directory', default=str(default_maps))
    parser.add_argument(
        '--output-base', default=str(default_maps / 'terrain_nav_v2'))
    parser.add_argument('--resolution', type=float, default=0.15)
    parser.add_argument('--obstacle-height', type=float, default=0.40)
    args = parser.parse_args()

    source = (
        Path(args.input).expanduser()
        if args.input else newest_cloud(args.map_directory))
    points = read_xyz(source)
    image, origin, ground = build_terrain_map(
        points,
        resolution=args.resolution,
        obstacle_height=args.obstacle_height,
    )
    pgm, yaml = save_map(image, origin, args.output_base, args.resolution)
    print(f'Input: {source}')
    print(f'Points: {len(points)}')
    print(
        f'Ground elevation: {float(np.nanmin(ground)):.2f} to '
        f'{float(np.nanmax(ground)):.2f} m')
    print(
        f'Cells: free={np.count_nonzero(image == 254)}, '
        f'occupied={np.count_nonzero(image == 0)}, '
        f'unknown={np.count_nonzero(image == 205)}')
    print(f'Experimental PGM: {pgm}')
    print(f'Experimental YAML: {yaml}')


if __name__ == '__main__':
    main()
