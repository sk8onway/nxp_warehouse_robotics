"""
map_utils.py

Utility functions for converting between map (grid) coordinates and
world (continuous) coordinates using ROS OccupancyGrid metadata.

These functions are commonly used in SLAM, navigation, and perception
pipelines where sensor/map data is grid-based but robot motion and goals
are defined in continuous world space.
"""

from typing import Tuple
from nav_msgs.msg import MapMetaData


def get_map_conversion_info(map_info: MapMetaData) -> Tuple[float, float, float]:
    """
    Extracts resolution and origin from map metadata.

    Args:
        map_info (MapMetaData): Metadata of the OccupancyGrid map.

    Returns:
        Tuple containing:
        - resolution (float): meters per grid cell
        - origin_x (float): world x-coordinate of map origin
        - origin_y (float): world y-coordinate of map origin
    """
    resolution = map_info.resolution
    origin_x = map_info.origin.position.x
    origin_y = map_info.origin.position.y
    return resolution, origin_x, origin_y


def get_world_coord_from_map_coord(
    map_x: int,
    map_y: int,
    map_info: MapMetaData
) -> Tuple[float, float]:
    """
    Converts map (grid) coordinates to world coordinates.

    Map coordinates are discrete (cell indices), whereas world coordinates
    are continuous (meters). The +0.5 offset converts the cell index to
    the center of the grid cell.

    Args:
        map_x (int): X index in the map grid
        map_y (int): Y index in the map grid
        map_info (MapMetaData): Metadata of the OccupancyGrid map

    Returns:
        Tuple[float, float]: (world_x, world_y) in meters
    """
    if map_info is None:
        return 0.0, 0.0

    resolution, origin_x, origin_y = get_map_conversion_info(map_info)

    world_x = (map_x + 0.5) * resolution + origin_x
    world_y = (map_y + 0.5) * resolution + origin_y

    return world_x, world_y


def get_map_coord_from_world_coord(
    world_x: float,
    world_y: float,
    map_info: MapMetaData
) -> Tuple[int, int]:
    """
    Converts world coordinates to map (grid) coordinates.

    This performs the inverse operation of get_world_coord_from_map_coord()
    by removing the map origin offset and scaling by resolution.

    Args:
        world_x (float): X coordinate in world frame (meters)
        world_y (float): Y coordinate in world frame (meters)
        map_info (MapMetaData): Metadata of the OccupancyGrid map

    Returns:
        Tuple[int, int]: (map_x, map_y) grid indices
    """
    if map_info is None:
        return 0, 0

    resolution, origin_x, origin_y = get_map_conversion_info(map_info)

    map_x = int((world_x - origin_x) / resolution)
    map_y = int((world_y - origin_y) / resolution)

    return map_x, map_y
