"""Validated local-N/E route presets -> MAVLink WGS84 waypoints.

Pure math; intentionally does not import ROS, connect to Cube or command motion.
The stored route is a reference for a *simulator* mission, not obstacle avoidance.
"""
from dataclasses import dataclass
import math

EARTH_M = 6378137.0

ROUTES = {
    "line2": ((1.8, 0.0), (3.6, 0.0)),
    "corner3": ((1.8, 0.0), (1.8, 1.8), (3.6, 1.8)),
    "square4": ((1.8, 0.0), (1.8, 1.8), (0.0, 1.8), (0.0, 0.0)),
    "zigzag5": ((1.6, 0.0), (3.0, 1.3), (4.4, 0.0), (5.8, 1.3), (7.2, 0.0)),
}


@dataclass(frozen=True)
class Waypoint:
    north_m: float
    east_m: float
    lat_e7: int
    lon_e7: int


def plan_route(home_lat: float, home_lon: float, offsets: tuple, *,
               max_radius_m: float = 12.0) -> list[Waypoint]:
    """Reject invalid, duplicate, unsafe-range and degenerate paths before upload."""
    if not (math.isfinite(home_lat) and math.isfinite(home_lon) and
            abs(home_lat) <= 80 and abs(home_lon) <= 180):
        raise ValueError("Invalid or high-latitude WGS84 origin")
    if not (math.isfinite(max_radius_m) and 0.5 <= max_radius_m <= 30):
        raise ValueError("Invalid maximum route radius")
    if not (2 <= len(offsets) <= 20):
        raise ValueError("Expected 2..20 waypoints")
    lat_rad = math.radians(home_lat)
    metres_per_lat_deg = math.pi * EARTH_M / 180.0
    metres_per_lon_deg = metres_per_lat_deg * math.cos(lat_rad)
    points = []
    previous = (0.0, 0.0)
    for offset in offsets:
        if len(offset) != 2:
            raise ValueError("Waypoint must contain north/east in metres")
        north, east = offset
        if not (math.isfinite(north) and math.isfinite(east)):
            raise ValueError("Non-finite waypoint")
        if math.hypot(north, east) > max_radius_m:
            raise ValueError("Waypoint beyond bounded local planning radius")
        segment = math.hypot(north - previous[0], east - previous[1])
        if segment < 0.35:
            raise ValueError("Duplicate or too-close consecutive waypoint")
        if segment > 10.0:
            raise ValueError("Route segment > 10 m")
        lat = home_lat + north / metres_per_lat_deg
        lon = home_lon + east / metres_per_lon_deg
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError("Waypoint outside geographic coordinate range")
        points.append(Waypoint(north, east, round(lat * 1e7), round(lon * 1e7)))
        previous = (north, east)
    return points


def route_length_m(offsets: tuple) -> float:
    """Total planned path length including the initial position."""
    last = (0.0, 0.0)
    length = 0.0
    for target in offsets:
        length += math.dist(last, target)
        last = target
    return length
