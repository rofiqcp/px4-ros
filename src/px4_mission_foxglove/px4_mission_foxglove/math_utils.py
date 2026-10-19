"""Pure geodesy and preview-only heading model (not real vehicle controls)."""
import math

NAV_WAYPOINT = 16
GLOBAL_FRAMES = frozenset((0, 3, 5, 6, 10, 11))

def waypoint(item):
    lat, lon = item.x / 1e7, item.y / 1e7
    return (int(item.command) == NAV_WAYPOINT and int(item.frame) in GLOBAL_FRAMES
            and math.isfinite(lat) and math.isfinite(lon)
            and -90 <= lat <= 90 and -180 <= lon <= 180
            and (abs(lat) + abs(lon)) > 1e-8)

def xy(lat, lon, base_lat, base_lon):
    earth = 6371000.0
    return (earth * math.radians(lon-base_lon) * math.cos(math.radians(base_lat)),
            earth * math.radians(lat-base_lat))

def preview(position, target, heading, speed_limit, correction, cruise=.35,
            yaw_limit=.25, heading_kp=.6, arrival=1.5):
    values = (*position, *target, heading, speed_limit, correction,
              cruise, yaw_limit, heading_kp, arrival)
    if not all(math.isfinite(float(v)) for v in values):
        return 0., 0., 'INVALID_NUMERIC_INPUT'
    dx, dy = target[0]-position[0], target[1]-position[1]
    if math.hypot(dx,dy) <= arrival:
        return 0., 0., 'WAYPOINT_REACHED_NEAR'
    if speed_limit <= 0 or cruise <= 0:
        return 0., 0., 'STOP_WALL'
    delta = math.atan2(math.sin(math.atan2(dy,dx)-heading),
                       math.cos(math.atan2(dy,dx)-heading))
    yaw = max(-yaw_limit,min(yaw_limit,heading_kp*delta+correction))
    if abs(delta) > math.radians(70):
        return 0., yaw, 'HEADING_NOT_ALIGNED'
    return min(speed_limit,cruise),yaw,'PREVIEW_ONLY'
