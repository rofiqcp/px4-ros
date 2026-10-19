#!/usr/bin/env python3
"""Pure planner unit cases; no ROS, MAVLink or hardware."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from waypoint_planner import ROUTES, plan_route, route_length_m


def invalid(lat, lon, points, why):
    try:
        plan_route(lat, lon, points)
    except ValueError:
        print("PLANNER_REJECT_PASS", why)
        return
    raise AssertionError("Planner must reject: " + why)


def main():
    count = 0
    for route, offsets in ROUTES.items():
        result = plan_route(47.397971, 8.546164, offsets)
        assert len(result) == len(offsets)
        assert route_length_m(offsets) > 0
        assert all(abs(p.lat_e7) < 900000000 for p in result)
        assert all(abs(p.lon_e7) < 1800000000 for p in result)
        assert result[-1].north_m == offsets[-1][0]
        print("PLANNER_VALID_PASS", route, len(result),
              round(route_length_m(offsets), 2))
        count += 1
    invalid(47, 8, ((0, 0), (0, 0)), "duplicate")
    invalid(47, 8, ((1, 1), (1.1, 1.1)), "adjacent_too_close")
    invalid(47, 8, ((1, 1), (31, 1)), "too_far")
    invalid(47, 8, ((1, 1), (12, 0)), "too_long_segment")
    invalid(47, 8, ((math.nan, 1), (2, 2)), "nan")
    invalid(47, 8, ((1, math.inf), (2, 2)), "infinite")
    invalid(95, 8, ROUTES["line2"], "bad_latitude")
    invalid(47, 181, ROUTES["line2"], "bad_longitude")
    invalid(89, 8, ROUTES["line2"], "near_pole")
    invalid(47, 8, ((1, 0),), "one_waypoint")
    invalid(47, 8, tuple((float(i), 0) for i in range(22)), "too_many")
    invalid(47, 179.99999, ((1, 2), (2, 4)), "longitude_wrap")
    print("WAYPOINT_PLANNER_UNIT_PASS", count, "valid and 12 invalid test cases")


if __name__ == "__main__":
    main()
