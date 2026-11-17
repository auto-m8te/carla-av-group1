"""Geometry of the Gazebo test track.

The track is an oval driven anticlockwise: two 150 m straights joined by two
half circles of radius 60 m.  All numbers here describe the centre line of
OUR lane (the right-hand lane).  The overtaking lane is 3.5 m to its left.

Both the world builder and the lane node use this file, so the road you see
in Gazebo and the lane the car follows can never disagree.
"""
import math

import numpy as np

STRAIGHT = 150.0    # m, length of each straight
RADIUS = 60.0       # m, radius of our lane centre in the bends
LANE_WIDTH = 3.5    # m
LENGTH = 2 * STRAIGHT + 2 * math.pi * RADIUS
STEP = 0.25         # m between stored centre-line points


def pose_at(s):
    """Centre of our lane at distance s along the track: x, y, yaw, curvature."""
    s = s % LENGTH
    half_circle = math.pi * RADIUS
    if s < STRAIGHT:                                   # bottom straight, heading +x
        return s, 0.0, 0.0, 0.0
    s -= STRAIGHT
    if s < half_circle:                                # right-hand bend
        a = -math.pi / 2 + s / RADIUS
        return STRAIGHT + RADIUS * math.cos(a), RADIUS + RADIUS * math.sin(a), a + math.pi / 2, 1.0 / RADIUS
    s -= half_circle
    if s < STRAIGHT:                                   # top straight, heading -x
        return STRAIGHT - s, 2 * RADIUS, math.pi, 0.0
    s -= STRAIGHT                                      # left-hand bend
    a = math.pi / 2 + s / RADIUS
    return RADIUS * math.cos(a), RADIUS + RADIUS * math.sin(a), a + math.pi / 2, 1.0 / RADIUS


def shifted(s, lateral):
    """A point beside our lane centre; lateral > 0 is to the left."""
    x, y, yaw, _ = pose_at(s)
    return x - lateral * math.sin(yaw), y + lateral * math.cos(yaw), yaw


# The whole centre line, sampled once, for fast nearest-point searches.
S = np.arange(0.0, LENGTH, STEP)
XY = np.array([pose_at(s)[:2] for s in S])
