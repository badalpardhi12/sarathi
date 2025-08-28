import re, math
import numpy as np
from transforms3d.euler import euler2quat

def parse_instruction(text):
    # naive number grabber; good enough for MVP commands like:
    # "position the end effector at 30 degree azimuth, 20 degree roll,
    #  and 45 degree elevation at 20 cm distance from the red box"
    nums = list(map(float, re.findall(r'(-?\d+\.?\d*)', text)))
    az_deg, roll_deg, el_deg, dist_cm = nums[:4]
    return math.radians(az_deg), math.radians(roll_deg), math.radians(el_deg), dist_cm/100.0

def target_from_spherical(box_pos, az, el, roll, r):
    # Spherical around the box in world frame:
    # azimuth about +Z, elevation from XY plane, radius = r
    x = box_pos[0] + r * math.cos(el) * math.cos(az)
    y = box_pos[1] + r * math.cos(el) * math.sin(az)
    z = box_pos[2] + r * math.sin(el)
    pos = np.array([x, y, z])

    # Orient the tool: z-axis points from EE toward the box, with a roll about this approach direction.
    # We'll construct a frame with -z toward the box, then roll about that.
    v = np.array(box_pos) - pos
    v /= np.linalg.norm(v)
    # choose an up vector and build R with Gram-Schmidt (omitted for brevity in MVP)
    # For now, align tool -Z with v and roll around v by 'roll'.
    # Produce an approximate quaternion (improve later).
    # Placeholder: use ZYX Euler (mobile XYZ on robot uses αβγ; we’ll map downstream).
    quat = euler2quat(roll, 0.0, az)  # simple placeholder
    return pos, quat
