import re, math, numpy as np
from scipy.spatial.transform import Rotation as R

NUM = r'(-?\d+(?:\.\d+)?)'
DEG = lambda v: math.radians(float(v))
CM  = lambda v: float(v)/100.0

def parse_instruction(text):
    vals = re.findall(NUM, text.lower())
    if len(vals) < 4:
        raise ValueError("Could not parse 4 numbers (az, roll, elev, dist_cm).")
    az, roll, el, dist_cm = vals[:4]
    return DEG(az), DEG(roll), DEG(el), CM(dist_cm)

def _safe_norm(v):
    n = np.linalg.norm(v)
    if n < 1e-9: raise ValueError("Degenerate direction (norm ~ 0).")
    return v / n

def look_at_with_roll(from_pos, target_pos, roll_deg=0.0):
    from_pos   = np.asarray(from_pos, dtype=float)
    target_pos = np.asarray(target_pos, dtype=float)

    # Forward (world) from camera/EE to target
    fwd = _safe_norm(target_pos - from_pos)

    # We want tool +Z to be z_axis = -fwd (so tool -Z "looks at" target)
    z_axis = -fwd

    # Choose a reference "up" to seed a valid right vector
    ref_up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(z_axis, ref_up)) > 0.99:   # near parallel
        ref_up = np.array([0.0, 1.0, 0.0])

    # Build an orthonormal right-handed basis
    x_axis = _safe_norm(np.cross(ref_up, z_axis))   # right
    y_axis = _safe_norm(np.cross(z_axis, x_axis))   # up (guarantees RH: x × y = z)

    R0 = np.column_stack([x_axis, y_axis, z_axis])  # world rotation, RH by construction

    # Roll about local +Z (tool z)
    Rz_local = R.from_euler('z', roll_deg, degrees=True).as_matrix()
    R_world  = R0 @ Rz_local

    # (Optional) Re-orthonormalize to kill tiny drift and enforce det=+1
    U, _, Vt = np.linalg.svd(R_world)
    R_world = U @ Vt
    if np.linalg.det(R_world) < 0:
        # flip one axis if needed to enforce proper rotation
        U[:,-1] *= -1
        R_world = U @ Vt

    q_xyzw = R.from_matrix(R_world).as_quat()
    q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
    return q_wxyz

def target_from_spherical(obj_pos, az_rad, el_rad, dist_m, roll_deg=0.0):
    obj_pos = np.asarray(obj_pos, dtype=float)
    dirw = np.array([
        math.cos(el_rad)*math.cos(az_rad),
        math.cos(el_rad)*math.sin(az_rad),
        math.sin(el_rad)
    ], dtype=float)
    pos = obj_pos + dist_m * dirw
    quat_wxyz = look_at_with_roll(pos, obj_pos, roll_deg)
    return pos, quat_wxyz
