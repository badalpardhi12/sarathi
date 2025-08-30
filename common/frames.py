# sarathi/common/frames.py
import numpy as np

def lookat_to_extrinsic(pos, lookat, up):
    """Build 4x4 world_T_cam from camera pos/lookat/up (right-handed, +Z up world)."""
    pos = np.asarray(pos, dtype=float)
    tgt = np.asarray(lookat, dtype=float)
    upv = np.asarray(up, dtype=float)

    z = pos - tgt                     # camera forward in world (points from target to camera)
    z /= np.linalg.norm(z)
    x = np.cross(upv, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)

    R = np.stack([x, y, z], axis=1)   # columns are x,y,z
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = pos
    return T

def intrinsics_from_vertical_fov(width, height, fov_v_deg):
    fov = np.deg2rad(fov_v_deg)
    fy = 0.5 * height / np.tan(fov/2.0)
    fx = fy * (width / height)
    cx = (width - 1) * 0.5
    cy = (height - 1) * 0.5
    K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=float)
    return K

def project_points_world_to_pixels(P_world, world_T_cam, K):
    """P_world: (N,3). Returns pixel coords (N,2) and depths (N,)."""
    P = np.concatenate([P_world, np.ones((P_world.shape[0],1))], axis=1).T  # 4xN
    cam_T_world = np.linalg.inv(world_T_cam)
    Pc = cam_T_world @ P
    x, y, z = Pc[0], Pc[1], Pc[2]  # camera coords
    uv = (K @ np.vstack([x/z, y/z, np.ones_like(z)]))[:2]
    return uv.T, z
