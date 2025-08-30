import os, json, numpy as np, genesis as gs
from pathlib import Path

OUT = Path("data/sim_rgbd_v1"); OUT.mkdir(parents=True, exist_ok=True)

def save_png(path, arr):  # quick helper (use imageio if you prefer)
    import imageio.v3 as iio
    if arr.dtype != np.uint8:  # rgb float [0..1] -> uint8
        arr = np.clip(arr*255,0,255).astype(np.uint8)
    iio.imwrite(path, arr)

def main():
    gs.init(backend=gs.gpu)
    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    red = scene.add_entity(
        gs.morphs.Box(size=(0.06,0.08,0.06), pos=(0.30,0.00,0.03)),
        surface=gs.surfaces.Plastic(color=(1,0.1,0.1,1), smooth=False),
    )
    arm = scene.add_entity(gs.morphs.URDF(file="assets/meca500/meca.urdf", fixed=True))
    ee  = arm.get_link("meca_axis_6_link")
    cam = scene.add_camera(res=(1280,720), pos=(0.50,-0.40,0.50),
                           lookat=(0.30,0.00,0.03), up=(0,0,1), fov=58.0, GUI=False)
    scene.build()

    # write intrinsics
    from math import tan, radians
    def intr(width,height,fov_v):
        fy = 0.5*height / tan(radians(fov_v/2)); fx = fy*(width/height)
        cx = (width-1)/2; cy = (height-1)/2
        return [[fx,0,cx],[0,fy,cy],[0,0,1]]
    K = intr(1280,720,58.0)
    (OUT/"intrinsics.json").write_text(json.dumps({"K":K}, indent=2))

    # generate 20 collision-free viewpoints around the red box
    import numpy.random as npr
    n, saved = 0, 0
    while saved < 20 and n < 200:
        n += 1
        # sample azimuth[-60..60], elev[10..60], radius[0.18..0.30]
        az  = np.deg2rad(npr.uniform(-60, 60))
        el  = np.deg2rad(npr.uniform(10, 60))
        r   = npr.uniform(0.18, 0.30)
        box = np.array([0.30,0.00,0.03])
        dirw= np.array([np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)])
        pos = box + r*dirw
        quat= np.array([1,0,0,0])  # keep orientation simple for data collection

        q_goal = arm.inverse_kinematics(link=ee, pos=pos, quat=quat)
        path   = arm.plan_path(qpos_goal=q_goal, num_waypoints=200)
        if path is None:  # planner failed
            continue
        # execute
        for wp in path:
            arm.control_dofs_position(wp); scene.step()
        for _ in range(30): scene.step()

        rgb, depth, seg, _ = cam.render(depth=True, segmentation=True, normal=False)
        base = OUT/f"frame_{saved:04d}"
        save_png(str(base)+"_rgb.png", rgb)
        np.save(str(base)+"_depth.npy", depth.astype(np.float32))
        np.save(str(base)+"_joints.npy", (arm.get_qpos()).cpu().numpy())
        (OUT/f"frame_{saved:04d}_meta.json").write_text(json.dumps({
            "ee_pos": ee.get_pos().tolist(),
            "ee_quat": ee.get_quat().tolist(),
            "cam": {"pos": cam.pos.tolist(), "lookat": cam.lookat.tolist(), "up": cam.up.tolist()},
            "box_pos": red.get_pos().tolist(),
        }, indent=2))
        saved += 1

if __name__ == "__main__":
    main()
