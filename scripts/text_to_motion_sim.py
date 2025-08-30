# scripts/text_to_motion_sim.py
import argparse, json
import numpy as np, genesis as gs
import sys, os

# add workspace directory to sys.path
workspace_path = "/home/badal/Documents/sarathi"
sys.path.append(workspace_path)

from skills.text_to_pose import parse_instruction, target_from_spherical
from control.execute_sim import make_scene, move_to_pose_with_plan
from common.frames import intrinsics_from_vertical_fov
from perception.resolve_object import resolve_object_ref

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--text", required=True, help="Natural language command")
    ap.add_argument("--viewer", action="store_true")
    args = ap.parse_args()

    gs.init(backend=gs.gpu)
    scene, arm, ee, box, cam = make_scene(viewer=args.viewer)

    # resolve object (MVP: known box position from scene)
    objs = {"red box": lambda: np.array(box.get_pos().cpu(), dtype=float)}
    obj_pos = resolve_object_ref(args.text, objs)

    # parse instruction -> spherical target
    az, roll, el, dist = parse_instruction(args.text)
    pos, quat_wxyz = target_from_spherical(obj_pos, az, el, dist, roll)

    ok = move_to_pose_with_plan(scene, arm, "meca_axis_6_link", pos, quat_wxyz)
    # Capture post-move RGB-D for verification
    # rgb, depth, seg, _ = cam.render(depth=True, segmentation=True, normal=False)

    # Save a quick JSON result for debugging
    K = intrinsics_from_vertical_fov(1280, 720, 58.0).tolist()
    out = {
        "ok": bool(ok),
        "target_world": {"pos": pos.tolist(), "quat_wxyz": quat_wxyz.tolist()},
        "ee_after": {"pos": ee.get_pos().tolist(), "quat": ee.get_quat().tolist()},
        "cam": {"pos": cam.pos.tolist(), "lookat": cam.lookat.tolist(), "up": cam.up.tolist()},
        "K": K,
    }
    print(json.dumps(out, indent=2))

if __name__ == "__main__":
    main()
