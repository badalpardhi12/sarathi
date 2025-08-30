# sarathi/control/execute_sim.py
import numpy as np
import time
import genesis as gs

def make_scene(fov_v=58.0, res=(1280,720), viewer=False):
    scene = gs.Scene(show_viewer=viewer)
    plane = scene.add_entity(gs.morphs.Plane())
    box   = scene.add_entity(
        gs.morphs.Box(size=(0.06,0.08,0.06), pos=(0.30,0.00,0.03)),
        surface=gs.surfaces.Plastic(color=(1,0.1,0.1,1), smooth=False),
    )
    arm   = scene.add_entity(gs.morphs.URDF(file="assets/meca500/meca.urdf", fixed=True))
    ee    = arm.get_link("meca_axis_6_link")
    cam   = scene.add_camera(res=res, pos=(0.50,-0.40,0.50),
                             lookat=(0.30,0.00,0.03), up=(0,0,1), fov=fov_v, GUI=False)
    scene.build()
    return scene, arm, ee, box, cam

def move_to_pose_with_plan(scene, arm, ee_link_name, pos, quat_wxyz, n_wp=200):
    ee = arm.get_link(ee_link_name)
    q_goal = arm.inverse_kinematics(link=ee, pos=np.asarray(pos), quat=np.asarray(quat_wxyz))
    path = arm.plan_path(qpos_goal=q_goal, num_waypoints=n_wp)
    if path is None:
        return False
    for wp in path:
        arm.control_dofs_position(wp); scene.step()
    for _ in range(30): scene.step()
    # wait 10 seconds
    time.sleep(10)
    return True
