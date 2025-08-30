import numpy as np, genesis as gs
from skills.nl_to_pose import parse_instruction, target_from_spherical

def run_command(text):
    az, roll, el, r = parse_instruction(text)

    gs.init(backend=gs.gpu)
    scene = gs.Scene(show_viewer=True)
    plane = scene.add_entity(gs.morphs.Plane())
    red_box = scene.add_entity(gs.morphs.Box(size=(0.06,0.08,0.06), pos=(0.30,0.00,0.03), rgba=(1,0,0,1)))
    arm = scene.add_entity(gs.morphs.URDF(file='assets/meca500/meca.urdf', fixed=True))
    scene.build()

    ee = arm.get_link('tool0')
    pos, quat = target_from_spherical(red_box.pos, az, el, roll, r)
    q_goal = arm.inverse_kinematics(link=ee, pos=pos, quat=quat)
    path = arm.plan_path(qpos_goal=q_goal, num_waypoints=200)

    for wp in path:
        arm.control_dofs_position(wp)
        scene.step()
    for _ in range(60): scene.step()

if __name__ == "__main__":
    run_command("position the end effector at 30 degree azimuth, 20 degree roll, and 45 degree elevation at 20 cm distance from the red box")
