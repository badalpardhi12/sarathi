import numpy as np
import genesis as gs
from genesis import morphs, surfaces

gs.init(backend=gs.gpu)

scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=0.01),
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(2.5, -1.5, 1.5), camera_lookat=(0.0, 0.0, 0.3), max_FPS=120
    ),
    show_viewer=True,
)

plane = scene.add_entity(gs.morphs.Plane())
red_box = scene.add_entity(
    morphs.Box(size=(0.06, 0.08, 0.06), pos=(0.30, 0.00, 0.03)),
    surface=surfaces.Plastic(color=(1.0, 0.0, 0.0, 1.0)),  # RGBA red
)

arm = scene.add_entity(gs.morphs.URDF(file='assets/meca500/meca.urdf', fixed=True))

scene.build()

# Replace 'tool0' with the end-effector link name in your URDF
ee_link = arm.get_link('meca_axis_6_link')  

# Example: move above the red box using IK then plan a collision-checked path
target_pos = np.array([0.30, 0.00, 0.20])     # 20 cm above the box
target_quat = np.array([0,1,0,0])             # world-x 180° (example)
q_goal = arm.inverse_kinematics(link=ee_link, pos=target_pos, quat=target_quat)

# Smooth, collision-aware trajectory to q_goal
path = arm.plan_path(qpos_goal=q_goal, num_waypoints=200)
for wp in path:
    arm.control_dofs_position(wp)
    scene.step()

# Let the controller settle
for _ in range(100):
    scene.step()

# building more complex trajectories

target_pos_list = [
    np.array([0.30, 0.00, 0.20]),  # 20 cm above the box
    np.array([0.35, 0.05, 0.25]),  # another position
    np.array([0.40, 0.00, 0.30]),  # another position
]

for target_pos in target_pos_list:
    target_quat = np.array([0,1,0,0])  # world-x 180° (example)
    q_goal = arm.inverse_kinematics(link=ee_link, pos=target_pos, quat=target_quat)

    # Smooth, collision-aware trajectory to q_goal
    path = arm.plan_path(qpos_goal=q_goal, num_waypoints=200)
    for wp in path:
        arm.control_dofs_position(wp)
        scene.step()

    # Let the controller settle
    for _ in range(100):
        scene.step()