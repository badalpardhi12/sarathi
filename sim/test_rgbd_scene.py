import numpy as np
import genesis as gs
from genesis import morphs, surfaces, options

gs.init(backend=gs.gpu)

scene = gs.Scene(
    sim_options=options.SimOptions(dt=0.01),
    viewer_options=options.ViewerOptions(
        camera_pos=(2.5, -1.5, 1.5), camera_lookat=(0.0, 0.0, 0.3), max_FPS=120
    ),
    vis_options=options.VisOptions(segmentation_level='link'),  # seg by link
    show_viewer=True,
)

plane = scene.add_entity(morphs.Plane())
red_box = scene.add_entity(
    morphs.Box(size=(0.06, 0.08, 0.06), pos=(0.30, 0.00, 0.03)),
    surface=surfaces.Plastic(color=(1.0, 0.0, 0.0, 1.0)),
)

arm = scene.add_entity(morphs.URDF(file='assets/meca500/meca.urdf', fixed=True))

# D435i-like pinhole RGB-D camera (vertical FOV 58°, near/far in meters)
cam = scene.add_camera(
    model='pinhole',
    res=(1280, 720),
    pos=(0.65, -0.35, 0.45),             # pick a vantage point
    lookat=(0.30,  0.00, 0.10),
    up=(0.0, 0.0, 1.0),
    fov=58.0,                             # vertical field of view
    GUI=False,
)
# Tighten z range for depth stability
cam.near = 0.10
cam.far  = 5.0

scene.build()

# Quick IK move so the arm isn't singular at start
ee_link = arm.get_link('meca_axis_6_link')  # adjust if needed
q_goal = arm.inverse_kinematics(link=ee_link, pos=np.array([0.35, 0.05, 0.20]))
for wp in arm.plan_path(qpos_goal=q_goal, num_waypoints=200):
    arm.control_dofs_position(wp); scene.step()

# Render one frame: RGB + depth + segmentation
rgb, depth, seg = cam.render(rgb=True, depth=True, segmentation=True)
print('RGB:', rgb.shape, 'Depth:', depth.shape, 'Seg:', seg.shape)

# Keep sim running
while scene.step():
    pass
