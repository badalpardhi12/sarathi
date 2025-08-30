import numpy as np
import genesis as gs

gs.init(backend=gs.gpu)

# viewer enabled
scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=0.01),
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(2.5, -1.5, 1.5),
        camera_lookat=(0.0, 0.0, 0.3),
        max_FPS=120,
    ),
    vis_options=gs.options.VisOptions(show_world_frame=True),
    renderer=gs.renderers.Rasterizer(),
    show_viewer=True,
)

# Headless mode
# scene = gs.Scene(show_viewer=False)
# cam = scene.add_camera(res=(1280,720), fov=58.0, GUI=False, debug=True)

# Ground & target object (red)
scene.add_entity(gs.morphs.Plane())
red_box = scene.add_entity(
    gs.morphs.Box(size=(0.06,0.08,0.06), pos=(0.30,0.00,0.03)),
    surface=gs.surfaces.Plastic(color=(1.0,0.1,0.1,1.0), smooth=False),
)

# Robot (fixed base)
arm = scene.add_entity(gs.morphs.URDF(file='assets/meca500/meca.urdf', fixed=True))
EE_LINK_NAME = 'meca_axis_6_link'   # change if needed
ee_link = arm.get_link(EE_LINK_NAME)

# D435i-like camera (pinhole), headless render
RES = (1280, 720)      # width, height
FOV_V = 58.0           # vertical FoV degrees (D435i depth ~58°)
cam = scene.add_camera(
    res=RES,
    pos=(0.50, -0.40, 0.50),
    lookat=(0.30, 0.00, 0.03),
    up=(0.0, 0.0, 1.0),
    fov=FOV_V,
    GUI=False,
)

scene.build()

# Optional: small settle
for _ in range(5):
    scene.step()

# Render a frame
rgb, depth, seg, normal = cam.render(depth=True, segmentation=True, normal=True)

# Example: move EE above the box with IK + collision-aware plan (needs OMPL installed)
target_pos = np.array([0.30, 0.00, 0.20])
target_quat = np.array([0, 1, 0, 0])    # world-x 180°, same as your test
q_goal = arm.inverse_kinematics(link=ee_link, pos=target_pos, quat=target_quat)

path = arm.plan_path(qpos_goal=q_goal, num_waypoints=200)  # ~2s trajectory
for wp in path:
    arm.control_dofs_position(wp)
    scene.step()

for _ in range(100):
    scene.step()

# Grab another RGBD after motion
rgb2, depth2, seg2, _ = cam.render(depth=True, segmentation=True, normal=False)
