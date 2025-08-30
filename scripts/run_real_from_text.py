import yaml, math
import numpy as np
from transforms3d.euler import mat2euler
from skills.text_to_pose import parse_instruction, target_from_spherical
from robot.meca_iface import Meca500

cfg = yaml.safe_load(open("configs/robot.yaml"))
ip = cfg["robot"]["ip"]

text = "position the end effector at 30 degree azimuth, 20 degree roll, and 45 degree elevation at 20 cm distance from the red box"

# For MVP on real robot, substitute a known box pose in robot base frame:
box_base = np.array([0.30, 0.00, 0.03])
az, roll, el, r = parse_instruction(text)
pos, quat = target_from_spherical(box_base, az, el, roll, r)

# Convert quat → mobile XYZ (α,β,γ). For MVP: compose a world->tool R and extract intrinsic XYZ.
# (Fill in with your preferred convention mapping; test with the robot's live readouts.)
# alpha,beta,gamma = ...

arm = Meca500(ip)
arm.connect(ip)
arm.move_pose_mobileXYZ(pos, (alpha, beta, gamma))
arm.stop()
