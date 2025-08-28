import mecademicpy.robot as mdr
import numpy as np
from transforms3d.euler import euler2deg

class Meca500:
    def __init__(self, ip):
        self.r = mdr.Robot()

    def connect(self, ip, sync=True):
        self.r.Connect(address=ip, enable_synchronous_mode=sync)
        self.r.ActivateAndHome()
        self.r.WaitHomed()

    def move_joints_deg(self, jdeg):
        self.r.MoveJoints(*jdeg)
        self.r.WaitIdle()

    def move_pose_mobileXYZ(self, xyz, alpha_beta_gamma_deg):
        x,y,z = xyz
        a,b,c = alpha_beta_gamma_deg
        self.r.MovePose(x, y, z, a, b, c)
        self.r.WaitIdle()

    def stop(self):
        self.r.WaitIdle()
        self.r.DeactivateRobot()
        self.r.Disconnect()
