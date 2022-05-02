import pybullet as p
import os

UR5_URDF_PATH = os.path.join("models", "ur5", "ur5.urdf")

class Ur5:
    """"
    This class is used for creating UR5 robot and controlling it
    """

    def __init__(self, base_position, base_orientation):
        self.id = p.loadURDF(UR5_URDF_PATH, base_position, base_orientation,
                             flags=p.URDF_USE_SELF_COLLISION, useFixedBase=True)


    def change_position(self, position, orientation):
        # TODO
        pass
        # maybe helpful:
        # orientation = p.getQuaternionFromEuler([3.14, 0., 0.])
        # target_position_joints = p.calculateInverseKinematics(self.id, 4, [0.1, 0.1, 0.4], targetOrientation=orientation)
        # p.setJointMotorControlArray(self.id, range(4), p.POSITION_CONTROL, targetPositions=target_position_joints)




