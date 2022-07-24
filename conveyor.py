from typing import Optional

import pybullet as p

from multiarm_planner.UR5 import UR5


class Conveyor(object):
    def __init__(self, p_simulation, location: list[int], speed: float = 5.0, urdf_path=None,
                 arms: Optional[list[UR5]] = None):
        """
        @param p_simulation: pybullet simulation physics client
        """
        if not urdf_path:
            urdf_path = 'models/conveyor/conveyor.urdf'

        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.p_simulation = p_simulation
        self.id = p_simulation.loadURDF(urdf_path, location, start_orientation, useFixedBase=True)
        self.speed = speed
        self.arms_ids = []
        if arms:
            self.arms_ids = [arm.body_id for arm in arms]
            self.arms_ids += [arm.end_effector.body_id for arm in arms]

    def convey(self):
        contact_points = self.p_simulation.getContactPoints(bodyA=self.id)
        # 2: bodyUniqueIdB
        # 4: linkIndexB
        # 6: positionOnB
        links = [(point[2], point[4], point[6]) for point in contact_points]

        for body_uid, link_index, _ in links:
            if body_uid in self.arms_ids:
                continue
            linear_velocity, angular_velocity = self.p_simulation.getBaseVelocity(body_uid)
            _, _, vz = linear_velocity
            linear_velocity = (0, self.speed, vz)
            self.p_simulation.resetBaseVelocity(body_uid, linear_velocity, angular_velocity)
            self.p_simulation.changeDynamics(body_uid, link_index, lateralFriction=0, anisotropicFriction=0)
