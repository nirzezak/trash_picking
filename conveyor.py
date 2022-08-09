from __future__ import annotations

from typing import Optional, List

import pybullet as p

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multiarm_planner.UR5 import UR5


class Conveyor(object):
    def __init__(self, p_simulation, location: List[int], speed: float = 5.0, urdf_path=None,
                 arms: Optional[List[UR5]] = None):
        """
        @param p_simulation: pybullet simulation physics client
        """
        if not urdf_path:
            urdf_path = 'models/conveyor/conveyor.urdf'

        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.p_simulation = p_simulation
        self.id = p_simulation.loadURDF(urdf_path, location, start_orientation, useFixedBase=True)
        self.speed = speed
        self.dont_convey = []
        if arms:
            self.dont_convey = [arm.body_id for arm in arms]
            self.dont_convey += [arm.end_effector.body_id for arm in arms]

    def convey(self):
        contact_points = self.p_simulation.getContactPoints(bodyA=self.id)
        # 2: bodyUniqueIdB
        # 4: linkIndexB
        # 6: positionOnB
        links = [(point[2], point[4], point[6]) for point in contact_points]

        for body_uid, link_index, _ in links:
            if body_uid in self.dont_convey:
                continue
            linear_velocity, _ = self.p_simulation.getBaseVelocity(body_uid)
            _, _, vz = linear_velocity
            linear_velocity = (0, self.speed, vz)
            self.p_simulation.resetBaseVelocity(body_uid, linear_velocity, (0, 0, 0))
            self.p_simulation.changeDynamics(body_uid, link_index, lateralFriction=0)

    def unconvey(self, uid):
        # TODO: Can new trash have the same ID as old removed trash?
        #   Either way, should probably remove the ID from this list when removing trash for less spam
        if uid not in self.dont_convey:
            self.dont_convey.append(uid)
