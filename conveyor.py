import pybullet as p


class Conveyor(object):
    def __init__(self, location, speed=5, urdf_path=None):
        if not urdf_path:
            urdf_path = 'models/conveyor/conveyor.urdf'

        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.id = p.loadURDF(urdf_path, location, start_orientation, useFixedBase=True)
        self.speed = speed

    def convey(self):
        contact_points = p.getContactPoints(bodyA=self.id)
        # 2: bodyUniqueIdB
        # 4: linkIndexB
        # 6: positionOnB
        links = [(point[2], point[4], point[6]) for point in contact_points]

        for body_uid, link_index, _ in links:
            linear_velocity, angular_velocity = p.getBaseVelocity(body_uid)
            _, _, vz = linear_velocity
            linear_velocity = (0, self.speed, vz)
            p.resetBaseVelocity(body_uid, linear_velocity, angular_velocity)
            p.changeDynamics(body_uid, link_index, lateralFriction=0, anisotropicFriction=0)
