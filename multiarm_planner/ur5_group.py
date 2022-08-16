import numpy as np
import pybullet as p


def split(a, n):
    k, m = divmod(len(a), n)
    return [a[i * k + min(i, m):(i + 1) * k + min(i + 1, m)] for i in range(n)]


class UR5Group:
    def __init__(self, ur5_arms, collision_distance=0):
        self.all_controllers = ur5_arms[:]
        self.active_controllers = ur5_arms[:]
        self.collision_distance = collision_distance

    def setup(self, start_poses, start_joints, specific_ur5s=None):
        self.disable_all_ur5s()
        self.enable_ur5s(count=len(start_poses), specific_ur5s=specific_ur5s)
        for c, pose, joints in zip(
                self.active_controllers, start_poses, start_joints):
            c.set_arm_joints(joints)
            # c.set_pose(pose) # I think that set_pose is useless at best and interferes at worst, so I removed it - @NIR
        return None

    def disable_all_ur5s(self):
        for i, ur5 in enumerate(self.all_controllers):
            ur5.disable(idx=i)
        self.active_controllers = []

    def enable_ur5s(self, count=None, specific_ur5s=None):
        self.disable_all_ur5s()
        if specific_ur5s:
            assert count == len(specific_ur5s)
            self.active_controllers = [ur5 for ur5 in self.all_controllers if ur5 in specific_ur5s]
        elif count:
            self.active_controllers = self.all_controllers[:count]
        [ur5.enable() for ur5 in self.active_controllers]

    def set_joint_positions(self, joint_values):
        assert len(joint_values) == self.compute_dof()
        robot_joint_values = split(joint_values, len(self.active_controllers))
        for c, jv in zip(self.active_controllers, robot_joint_values):
            c.set_arm_joints(jv)

    def get_joint_positions(self):
        joint_values = []
        for c in self.active_controllers:
            joint_values += c.get_arm_joint_values()
        return joint_values

    def compute_dof(self):
        return sum([len(c.GROUP_INDEX['arm'])
                    for c in self.active_controllers])

    def difference_fn(self, q1, q2):
        difference = []
        split_q1 = split(q1, len(self.active_controllers))
        split_q2 = split(q2, len(self.active_controllers))
        for ctrl, q1_, q2_ in zip(self.active_controllers, split_q1, split_q2):
            difference += ctrl.arm_difference_fn(q1_, q2_)
        return difference

    def distance_fn(self, q1, q2):
        diff = np.array(self.difference_fn(q2, q1))
        return np.linalg.norm(diff)

    def sample_fn(self):
        values = []
        for ctrl in self.active_controllers:
            values += ctrl.arm_sample_fn()
        return values

    def get_extend_fn(self, resolutions=None):
        dof = self.compute_dof()
        if resolutions is None:
            resolutions = 0.05 * np.ones(dof)

        def fn(q1, q2):
            steps = np.abs(np.divide(self.difference_fn(q2, q1), resolutions))
            num_steps = int(max(steps))
            waypoints = []
            diffs = self.difference_fn(q2, q1)
            for i in range(num_steps):
                waypoints.append(
                    list(((float(i) + 1.0) /
                          float(num_steps)) * np.array(diffs) + q1))
            return waypoints

        return fn

    def get_collision_fn(self, log=False, collision_distance=None):
        collision_distance = self.collision_distance if collision_distance is None else collision_distance

        # Automatically check everything
        def collision_fn(q=None):
            if q is not None:
                self.set_joint_positions(q)
            return any([c.check_collision(
                collision_distance=collision_distance)
                for c in self.active_controllers])

        return collision_fn

    def forward_kinematics(self, q):
        """ return a list of eef poses """
        poses = []
        split_q = split(q, len(self.active_controllers))
        for ctrl, q_ in zip(self.active_controllers, split_q):
            poses.append(ctrl.forward_kinematics(q_))
        return poses
