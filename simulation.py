import os
import time
import pybullet as p
import pybullet_data
from multiarm_planner.UR5 import UR5

URDF_FILES_PATH = "models"

CONVEYOR_LOCATION = [0, 0, 0.5]
BINS_LOCATIONS = [[1.5, 0.0, 0.1], [-1.5, 0.0, 0.1]]
UR5_LOCATIONS = [([1, 0, 1], [0, 0, 1, 0]), ([-1, 0, 1], [0, 0, 1, 0])]
FRAME_RATE = 1 / 240.


def create_conveyor():
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    return p.loadURDF('models/conveyor/conveyor.urdf', CONVEYOR_LOCATION, start_orientation, useFixedBase=True)


def create_arms():
    return [UR5(ur5_loc) for ur5_loc in UR5_LOCATIONS]


def create_bins():
    return [p.loadURDF(os.path.join(URDF_FILES_PATH, "bin.urdf"), bin_loc, flags=p.URDF_USE_INERTIA_FROM_FILE,
                       useFixedBase=True) for bin_loc in BINS_LOCATIONS]


def create_env():
    ids = {}
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)

    ids['plane'] = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
    ids['bins'] = create_bins()
    ids['arms'] = create_arms()
    ids['conveyor'] = create_conveyor()

    return ids


if __name__ == '__main__':
    create_env()
    for _ in range(10000):
        p.stepSimulation()
        time.sleep(FRAME_RATE)
    # ur5_arms[0].end_effector.close()
    # ur5_arms[0].end_effector.open()
