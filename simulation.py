import os
import time
import pybullet as p
import pybullet_data
from Ur5 import Ur5

URDF_FILES_PATH = "models"

BINS_LOCATIONS = [[1.5, 0.0, 0.1], [-1.5, 0.0, 0.1]]
UR5_LOCATIONS = [([1, 0, 1], [0, 0, 1, 0]), ([-1, 0, 1], [0, 0, 1, 0])]


# Open GUI and set pybullet_data in the path
p.connect(p.GUI)
# p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
p.setTimeStep(1 / 240.)

# Load plane contained in pybullet_data
plane_id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

bins_ids = [p.loadURDF(os.path.join(URDF_FILES_PATH, "bin.urdf"), bin_loc, flags=p.URDF_USE_INERTIA_FROM_FILE,
                   useFixedBase=True) for bin_loc in BINS_LOCATIONS]

ur5_arms: list[Ur5] = [Ur5(*ur5_loc) for ur5_loc in UR5_LOCATIONS]

trash_id = p.loadURDF(os.path.join(URDF_FILES_PATH, 'YcbMustardBottle', "model.urdf"), [0., 0., 0.],
                      flags=p.URDF_USE_INERTIA_FROM_FILE)

p.setGravity(0, 0, -9.8)


if __name__ == '__main__':
    while True:
        p.stepSimulation()
        time.sleep(1./240)