import numpy as np
import pybullet as p


class Trash(object):
    """
    Class to handle trash objects

    @param path: Path to trash URDF file
    @param gripping_points: Trash gripping points (used for the arms' gripping
    method). The points should be relative to the trash, and not be absolute
    (since the trash object will move)
    @param id: The object's ID as returned by pybullet
    @param trash_size: How many arms are required to lift the object?
    """
    def __init__(self, p_simulation, path=None, location=None, gripping_points=None, trash_type=TrashTypes.PLASTIC):
        """
        @param p_simulation: pybullet simulation physics client
        @param path: Path to the URDF file containing the object
        @param location: Location to load the trash to
        @param gripping_points: The points that we can grip to. If the object
        is light, it should be a list of length 1. If the object is heavy, it
        should be a list of length 2.
        @param trash_type: The type of the trash.
        """
        self.path = path
        self.gripping_points = np.array(gripping_points)
        self.id = p_simulation.loadURDF(self.path, location, useFixedBase=False)
        self.location = location
        self.id = p.loadURDF(self.path, location, useFixedBase=False)
        self.trash_size = len(self.gripping_points)
        self.p_simulation = p_simulation
        self.trash_type = trash_type

    def get_curr_gripping_points(self):
        """
        Calculate the current gripping points, based on the current location,
        in world coordinates.
        """
        new_gripping_points = []
        pos, orientation = self.p_simulation.getBasePositionAndOrientation(self.id)
        # Create the rotation matrix from the orientation
        rotation_matrix = p.getMatrixFromQuaternion(orientation)
        rotation_matrix = np.array(rotation_matrix).reshape((3, 3))
        pos = np.array(pos)
        for point in self.gripping_points:
            # Convert the gripping point to world coordinates
            new_point = pos + rotation_matrix @ point
            new_gripping_points.append(new_point)

        return new_gripping_points


MUSTARD_CONFIG = {
    'path': r'models/YcbMustardBottle/model.urdf',
    'location': [0, 0, 1],
    'gripping_points': [[0, 0, 1]],
}
