from typing import List

import numpy as np
import pybullet as p
import random

from trash_types import TrashTypes


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
    def __init__(self, p_simulation, path=None, location=None, gripping_points=None, trash_type=TrashTypes.PLASTIC, orientation=None, **kwargs):
        """
        @param p_simulation: pybullet simulation physics client
        @param path: Path to the URDF file containing the object
        @param location: Location to load the trash to
        @param gripping_points: The points that we can grip to. If the object
        is light, it should be a list of length 1. If the object is heavy, it
        should be a list of length 2.
        @param trash_type: The type of the trash.
        """
        # Assign random orientation to trash
        # Note: change the way orientation assignment works as you please - the important code for handling
        # changed orientation (which is the gripping point calculation using rotation matrix) is already implemented
        orientation = [0, 0, random.choice([0, np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 3, -np.pi / 3])] if orientation is None else orientation

        self.path = path
        self.gripping_points = np.array(gripping_points)
        self.id = p_simulation.loadURDF(self.path, location, p_simulation.getQuaternionFromEuler(orientation), useFixedBase=False)
        self.trash_size = len(self.gripping_points)
        self.p_simulation = p_simulation
        self.trash_type = trash_type

        self.trash_config = {
            'path': self.path,
            'location': location,
            'gripping_points': self.gripping_points,
            'orientation': orientation
        }

    def get_curr_gripping_points(self) -> List[List[int]]:
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

    def get_curr_position(self) -> List[int]:
        """
        Get current position of trash object (x,y,z only, not orientation)
        """
        return self.p_simulation.getBasePositionAndOrientation(self.id)[0]

    def get_id(self) -> int:
        """
        Get pybullet ID of this trash object
        """
        return self.id

    def get_trash_config_at_loc(self, location):
        """
        Get the trash configuration, but at the given location
        """
        conf = self.trash_config.copy()
        conf['location'] = location
        return conf

    def reset_friction(self):
        """
        Give the trash object its default friction back
        """
        self.p_simulation.changeDynamics(self.get_id(), -1, lateralFriction=0.8)
