from typing import List

from trash_types import TrashTypes


class Bin(object):
    """
    Class to describe the trash bins
    """
    def __init__(self, p_simulation, location: List[int], trash_type: TrashTypes, urdf_path=None):
        """
        @param p_simulation: pybullet simulation physics client
        @param location: Location to spawn the bin
        @param trash_type: The type of trash to be recycled in this bin
        @param urdf_path: path to the URDF file of the bin
        """
        if not urdf_path:
            urdf_path = 'models/bin.urdf'

        self.id = p_simulation.loadURDF(urdf_path, location, useFixedBase=True)
        self.location = location
        self.trash_type = trash_type
        self.p_simulation = p_simulation

        self._change_color()

    def _change_color(self):
        """
        Change the color of the trash bin, according to its type.
        """
        if self.trash_type == TrashTypes.PLASTIC:
            rgba = [1.0, 0.6, 0.0, 1.0]
        elif self.trash_type == TrashTypes.PAPER:
            rgba = [0.0, 0.0, 1.0, 1.0]
        elif self.trash_type == TrashTypes.ELECTRONIC:
            rgba = [1.0, 1.0, 1.0, 1.0]
        else:
            raise ValueError("Couldn't find color for trash type")

        self.p_simulation.changeVisualShape(self.id, -1, rgbaColor=rgba)

    def is_inside_trash_bin(self, location):
        """
        Checks if the given location is inside the trash bin
        """

        x_diff = abs(location[0] - self.location[0])
        y_diff = abs(location[1] - self.location[1])

        return x_diff < 0.205 and y_diff < 0.29 and location[2] < 0.3

