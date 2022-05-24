import pybullet as p


class Bin(object):
    """
    Class to describe the trash bins
    """
    def __init__(self, location, trash_type, urdf_path=None):
        """
        @param location: Location to spawn the bin
        @param trash_type: The type of trash to be recycled in this bin
        @param urdf_path: path to the URDF file of the bin
        """
        if not urdf_path:
            urdf_path = 'models/bin.urdf'

        self.id = p.loadURDF(urdf_path, location, useFixedBase=True)
        self.location = location
        self.trash_type = trash_type
