from enum import Enum
import trash_types


class TrashConfig(Enum):

    MUSTARD = {
        'path': r'models/YcbMustardBottle/model.urdf',
        'location': [-0.21, -2.5, 0.585],
        'gripping_points': [[0.03, 0.05, 0.175]],
        'trash_type': trash_types.TrashTypes.PLASTIC
    }
    METAL_CAN = {
        'path': r'models/YcbMasterChefCan/model.urdf',
        'location': [-0.2, -2.5, 0.55],
        'gripping_points': [[0.02, 0.05, 0.092]],
        'trash_type': trash_types.TrashTypes.ELECTRONIC
    }
    PAPER_BOX = {
        'path': r'models/YcbCrackerBox/model.urdf',
        'location': [-0.21, -2.5, 0.61],
        'gripping_points': [[-0.04, 0.05, 0.2]],
        'trash_type': trash_types.TrashTypes.PAPER
    }

    def signed_value(self, sign):
        """
        @param sign: the new sign of the trash config

        Returns the original/mirrored (on X-axis) trash config if sign is 1/-1 respectively
        """

        signed_value = self.value.copy()

        signed_value['location'] = signed_value['location'].copy()
        signed_value['gripping_points'] = signed_value['gripping_points'].copy()
        signed_value['gripping_points'][0] = signed_value['gripping_points'][0].copy()

        signed_value['location'][0] *= sign
        signed_value['gripping_points'][0][0] *= sign

        return signed_value
