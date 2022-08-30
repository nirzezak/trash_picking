from enum import Enum
import trash_types
# Warning: Do not set to something bigger than -3.5,
# because it will cause a lot of lost trash
TRASH_INIT_Y_VAL = -5.4


class TrashConfig(Enum):

    MUSTARD = {
        'path': r'models/YcbMustardBottle/model.urdf',
        'location': [-0.21, TRASH_INIT_Y_VAL, 0.585],
        'gripping_points': [[0.02, 0.05, 0.175]],
        'trash_type': trash_types.TrashTypes.PLASTIC
    }
    METAL_CAN = {
        'path': r'models/YcbMasterChefCan/model.urdf',
        'location': [-0.2, TRASH_INIT_Y_VAL, 0.55],
        'gripping_points': [[0.02, 0.05, 0.092]],
        'trash_type': trash_types.TrashTypes.ELECTRONIC
    }
    PAPER_BOX = {
        'path': r'models/YcbCrackerBox/model.urdf',
        'location': [-0.21, TRASH_INIT_Y_VAL, 0.61],
        'gripping_points': [[-0.04, 0.05, 0.2]],
        'trash_type': trash_types.TrashTypes.PAPER,
        'mirrored_gripping_points': [[-0.035, 0.05, 0.2]]
    }

    def signed_value(self, sign):
        """
        @param sign: the new sign of the trash config

        Returns the original/mirrored (on X-axis) trash config if sign is 1/-1 respectively
        """

        signed_value = self.value.copy()

        if sign == -1 and 'mirrored_location' in signed_value:
            signed_value['location'] = signed_value['mirrored_location'].copy()

        else:
            signed_value['location'] = signed_value['location'].copy()
            signed_value['location'][0] *= sign

        if sign == -1 and 'mirrored_gripping_points' in signed_value:
            signed_value['gripping_points'] = signed_value['mirrored_gripping_points'].copy()
            signed_value['gripping_points'][0] = signed_value['mirrored_gripping_points'][0].copy()

        else:
            signed_value['gripping_points'] = signed_value['gripping_points'].copy()
            signed_value['gripping_points'][0] = signed_value['gripping_points'][0].copy()
            signed_value['gripping_points'][0][0] *= sign

        return signed_value
