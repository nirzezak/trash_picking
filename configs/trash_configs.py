from enum import Enum
import trash_types


class TrashConfig(Enum):

    MUSTARD = {
        'path': r'models/YcbMustardBottle/model.urdf',
        'location': [-0.2, -2.5, 0.61],
        'gripping_points': [[0.02, 0.05, 0.175]],
        'trash_type': trash_types.TrashTypes.PLASTIC
    }
    METAL_CAN = {
        'path': r'models/YcbMasterChefCan/model.urdf',
        'location': [-0.2, -2.5, 0.61],
        'gripping_points': [[0.02, 0.05, 0.092]],
        'trash_type': trash_types.TrashTypes.ELECTRONIC
    }
    PAPER_BOX = {
        'path': r'models/YcbCrackerBox/model.urdf',
        'location': [-0.21, -2.5, 0.61],
        'gripping_points': [[-0.04, 0.05, 0.195]],
        'trash_type': trash_types.TrashTypes.PAPER
    }
