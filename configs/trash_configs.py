from enum import Enum
import trash_types


class TrashConfig(Enum):

    MUSTARD = {
        'path': r'models/YcbMustardBottle/model.urdf',
        'location': [-0.2, -2.5, 0.57],
        'gripping_points': [[0.2, 0.11, 0.13]],
        'trash_type': trash_types.TrashTypes.PLASTIC
    }
    METAL_CAN = {
        'path': r'models/YcbMasterChefCan/model.urdf',
        'location': [-0.2, -2.5, 0.57],
        'gripping_points': [[0.2, 0.11, 0]],
        'trash_type': trash_types.TrashTypes.ELECTRONIC
    }
    PAPER_BOX = {
        'path': r'models/YcbCrackerBox/model.urdf',
        'location': [-0.15, -2.5, 0.57],
        'gripping_points': [[0.2, 0.11, 0.175]],
        'trash_type': trash_types.TrashTypes.PAPER
    }
