from enum import Enum
import trash_types


class TrashConfig(Enum):

    MUSTARD = {
        'path': r'models/YcbMustardBottle/model.urdf',
        'location': [0, -2.5, 0.57],
        'gripping_points': [[0, 0, 1]],
        'trash_type': trash_types.TrashTypes.PLASTIC
    }
    METAL_CAN = {
        'path': r'models/YcbMasterChefCan/model.urdf',
        'location': [0, -2.5, 0.57],
        'gripping_points': [[0.1225, 0.025, 0.75]],
        'trash_type': trash_types.TrashTypes.ELECTRONIC
    }
    PAPER_BOX = {
        'path': r'models/YcbCrackerBox/model.urdf',
        'location': [0, -2.5, 0.57],
        'gripping_points': [[0, 0, 1]],
        'trash_type': trash_types.TrashTypes.PAPER
    }
