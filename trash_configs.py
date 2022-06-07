from enum import Enum


class TrashConfig(Enum):

    MUSTARD = {
        'path': r'models/YcbMustardBottle/model.urdf',
        'location': [-0.15, -0.4, 0.6],
        'gripping_points': [[0, 0, 1]],
    }
