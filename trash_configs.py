from enum import Enum


class TrashConfig(Enum):

    MUSTARD = {
        'path': r'models/YcbMustardBottle/model.urdf',
        'location': [0, -2.5, 0.57],
        'gripping_points': [[0, 0, 1]],
    }
