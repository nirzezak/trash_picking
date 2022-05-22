import pybullet as p
from trash import Trash, TrashType
from utils import add_element_wise

TRASH_BASE_OFFSET_FROM_CONV = [0, 0, 0.5]

class TrashGenerator(object):
    def __init__(self, p_simulation, interval, conveyor_dims, conveyor_location):
        """
        @param p_simulation: pybullet simulation physics client
        """
        self.interval = interval
        self.conveyor_dims = conveyor_dims
        self.conveyor_location = conveyor_location
        self.trash = []
        self.p_simulation = p_simulation

    def summon_trash(self, trash_type: TrashType):
        config = trash_type.value
        config['location'] = add_element_wise(self.conveyor_location, TRASH_BASE_OFFSET_FROM_CONV)
        new_trash = Trash(p_simulation=self.p_simulation, **config)
        self.trash.append(new_trash)
        return new_trash

    def remove_trash(self, trash_uid):
        for i, trash_obj in enumerate(self.trash):
            if trash_obj.id == trash_uid:
                self.trash.pop(i)
                self.p_simulation.removeBody(trash_uid)
                return
