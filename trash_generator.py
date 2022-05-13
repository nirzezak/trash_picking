import pybullet as p
from trash import Trash


class TrashGenerator(object):
    def __init__(self, interval, conveyor_dims, conveyor_location):
        self.interval = interval
        self.conveyor_dims = conveyor_dims
        self.conveyor_location = conveyor_location
        self.trash = []

    def summon_trash(self, config):
        self.trash.append(Trash(**config))

    def remove_trash(self, trash_uid):
        for i, trash_obj in enumerate(self.trash):
            if trash_obj.id == trash_uid:
                self.trash.pop(i)
                p.removeBody(trash_uid)
                return
