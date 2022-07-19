from trash import Trash

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

    def summon_trash(self, trash_config, forced_location=None):
        config = trash_config.value
        if forced_location is not None:
            config['location'] = forced_location
        new_trash = Trash(self.p_simulation, **config)
        self.trash.append(new_trash)
        return new_trash

    def remove_trash(self, trash_uid=None):
        """"
        If trash_uid=None, removes all trash
        Else, removes trash with id = trash_uid
        """
        for i, trash_obj in enumerate(self.trash):
            if trash_obj.get_id() == trash_uid or trash_uid is None:
                self.trash.pop(i)
                self.p_simulation.removeBody(trash_obj.get_id())
                return
