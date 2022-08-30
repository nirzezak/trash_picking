import random
from abc import ABC, abstractmethod

import ticker
from configs.trash_configs import TrashConfig


class SummonComponent(ABC):
    @abstractmethod
    def step(self):
        pass


class AdvancedRandomSummonComponent(SummonComponent):
    """
    This component summons a random trash every few ticks
    """

    def __init__(self, trash_generator, task_manager, summon_tick):
        self.trash_generator = trash_generator
        self.task_manager = task_manager
        self.summon_tick = summon_tick
        self.right_summon_tick = random.randint(0, 2000)
        self.left_summon_tick = random.randint(0, 2000)

    def step(self):
        if ticker.now() % self.right_summon_tick == 0:
            config = random.choice(list(TrashConfig))
            signed_value = config.signed_value(-1)
            trash = self.trash_generator.summon_trash(signed_value)
            self.task_manager.add_trash(trash)
            self.right_summon_tick = ticker.now() + random.randint(2000, 5000)

        if ticker.now() % self.left_summon_tick == 0:
            config = random.choice(list(TrashConfig))
            signed_value = config.signed_value(1)
            trash = self.trash_generator.summon_trash(signed_value)
            self.task_manager.add_trash(trash)
            self.left_summon_tick = ticker.now() + random.randint(2000, 5000)


class RandomSummonComponent(SummonComponent):
    """
    This component summons a random trash every few ticks
    """

    def __init__(self, trash_generator, task_manager, summon_tick):
        self.trash_generator = trash_generator
        self.task_manager = task_manager
        self.summon_tick = summon_tick

    def step(self):
        if ticker.now() % self.summon_tick == 0:
            config = random.choice(list(TrashConfig))
            sign = random.choice([-1, 1])
            signed_value = config.signed_value(sign)
            trash = self.trash_generator.summon_trash(signed_value)
            self.task_manager.add_trash(trash)


class DeterministicSummonComponent(SummonComponent):
    """
    This component summons the same trash every few ticks
    """

    def __init__(self, trash_generator, task_manager, summon_tick, trash=TrashConfig.MUSTARD):
        self.trash_generator = trash_generator
        self.task_manager = task_manager
        self.summon_tick = summon_tick
        self.trash = trash

    def step(self):
        if ticker.now() % self.summon_tick == 0:
            trash = self.trash_generator.summon_trash(self.trash.value)
            self.task_manager.add_trash(trash)


class FixedAmountSummonComponent(SummonComponent):
    """
    This component summons the same trash
    """

    def __init__(self, trash_generator, task_manager, summon_tick, trash=TrashConfig.MUSTARD, amount=2):
        self.trash_generator = trash_generator
        self.task_manager = task_manager
        self.summon_tick = summon_tick
        self.trash = trash
        self.amount = amount

    def step(self):
        if self.amount != 0 and ticker.now() % self.summon_tick == 0:
            trash = self.trash_generator.summon_trash(self.trash.value)
            self.task_manager.add_trash(trash)
            self.amount -= 1


class TrashListSummonComponent(SummonComponent):
    """
    This component summons the trash given in trash_lst argument
    """

    def __init__(self, trash_generator, task_manager, summon_tick, trash_lst, side=None):
        """
        :param trash_lst: list of trash configs
        :param side: list of 1 or -1, the side of the trash on the conveyor
        if side is None, all trash will be generated on side 1
        Assumes len(trash_lst) = len(side) if side is not None
        """
        self.trash_generator = trash_generator
        self.task_manager = task_manager
        self.summon_tick = summon_tick
        self.trash_lst = trash_lst
        self.next_trash_idx = 0
        self.side = side if side is not None else [1 for _ in range(len(trash_lst))]

    def step(self):
        if self.next_trash_idx < len(self.trash_lst) and ticker.now() % self.summon_tick == 0:
            trash = self.trash_generator.summon_trash(
                self.trash_lst[self.next_trash_idx].signed_value(self.side[self.next_trash_idx]))
            self.task_manager.add_trash(trash)
            self.next_trash_idx += 1
