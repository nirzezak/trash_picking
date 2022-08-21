import random
from abc import ABC, abstractmethod

import ticker
from configs.trash_configs import TrashConfig


class SummonComponent(ABC):
    @abstractmethod
    def step(self):
        pass


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
            trash = self.trash_generator.summon_trash(config.value)
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

    def __init__(self, trash_generator, task_manager, summon_tick, trash_lst):
        self.trash_generator = trash_generator
        self.task_manager = task_manager
        self.summon_tick = summon_tick
        self.trash_lst = trash_lst
        self.next_trash_idx = 0

    def step(self):
        if self.next_trash_idx < len(self.trash_lst) and ticker.now() % self.summon_tick == 0:
            trash = self.trash_generator.summon_trash(self.trash_lst[self.next_trash_idx].value)
            self.task_manager.add_trash(trash)
            self.next_trash_idx += 1
