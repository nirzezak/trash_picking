import random
from abc import ABC, abstractmethod
from typing import List

import ticker
from configs.trash_configs import TrashConfig
from task_manager import TaskManagerComponent
from trash_generator import TrashGenerator


class SummonComponent(ABC):
    """
    Interface to summon trash
    """
    @abstractmethod
    def step(self):
        """
        Function to call in each simulation step
        """
        pass


class AdvancedRandomSummonComponent(SummonComponent):
    """
    This component summons a random trash every few ticks
    """

    def __init__(self, trash_generator: TrashGenerator, task_manager: TaskManagerComponent, summon_tick: int):
        """
        :param trash_generator: object to generate trash in pybullet
        :param task_manager: object to current task manager (to register spawned trash in it)
        :param summon_tick: how often to summon trash. This would be added to a few random ticks every time
        """
        self.trash_generator = trash_generator
        self.task_manager = task_manager
        self.summon_tick = summon_tick
        self.right_summon_tick = random.randint(0, 2000)
        self.left_summon_tick = random.randint(0, 2000)

    def step(self):
        """
        See SummonComponent
        """
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

    def __init__(self, trash_generator: TrashGenerator, task_manager: TaskManagerComponent, summon_tick: int):
        """
        :param trash_generator: object to generate trash in pybullet
        :param task_manager: object to current task manager (to register spawned trash in it)
        :param summon_tick: how often to summon trash. This would be added to a few random ticks every time
        """
        self.trash_generator = trash_generator
        self.task_manager = task_manager
        self.summon_tick = summon_tick

    def step(self):
        """
        See SummonComponent
        """
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

    def __init__(self, trash_generator: TrashGenerator, task_manager: TaskManagerComponent, summon_tick: int,
                 trash=TrashConfig.MUSTARD):
        """
        :param trash_generator: object to generate trash in pybullet
        :param task_manager: object to current task manager (to register spawned trash in it)
        :param summon_tick: how often to summon trash. This would be added to a few random ticks every time
        :param trash: config the the trash to summon
        """
        self.trash_generator = trash_generator
        self.task_manager = task_manager
        self.summon_tick = summon_tick
        self.trash = trash

    def step(self):
        """
        See SummonComponent
        """
        if ticker.now() % self.summon_tick == 0:
            trash = self.trash_generator.summon_trash(self.trash.value)
            self.task_manager.add_trash(trash)


class FixedAmountSummonComponent(SummonComponent):
    """
    This component summons the same trash
    """

    def __init__(self, trash_generator: TrashGenerator, task_manager: TaskManagerComponent, summon_tick: int,
                 trash=TrashConfig.MUSTARD, amount: int = 2):
        """
        :param trash_generator: object to generate trash in pybullet
        :param task_manager: object to current task manager (to register spawned trash in it)
        :param summon_tick: how often to summon trash. This would be added to a few random ticks every time
        :param trash: config the the trash to summon
        :param amount: how much trash to summon
        """
        self.trash_generator = trash_generator
        self.task_manager = task_manager
        self.summon_tick = summon_tick
        self.trash = trash
        self.amount = amount

    def step(self):
        """
        See SummonComponent
        """
        if self.amount != 0 and ticker.now() % self.summon_tick == 0:
            trash = self.trash_generator.summon_trash(self.trash.value)
            self.task_manager.add_trash(trash)
            self.amount -= 1


class TrashListSummonComponent(SummonComponent):
    """
    This component summons the trash given in trash_lst argument
    """

    def __init__(self, trash_generator: TrashGenerator, task_manager: TaskManagerComponent, summon_tick: int,
                 trash_lst: List, side: List[int] = None):
        """
        :param trash_generator: object to generate trash in pybullet
        :param task_manager: object to current task manager (to register spawned trash in it)
        :param summon_tick: how often to summon trash. This would be added to a few random ticks every time
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
        """
        See SummonComponent
        """
        if self.next_trash_idx < len(self.trash_lst) and ticker.now() % self.summon_tick == 0:
            trash = self.trash_generator.summon_trash(
                self.trash_lst[self.next_trash_idx].signed_value(self.side[self.next_trash_idx]))
            self.task_manager.add_trash(trash)
            self.next_trash_idx += 1
