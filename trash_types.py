from enum import Enum, auto


class TrashTypes(Enum):
    """
    Different types of trash objects
    """
    PLASTIC = auto()
    PAPER = auto()
    ELECTRONIC = auto()
