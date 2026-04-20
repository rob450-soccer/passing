from abc import ABC, abstractmethod
from typing import override
from mujococodebase.world.field_landmarks import FieldLandmarks


class Field(ABC):
    def __init__(self, world):
        from mujococodebase.world.world import World  # type hinting
        self.world: World = world
        self.field_landmarks: FieldLandmarks = FieldLandmarks(world=self.world)

    def get_our_goal_position(self):
        return (-self.get_length()/2, 0)

    def get_their_goal_position(self):
        return (self.get_length()/2, 0)

    @abstractmethod
    def get_goal_width(self) -> float:
        """Mouth width of the goal (m), along the field Y axis; used when vision landmarks are not yet visible."""
        raise NotImplementedError()

    @abstractmethod
    def get_width(self):
        raise NotImplementedError()

    @abstractmethod
    def get_length(self):
        raise NotImplementedError()
    

class FIFAField(Field):
    def __init__(self, world):
        super().__init__(world)

    @override
    def get_goal_width(self) -> float:
        return 7.32

    @override
    def get_width(self):
        return 68

    @override
    def get_length(self):
        return 105
    

class HLAdultField(Field):
    def __init__(self, world):
        super().__init__(world)

    @override
    def get_goal_width(self) -> float:
        # Matches rcssservermj HL adult soccer field goal_dim[1] (m).
        return 2.6

    @override
    def get_width(self):
        return 9

    @override
    def get_length(self):
        return 14