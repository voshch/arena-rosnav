

"""Interface definitions for simulator interactions with obstacles, pedestrians, and robots."""

import abc
from collections.abc import Sequence

from arena_people_msgs.msg import Pedestrians
from arena_simulation_setup.shared import Elevator
from task_generator.shared import (
    Door,
    DynamicObstacle,
    Floor,
    Obstacle,
    Robot,
    Wall,
)


class ObstacleITF(abc.ABC):
    """Abstract base class for obstacle management in simulators."""
    @abc.abstractmethod
    def obstacle_spawn(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        """Spawn obstacles."""
        raise NotImplementedError()

    @abc.abstractmethod
    def obstacle_move(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        """Move obstacles."""
        raise NotImplementedError()

    @abc.abstractmethod
    def obstacle_delete(self, obstacles: Sequence[Obstacle]) -> Sequence[bool]:
        """Delete obstacles."""
        raise NotImplementedError()


class PedestrianITF(abc.ABC):
    """Abstract base class for pedestrian management in simulators."""
    @abc.abstractmethod
    def pedestrian_spawn(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        """Spawn pedestrians."""
        raise NotImplementedError()

    @abc.abstractmethod
    def pedestrian_move(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        """Teleport pedestrians."""
        raise NotImplementedError()

    @abc.abstractmethod
    def pedestrian_delete(self, pedestrians: Sequence[DynamicObstacle]) -> Sequence[bool]:
        """Delete pedestrians."""
        raise NotImplementedError()

    @abc.abstractmethod
    def pedestrian_update(self, pedestrians: Pedestrians) -> Sequence[bool]:
        """Navigate pedestrians to position with velocity."""
        raise NotImplementedError()


class RobotITF(abc.ABC):
    """Abstract base class for robot management in simulators."""
    @abc.abstractmethod
    def robot_spawn(self, robots: Sequence[Robot]) -> Sequence[bool]:
        """Spawn robots."""
        raise NotImplementedError()

    @abc.abstractmethod
    def robot_move(self, robots: Sequence[Robot]) -> Sequence[bool]:
        """Move robots."""
        raise NotImplementedError()

    @abc.abstractmethod
    def robot_delete(self, robots: Sequence[Robot]) -> Sequence[bool]:
        """Delete robots."""
        raise NotImplementedError()


class WorldITF(abc.ABC):
    @abc.abstractmethod
    def spawn_walls(self, walls: list[Wall]) -> bool:
        """
        Add a list of walls to the simulator.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def spawn_floors(self, floors: list[Floor]) -> bool:
        """
        Add a list of floors to the simulator.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def spawn_doors(self, doors: list[Door]) -> bool:
        """
        Add a list of doors to the simulator.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def spawn_elevators(self, elevators: list[Elevator]) -> bool:
        """
        Add a list of elevators to the simulator.
        """
        raise NotImplementedError()

    # TODO rename
    def remove_walls_doors(self) -> bool:
        """
        Remove every spawned wall and door from the simulator.
        """
        raise NotImplementedError()
