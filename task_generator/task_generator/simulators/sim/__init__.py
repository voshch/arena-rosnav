from __future__ import annotations

import abc

from arena_rclpy_mixins.shared import Namespace
from task_generator.constants import Constants
from task_generator.utils.registry import Registry

from task_generator import NodeInterface

from ._interface import ObstacleITF, PedestrianITF, RobotITF, WorldITF


class BaseSim(NodeInterface, ObstacleITF, PedestrianITF, RobotITF, WorldITF, abc.ABC):

    _namespace: Namespace

    def __init__(self, namespace: Namespace):
        NodeInterface.__init__(self)
        self._namespace = namespace

    @abc.abstractmethod
    def before_reset_task(self) -> bool:
        """
        Is executed each time before the task is reset. This is useful in
        order to pause the simulation.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def after_reset_task(self) -> bool:
        """
        Is executed after the task is reset. This is useful to unpause the
        simulation.
        """
        raise NotImplementedError()


SimulatorRegistry = Registry[Constants.SimSimulator, BaseSim]()


@SimulatorRegistry.register(Constants.SimSimulator.DUMMY)
def lazy_dummy():
    from .dummy_simulator import DummySimulator
    return DummySimulator


@SimulatorRegistry.register(Constants.SimSimulator.FLATLAND)
def lazy_flatland():
    from .flatland_simulator import FlatlandSimulator
    return FlatlandSimulator


@SimulatorRegistry.register(Constants.SimSimulator.GAZEBO)
def lazy_gazebo():
    from .gazebo_simulator import GazeboSimulator
    return GazeboSimulator


# @SimulatorRegistry.register(Constants.SimSimulator.UNITY)
# def lazy_unity():
#     from .unity_simulator import UnitySimulator
#     return UnitySimulator


@SimulatorRegistry.register(Constants.SimSimulator.ISAAC)
def lazy_isaac():
    from .isaac_simulator import IsaacSimulator
    return IsaacSimulator
