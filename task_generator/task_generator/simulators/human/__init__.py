import abc
import typing
from collections.abc import Sequence

import rclpy
import rclpy.publisher
from arena_rclpy_mixins.shared import Namespace
from geometry_msgs.msg import PoseStamped

from task_generator import NodeInterface
from task_generator.constants import Constants
from task_generator.shared import (Door, DynamicObstacle, Floor, Obstacle,
                                   Pose, Robot, Wall)
from task_generator.simulators.human.utils import KnownObstacles, ObstacleLayer
from task_generator.simulators.sim import BaseSim
from task_generator.utils.registry import Registry


class BaseHumanSimulator(NodeInterface, abc.ABC):

    _goal_pub: rclpy.publisher.Publisher
    _known_obstacles: KnownObstacles

    def __init__(
        self,
        namespace: Namespace,
        simulator: BaseSim,
    ):
        """
        Initialize dynamic obstacle manager.

        Args:
            namespace: global namespace
            simulator: Simulator instance
            node: ROS Node instance (optional)
        """
        self._simulator = simulator
        self._namespace = namespace

        NodeInterface.__init__(self)

        self._known_obstacles = KnownObstacles()

        self._goal_pub = self.node.create_publisher(
            PoseStamped,
            self._namespace("/goal"),
            1
        )

    def spawn_obstacles(
        self,
        obstacles: Sequence[Obstacle],
        layer: ObstacleLayer = ObstacleLayer.INUSE
    ):
        """
        Loads given obstacles into the simulator.
        """
        self._logger.debug(f'spawning {len(obstacles)} static obstacles')

        unspawneds = []

        for obstacle in obstacles:
            if (known := self._known_obstacles.get(obstacle.name)) is not None:
                known.obstacle = obstacle
                self._simulator.obstacle_move(known.obstacle.name, known.obstacle.pose)
                known.layer = layer
            else:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle,
                )
            if not known.spawned:
                unspawneds.append(known)

        to_simulator = []
        for (known, obstacle) in zip(unspawneds, self._spawn_obstacles_impl([unspawned.obstacle for unspawned in unspawneds])):
            if not obstacle:
                continue
            known.obstacle = obstacle
            known.spawned = True

            if known.layer == ObstacleLayer.UNUSED:
                to_simulator.append(known.obstacle)

        self._simulator.obstacle_spawn(to_simulator)

    def spawn_dynamic_obstacles(
        self,
        obstacles: typing.Sequence[DynamicObstacle]
    ):
        """
        Loads given obstacles into the simulator.
        """
        self._logger.debug(f'spawning {len(obstacles)} dynamic obstacles')

        unspawneds = []

        for obstacle in obstacles:
            if (known := self._known_obstacles.get(obstacle.name)) is not None:
                known.obstacle = obstacle
                self._simulator.pedestrian_move(known.obstacle.name, known.obstacle.pose)
                known.layer = ObstacleLayer.INUSE
            else:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle
                )
            if not known.spawned:
                unspawneds.append(known)

        to_simulator = []

        for (known, obstacle) in zip(unspawneds, self._spawn_dynamic_obstacles_impl([unspawned.obstacle for unspawned in unspawneds])):
            if not obstacle:
                continue
            known.obstacle = obstacle
            known.spawned = True

            if known.layer == ObstacleLayer.UNUSED:
                to_simulator.append(known.obstacle)
        self._simulator.pedestrian_spawn(to_simulator)

    def spawn_world(
        self,
        walls: Sequence[Wall],
        doors: Sequence[Door],
    ):
        """
        Adds walls and doors to the simulator.
        """
        self._logger.debug(f'spawning {len(walls)} walls and {len(doors)} doors')
        # Ensure doors are spawned first so wall-spawn logic can split walls
        # and create gaps where doors are present.
        self._simulator.spawn_doors(list(doors))
        self._simulator.spawn_walls(list(walls))
        self._spawn_walls_impl(walls)
        self._spawn_doors_impl(doors)

    def unuse_obstacles(self):
        """
        Prepares obstacles for reuse or removal.
        """
        self._logger.debug('unusing obstacles')
        self._remove_obstacles_impl()
        for obstacle in self._known_obstacles.values():
            obstacle.spawned = False
            if obstacle.layer == ObstacleLayer.INUSE:
                obstacle.layer = ObstacleLayer.UNUSED

    def remove_obstacles(
        self,
        purge: ObstacleLayer = ObstacleLayer.UNUSED
    ):
        """
        Removes obstacles from simulator.
        @purge: remove obstacles down to this layer
        """
        self._logger.debug(f'removing obstacles (level {purge})')
        if purge >= ObstacleLayer.WORLD:
            self._simulator.remove_walls_doors()

        static = []
        dynamic = []
        for oid, known in list(self._known_obstacles.items()):
            if purge >= known.layer:
                if isinstance(known.obstacle, DynamicObstacle):
                    dynamic.append(known.obstacle)
                else:
                    static.append(known.obstacle)
                self._known_obstacles.forget(name=oid)

        self._simulator.obstacle_delete(static)
        self._simulator.pedestrian_delete(dynamic)

    def spawn_robot(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        """
        Spawns a robot.
        @robot: Robot.
        """
        self._logger.debug(f'spawning {len(robots)} robots')
        sim_success = self._simulator.robot_spawn(robots)
        human_success = self._spawn_robot_impl(tuple(r for r, s in zip(robots, sim_success) if s))
        human_iter = iter(human_success)
        success = (s and next(human_iter) for s in sim_success)
        return tuple(success)

    def remove_robot(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        """
        Removes robot from the simulation.
        @robot: Robot.
        """
        self._logger.debug(f'removing {len(robots)} robots')
        sim_success = self._simulator.robot_delete(robots)
        human_success = self._remove_robot_impl(tuple(r for r, s in zip(robots, sim_success) if s))
        human_iter = iter(human_success)
        success = (s and next(human_iter) for s in sim_success)
        return tuple(success)

    def move_robot(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        """
        Moves robot.
        @robot: Robot.
        """
        self._logger.debug(f'moving {len(robots)} robots')
        sim_success = self._simulator.robot_move(robots)
        human_success = self._move_robot_impl(tuple(r for r, s in zip(robots, sim_success) if s))
        human_iter = iter(human_success)
        success = (s and next(human_iter) for s in sim_success)
        return tuple(success)

    # impl

    @abc.abstractmethod
    def _spawn_obstacles_impl(
        self,
        obstacles: Sequence[Obstacle],
    ) -> Sequence[Obstacle | None]:
        ...

    @abc.abstractmethod
    def _spawn_dynamic_obstacles_impl(
        self,
        obstacles: Sequence[DynamicObstacle],
    ) -> Sequence[DynamicObstacle | None]:
        ...

    @abc.abstractmethod
    def _remove_obstacles_impl(
        self,
    ) -> bool:
        ...

    @abc.abstractmethod
    def _spawn_walls_impl(
        self,
        walls: Sequence[Wall],
    ) -> bool:
        ...

    @abc.abstractmethod
    def _spawn_doors_impl(
        self,
        doors: Sequence[Door],
    ) -> bool:
        ...

    @abc.abstractmethod
    def _spawn_robot_impl(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        ...

    @abc.abstractmethod
    def _remove_robot_impl(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        ...

    @abc.abstractmethod
    def _move_robot_impl(
        self,
        robots: Sequence[Robot],
    ) -> Sequence[bool]:
        ...


EntityManagerRegistry = Registry[Constants.HumanSimulator, BaseHumanSimulator]()


@EntityManagerRegistry.register(Constants.HumanSimulator.DUMMY)
def dummy():
    from .dummy import DummyHumanSimulator
    return DummyHumanSimulator


@EntityManagerRegistry.register(Constants.HumanSimulator.HUNAV)
def lazy_hunavsim():
    from .hunav.hunav import HunavHumanSimulator
    return HunavHumanSimulator


@EntityManagerRegistry.register(Constants.HumanSimulator.ISAAC)
def isaacsim():
    from .isaac import IsaacHumanSimulator
    return IsaacHumanSimulator
