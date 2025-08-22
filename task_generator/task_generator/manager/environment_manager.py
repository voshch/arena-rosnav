import itertools
import typing
from collections.abc import Callable, Collection, Iterator
from typing import Any

import attrs
from arena_simulation_setup.worlds.world import WorldDescription

from task_generator import NodeInterface
from task_generator.shared import (Door, DynamicObstacle, Entity, Obstacle,
                                   Orientation, Pose, Position, Robot, Wall)
from task_generator.simulators.human import BaseHumanSimulator
from task_generator.simulators.human.utils import ObstacleLayer
from task_generator.simulators.sim import BaseSim

EntityPropsT = typing.TypeVar('EntityPropsT', bound=Entity)


class _Realizer:
    @attrs.frozen()
    class _Configuration:
        x: float = 0.0
        y: float = 0.0
        prefix: str = ''

    _config: "_Configuration"

    @typing.overload
    def realize(self, target: str) -> str: ...

    def _prefix(self, s: str) -> str:
        return self._config.prefix + s

    def _realize_position(self, position: Position) -> Position:
        position.x += self._config.x
        position.y += self._config.y
        return position

    def _realize_orientation(self, orientation: Orientation) -> Orientation:
        return orientation

    def _realize_pose(self, pose: Pose) -> Pose:
        pose.position = self._realize_position(pose.position)
        return pose

    @typing.overload
    def realize(self, target: EntityPropsT) -> EntityPropsT: ...

    @typing.overload
    def realize(self, target: Position) -> Position: ...

    @typing.overload
    def realize(self, target: Pose) -> Pose: ...

    def _realize_entity(self, entity: EntityPropsT) -> EntityPropsT:
        entity.pose = self._realize_pose(entity.pose)
        entity.name = self._prefix(entity.name)
        return entity

    @typing.overload
    def realize(self, target: Wall) -> Wall: ...

    def _realize_wall(self, wall: Wall) -> Wall:
        return attrs.evolve(
            wall,
            start=self._realize_position(wall.start),
            end=self._realize_position(wall.end),
        )

    @typing.overload
    def realize(self, target: Door) -> Door: ...

    def _realize_door(self, door: Door) -> Door:
        return attrs.evolve(
            door,
            start=self._realize_position(door.start),
            end=self._realize_position(door.end),
        )

    def realize(
        self,
        target
    ):
        if isinstance(target, str):
            return self._prefix(target)

        if isinstance(target, Position):
            return self._realize_position(target)

        if isinstance(target, Pose):
            return self._realize_pose(target)

        if isinstance(target, Entity):
            return self._realize_entity(target)

        if isinstance(target, Wall):
            return self._realize_wall(target)

        if isinstance(target, Door):
            return self._realize_door(target)

        raise TypeError(f'realization not implemented for type {type(target)}')


class EnvironmentManager(NodeInterface, _Realizer):

    _namespace: str
    _human_simulator: BaseHumanSimulator
    _simulator: BaseSim

    id_generator: Iterator[int]

    def __init__(
        self,
        namespace,
        simulator: BaseSim,
        entity_manager: BaseHumanSimulator,
    ):
        NodeInterface.__init__(self)

        self._namespace = namespace
        self._simulator = simulator
        self._human_simulator = entity_manager

        ref_x, ref_y = self.node.rosparam[tuple[float, float]].get('reference', [0.0, 0.0])
        prefix = self.node.rosparam[str].get('prefix', '')
        self._config = self._Configuration(
            x=ref_x,
            y=ref_y,
            prefix=prefix,
        )

        self.id_generator = itertools.count(434)

    def spawn_world_obstacles(self, world: WorldDescription):
        """
        Loads given obstacles into the simulator,
        the map file is retrieved from launch parameter "world"
        """

        walls = world.all_walls
        doors = world.all_doors
        floors = list(world.all_floors)

        realized_doors = list(map(self._realize_door, doors))
        if realized_doors:
            self._simulator.spawn_doors(realized_doors)

        if walls or doors:
            self._human_simulator.spawn_world(
                list(map(self._realize_wall, walls)),
                realized_doors,
            )
        if floors:
            self._logger.debug(f'spawning {len(floors)}')
            self._simulator.spawn_floors(list(floors))
        self._human_simulator.spawn_obstacles(
            list(map(self._realize_entity, world.all_static_entities)),
            layer=ObstacleLayer.WORLD,
        )

    def spawn_dynamic_obstacles(self, setups: Collection[DynamicObstacle]):
        """
        Loads given dynamic obstacles into the simulator.
        """

        self._human_simulator.spawn_dynamic_obstacles(
            list(map(self._realize_entity, setups))
        )

    def spawn_obstacles(self, setups: Collection[Obstacle]):
        """
        Loads given obstacles into the simulator.
        """

        self._human_simulator.spawn_obstacles(
            list(map(self._realize_entity, setups))
        )

    def spawn_robot(self, robot: Robot) -> Robot:
        """
        Loads given robot into the simulator
        """
        robot = self._realize_entity(robot)
        self._human_simulator.spawn_robot(robot)
        return robot

    def move_robot(self, name: str, pose: Pose):
        """
        Moves given robot
        """
        self._human_simulator.move_robot(
            name=name,
            pose=self._realize_pose(pose),
        )

    def respawn(self, callback: Callable[[], Any]):
        """
        Unuse obstacles, (re-)use them in callback, finally remove unused obstacles
        @callback: Function to call between unuse and remove
        """
        self._human_simulator.unuse_obstacles()
        callback()
        self._human_simulator.remove_obstacles(purge=ObstacleLayer.UNUSED)

    def reset(self, purge: ObstacleLayer = ObstacleLayer.INUSE):
        """
        Unuse and remove all obstacles
        """
        self._human_simulator.remove_obstacles(purge=purge)
