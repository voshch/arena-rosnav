from task_generator.simulators.sim import BaseSim

from collections.abc import Sequence
from task_generator.shared import Entity


class DummySimulator(BaseSim):
    """
    Does nothing.
    """

    def before_reset_task(self):
        self._logger.debug("pausing")
        return True

    def after_reset_task(self):
        self._logger.debug("unpausing")
        return True

    # fake spawn
    def __spawn_entity(self, entities: Sequence[Entity]) -> Sequence[bool]:
        self._logger.debug(f"spawning {len(entities)} entities")
        return tuple(True for _ in entities)

    def obstacle_spawn(self, obstacles):
        return self.__spawn_entity(obstacles)

    def pedestrian_spawn(self, pedestrians):
        return self.__spawn_entity(pedestrians)

    def robot_spawn(self, robots):
        return self.__spawn_entity(robots)

    # fake move
    def __move_entity(self, entities: Sequence[Entity]) -> Sequence[bool]:
        self._logger.debug(f"moving {len(entities)} entities")
        return tuple(True for _ in entities)

    def obstacle_move(self, obstacles):
        return self.__move_entity(obstacles)

    def pedestrian_move(self, pedestrians):
        return self.__move_entity(pedestrians)

    def robot_move(self, robots):
        return self.__move_entity(robots)

    # fake delete
    def __delete_entity(self, entities: Sequence[Entity]) -> Sequence[bool]:
        self._logger.debug(f"deleting {len(entities)} entities")
        return tuple(True for _ in entities)

    def obstacle_delete(self, obstacles):
        return self.__delete_entity(obstacles)

    def pedestrian_delete(self, pedestrians):
        return self.__delete_entity(pedestrians)

    def robot_delete(self, robots):
        return self.__delete_entity(robots)

    # assorted
    def pedestrian_update(self, pedestrians):
        self._logger.debug(f'updating {len(pedestrians.pedestrians)} pedestrians')
        return tuple(True for _ in pedestrians.pedestrians)

    # world interface
    def spawn_walls(self, walls):
        self._logger.debug(f'spawning {len(walls)} walls')
        for wall in walls:
            wall.assets()
        return True

    def spawn_floors(self, floors):
        self._logger.debug(f'spawning {len(floors)} floors')
        return True

    def spawn_doors(self, doors):
        self._logger.debug(f'spawning {len(doors)} doors')
        return True

    def spawn_elevators(self, elevators) -> bool:
        self._logger.debug(f'spawning {len(elevators)} elevators')
        return True

    def remove_walls_doors(self):
        self._logger.debug('removing all walls and doors')
        return True
