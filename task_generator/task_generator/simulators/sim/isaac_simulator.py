import itertools
import os
import random
import sys
import time
import traceback
import typing

import arena_people_msgs.msg
import arena_simulation_setup.entities.robot
import attrs
import numpy as np
import rclpy
import rclpy.client
from geometry_msgs.msg import Point
from isaacsim_msgs.msg import (
    Door,
    Elevator,
    Floor,
    Material,
    Pedestrian,
    PedestrianGoal,
    Prim,
    Scale,
    Wall,
)
from isaacsim_msgs.srv import (
    DeletePrims,
    EditPrims,
    NavigatePedestrians,
    SpawnDoors,
    SpawnElevators,
    SpawnFloors,
    SpawnPedestrians,
    SpawnPrims,
    SpawnUrdf,
    SpawnUsd,
    SpawnWalls,
)
from std_msgs.msg import String as StdString
from task_generator.shared import (
    DynamicObstacle,
    ModelType,
    Namespace,
    Obstacle,
    Robot,
)
from task_generator.simulators.sim import BaseSim, NodeInterface


@attrs.define()
class _Service:
    type_: typing.Any
    name: str

    _client: rclpy.client.Client = attrs.field(init=False)

    @property
    def client(self) -> rclpy.client.Client:
        if self._client is None:
            raise RuntimeError(f"client for service {self.name} not initialized")
        return self._client

    @client.setter
    def client(self, value: rclpy.client.Client):
        self._client = value


class IsaacSimulator(BaseSim):

    _NS_PRIM = Namespace('Obstacles')
    _NS_PEDESTRIAN = Namespace('Pedestrians')
    _NS_ROBOT = Namespace('Robots')
    _NS_WALL = Namespace('Walls')
    _NS_FLOOR = Namespace('Floors')
    _NS_DOOR = Namespace('Doors')

    class _services:
        DeleteAllPedestrians = _Service(type_=DeletePrims, name="isaac/DeleteAllPedestrians")
        DeletePrims = _Service(type_=DeletePrims, name="isaac/DeletePrims")
        EditPrims = _Service(type_=EditPrims, name="isaac/EditPrims")
        NavigatePedestrians = _Service(type_=NavigatePedestrians, name="isaac/NavigatePedestrians")
        SpawnDoors = _Service(type_=SpawnDoors, name="isaac/SpawnDoors")
        SpawnFloors = _Service(type_=SpawnFloors, name='isaac/SpawnFloors')
        SpawnPedestrians = _Service(type_=SpawnPedestrians, name="isaac/SpawnPedestrians")
        SpawnPrims = _Service(type_=SpawnPrims, name="isaac/SpawnPrims")
        SpawnUrdf = _Service(type_=SpawnUrdf, name="isaac/SpawnUrdf")
        SpawnUsd = _Service(type_=SpawnUsd, name="isaac/SpawnUsd")
        SpawnWalls = _Service(type_=SpawnWalls, name="isaac/SpawnWalls")
        SpawnElevators = _Service(type_=SpawnElevators, name="isaac/SpawnElevators")

    def __init__(self, namespace):
        """Initialize IsaacSimulator

        Args:
            namespace: Namespace for the simulator
        """
        NodeInterface.__init__(self)
        super().__init__(namespace)

        self.wall_counter = itertools.count()
        self.floor_counter = itertools.count()
        self._spawned_doors = []

        self._init_service_clients()

        if hasattr(self, 'node') and self.node is not None:
            self._logger.info(f"IsaacSimulator initialized with namespace: {namespace}")

    def robot_spawn(self, robots):
        def impl(robot: Robot) -> bool:
            try:
                model = robot.model.get(
                    (
                        ModelType.URDF,
                        # ModelType.USD
                    )
                )

                if model.type == ModelType.URDF:
                    robot_params = arena_simulation_setup.entities.robot.Robot(robot.model.name).model_params

                    fq_name = self._NS_ROBOT(robot.name)

                    self._services.SpawnUrdf.client.call(
                        SpawnUrdf.Request(
                            name=fq_name,
                            urdf_path=model.path,
                            robot_model=robot.model.name,
                            localization=True,
                            tf_prefix=robot.name,
                            base_frame=robot_params.base_frame,
                            odom_frame=robot_params.odom_frame,
                            pose=robot.pose.to_msg(),
                            cmd_vel_topic=self.node.service_namespace(robot.name, 'cmd_vel'),
                            joint_states_topic=self.node.service_namespace(robot.name, 'joint_states'),
                        )
                    )

                    # from isaac_utils.managers.door_manager import
                    base_frame = robot_params.base_frame
                    robot_prim_path = os.path.join("/World", fq_name, base_frame)

                    # Publish registration message so DoorManager in IsaacSim process
                    # registers the robot. This avoids cross-process direct calls.
                    try:
                        if getattr(self, '_reg_pub', None) is not None:
                            self._reg_pub.publish(StdString(data=f"robot|{robot_prim_path}"))
                            self._logger.debug(f"Published registration for robot: {robot_prim_path}")
                        else:
                            self._logger.warning('Registration publisher not available; robot not registered with IsaacSim DoorManager')
                    except Exception as e:
                        self._logger.warning(f'Failed to publish robot registration: {e}')

                    return True

                # TODO
                raise NotImplementedError(
                    f"robot model of type {model.type} can't be spawned by {self.__class__.__name__}"
                )

            except Exception as e:
                self._logger.error(repr(e))
                return False

        return tuple(map(impl, robots))

    def obstacle_spawn(self, obstacles):
        req = SpawnPrims.Request()

        results = [True] * len(obstacles)

        for i, obstacle in enumerate(obstacles):
            model = obstacle.model.get([ModelType.USD])
            if model.type is ModelType.UNKNOWN:
                results[i] = False
                continue
            prim = Prim()
            prim.usd_path = model.path
            prim.name = self._NS_PRIM(obstacle.name)
            prim.pose = obstacle.pose.to_msg()
            req.prims.append(prim)

        response = self._services.SpawnPrims.client.call(req)
        response_iter = iter(response.ret)

        return tuple(a and next(response_iter) for a in results)

    def obstacle_move(self, obstacles):
        def move_obstacle(obstacle: Obstacle) -> bool:
            return self._move_entity(self._NS_PRIM(obstacle.name), obstacle.pose)
        return tuple(map(move_obstacle, obstacles))

    def pedestrian_move(self, pedestrians):
        def move_pedestrian(pedestrian: Pedestrian) -> bool:
            return self._move_entity(self._NS_PEDESTRIAN(pedestrian.name), pedestrian.pose)
        return tuple(map(move_pedestrian, pedestrians))

    def robot_move(self, robots):
        def move_robot(robot: Robot) -> bool:
            return self._move_entity(self._NS_ROBOT(robot.name), robot.pose)
        return tuple(map(move_robot, robots))

    def obstacle_delete(self, obstacles):
        return tuple(self._delete_entity(self._NS_PRIM(o.name)) for o in obstacles)

    def pedestrian_delete(self, pedestrians):
        return (True,) * len(pedestrians)
        # TODO uncomment when pedestrians aren't deleted immediately
        return tuple(self._delete_entity(self._NS_PEDESTRIAN(p.name)) for p in pedestrians)

    def robot_delete(self, robots):
        return tuple(self._delete_entity(self._NS_ROBOT(r.name)) for r in robots)

    def remove_walls_doors(self):
        self._delete_entity(self._NS_WALL)
        self._delete_entity(self._NS_DOOR)
        return True

    def spawn_walls(self, walls):
        # return True
        self._logger.debug("Attempting to spawn walls")

        walls_req = SpawnWalls.Request()
        prims_req = SpawnPrims.Request()

        for wall in walls:

            segments, obstacles = wall.assets()

            for segment in segments:
                end = segment.end.to_msg()
                end.z += segment.height
                try:
                    wall_name = self.node._environment_manager.realize(f"wall_{next(self.wall_counter)}")
                    walls_req.walls.append(
                        Wall(
                            name=self._NS_WALL(wall_name),
                            start=segment.start.to_msg(),
                            end=end,
                            material=Material(**segment.material.load(default=segment.material.DEFAULT().load()).asdict()),
                            thickness=segment.width,
                        )
                    )

                except Exception as e:
                    self._logger.error("Failed to spawn wall")
                    self._logger.error(repr(e))
                    traceback.print_exc(file=sys.stderr)

            for obstacle in obstacles:
                try:
                    prim_name = self.node._environment_manager.realize(f"obstacle_{next(self.wall_counter)}")
                    model = obstacle.model.get(ModelType.USD)
                    if model.type is ModelType.UNKNOWN:
                        continue
                    prim = Prim()
                    prim.usd_path = model.path
                    prim.name = self._NS_WALL(prim_name)
                    prim.pose = obstacle.pose.to_msg()
                    prims_req.prims.append(prim)

                except Exception as e:
                    self._logger.error("Failed to spawn wall obstacle")
                    self._logger.error(repr(e))
                    traceback.print_exc(file=sys.stderr)

        res = all(self._services.SpawnWalls.client.call(walls_req).ret) and all(self._services.SpawnPrims.client.call(prims_req).ret)

        self._logger.info("All walls spawned.")
        return res

    def spawn_floors(self, floors) -> bool:
        self._logger.info("Attempting to spawn floors")

        req = SpawnFloors.Request()

        for floor in floors:
            try:
                i = next(self.floor_counter)
                req.floors.append(
                    Floor(
                        name=self._NS_FLOOR(f"floor_{i}"),
                        x_length=floor.x_length,
                        y_length=floor.y_length,
                        pos=floor.pos.to_msg(),
                        material=Material(**floor.material.load(default=floor.material.DEFAULT().load()).asdict()),
                    )
                )

            except Exception as e:
                self._logger.error("Failed to spawn floor")
                self._logger.error(repr(e))
                traceback.print_exc(file=sys.stderr)

        res = all(self._services.SpawnFloors.client.call(req).ret)
        self._logger.info("All floors spawned successfully.")
        return res

    def spawn_doors(self, doors) -> bool:
        req = SpawnDoors.Request()
        for door in doors:
            try:
                end = door.end.to_msg()
                end.z += door.height
                req.doors.append(
                    Door(
                        name=self._NS_DOOR(door.name),
                        start=door.start.to_msg(),
                        end=end,
                        material=Material(**door.material.load(default=door.material.DEFAULT().load()).asdict()),
                        thickness=0.1,
                        kind=door.kind,
                    )
                )
            except Exception as e:
                self._logger.error("Failed to spawn door")
                self._logger.error(repr(e))
                traceback.print_exc(file=sys.stderr)
        res = all(self._services.SpawnDoors.client.call(req).ret)
        self._logger.info("All doors spawned successfully.")
        return res

    def spawn_elevators(self, elevators) -> bool:
        self._logger.debug(f"IsaacSimulator.spawn_elevators ENTRY, elevators: {elevators}")
        self._logger.debug(f"IsaacSimulator.spawn_elevators called with: {[e.name for e in elevators]}")
        for e in elevators:
            self._logger.debug(f"Elevator data: {e}")

        req = SpawnElevators.Request()
        for elevator in elevators:
            try:
                pos = elevator.position
                size = elevator.size
                size = Scale(x=size[0], y=size[1], z=size[2])
                req.elevators.append(
                    Elevator(
                        name=elevator.name,
                        position=pos,
                        size=size,
                        height_min=elevator.height_min,
                        height_max=elevator.height_max,
                        material=Material(**elevator.material.load(default=elevator.material.DEFAULT().load()).asdict()),
                    )
                )
            except Exception as e:
                self._logger.error(f"Failed to append elevator: {elevator.name}, error: {e}")

        res = all(self._services.SpawnElevators.client.call(req).ret)
        self._logger.debug("All elevators spawned successfully." if res else "Failed to spawn one or more elevators")
        return res

    # TODO: update
    def before_reset_task(self):
        self._delete_all_pedestrians(self._NS_PEDESTRIAN)
        time.sleep(0.5)
        return True

    # TODO: update
    def after_reset_task(self):
        return True

    def pedestrian_spawn(self, pedestrians):

        req = SpawnPedestrians.Request()
        on_success: list[tuple[str, str]] = []

        # TODO implement targeted pedestrian models
        for pedestrian in pedestrians:
            available_models: dict[str, str] = {
                # "F_Business_02",
                # "F_Medical_01",
                # "M_Medical_01",
                # "biped_demo",
                # "female_adult_police_01_new",
                # "female_adult_police_02",
                # "female_adult_police_03_new",
                # "male_adult_construction_01_new",
                # "male_adult_construction_03",
                # "male_adult_construction_05_new",
                # "male_adult_police_04",
                "female_adult_business_02": "original_female_adult_business_02",
                "female_adult_medical_01": "original_female_adult_medical_01",
                "female_adult_police_01": "original_female_adult_police_01",
                "female_adult_police_02": "original_female_adult_police_02",
                "female_adult_police_03": "original_female_adult_police_03",
                "male_adult_construction_01": "original_male_adult_construction_01",
                "male_adult_construction_02": "original_male_adult_construction_02",
                "male_adult_construction_03": "original_male_adult_construction_03",
                "male_adult_construction_05": "original_male_adult_construction_05",
                "male_adult_medical_01": "original_male_adult_medical_01",
                "male_adult_police_04": "original_male_adult_police_04",
            }
            if pedestrian.model.name in available_models:
                model_name = pedestrian.model.name
            else:
                model_name = random.choice(tuple(available_models.keys()))

            ped = Pedestrian()
            ped.name = self._NS_PEDESTRIAN(pedestrian.name)
            ped.character_name = available_models[model_name]
            ped.pose = pedestrian.pose.to_msg()
            ped.controller_stats = False

            req.pedestrians.append(ped)
            on_success.append((pedestrian.name, model_name))

        res = self._services.SpawnPedestrians.client.call(req)

        for status, (name, model_name) in zip(res.ret, on_success):
            if status:
                self.ped_dict[name] = model_name

        self.pedestrian_update(
            arena_people_msgs.msg.Pedestrians(pedestrians=[
                arena_people_msgs.msg.Pedestrian(
                    name=ped.name,
                    pose=ped.pose.to_msg(),
                )
                for status, ped
                in zip(res.ret, pedestrians)
                if status
            ])
        )

        return res.ret

    def pedestrian_update(self, pedestrians):
        req = NavigatePedestrians.Request()

        def impl(ped: DynamicObstacle) -> bool:
            name = ped.name
            if not name in self.ped_dict:
                self._logger.warning(f"Pedestrian {name} not found in ped_dict: {list(self.ped_dict.keys())}")
                return False

            goal = PedestrianGoal()
            goal.name = self._NS_PEDESTRIAN(name, "ManRoot", self.ped_dict[name])
            goal.position = ped.pose.position
            goal.velocity = np.linalg.norm([ped.twist.linear.x, ped.twist.linear.y])
            req.goals.append(goal)
            return True

        preflight = tuple(map(impl, pedestrians.pedestrians))
        results = self._services.NavigatePedestrians.client.call(req).ret

        return tuple(a and b for a, b in zip(preflight, results))

    def _delete_entity(self, name: str) -> bool:
        self._logger.debug(f"Attempting to delete prim {name}")

        res = self._services.DeletePrims.client.call(
            DeletePrims.Request(
                names=[name]
            )
        )

        return res.ret[0]

    def _delete_all_pedestrians(self, prim_path):
        self._logger.info(f"Attempting to delete prim named {prim_path}")

        res = self._services.DeleteAllPedestrians.client.call(
            DeletePrims.Request(names=[prim_path])
        )

        return res.ret[0]

    def _move_entity(self, name, pose):
        self._logger.debug(f"Attempting to move entity: {name}")
        self._logger.debug(f"position: {pose.position.x,pose.position.y}")
        self._logger.debug(f"orientation: {pose.orientation}")

        if name in self.ped_dict:
            name = os.path.join('pedestrians', name)

        response = self._services.EditPrims.client.call(
            EditPrims.Request(
                prims=[
                    Prim(
                        name=name,
                        pose=pose.to_msg(),
                    )
                ],
                pose=True,
            )
        )

        return response.ret[0]

    def _init_service_clients(self):
        """
        Initialize all ROS 2 service clients and wait for their availability.
        """
        if hasattr(self, 'node') and self.node is not None:
            logger = self._logger
            logger.info("Initializing service clients...")
        else:
            logger = None
            print("Initializing service clients...")

        # Define services with their corresponding client attributes
        for service in (service for at, service in self._services.__dict__.items() if not at.startswith('_')):
            service.client = self.node.create_client(service.type_, service.name) if hasattr(self, 'node') and self.node is not None else None
            if logger:
                logger.debug(f'Waiting for service "{service.name}"...')
            else:
                print(f'Waiting for service "{service.name}"...')

            poll_interval: float = 1.0
            shout_every: int = 30

            polls: int = 0
            if service._client is not None:
                while not service._client.wait_for_service(timeout_sec=poll_interval):
                    polls += 1
                    if polls % shout_every == 0:
                        if logger:
                            logger.warning(f'Service "{service.name}" not available after waiting {poll_interval * polls}s')
                        else:
                            print(f'Service "{service.name}" not available after waiting {poll_interval * polls}s')
                if logger:
                    logger.debug(f'Service "{service.name}" is now available.')
                else:
                    print(f'Service "{service.name}" is now available.')

        self.ped_dict = {}

        # Publisher for external registration messages so IsaacSim's DoorManager
        # can be informed about spawned entities in the IsaacSim process.
        try:
            self._reg_pub = self.node.create_publisher(StdString, '/isaac/register_entity', 10) if hasattr(self, 'node') and self.node is not None else None
            if logger:
                logger.info('Created /isaac/register_entity publisher')
            else:
                print('Created /isaac/register_entity publisher')
        except Exception as e:
            self._reg_pub = None
            if logger:
                logger.warning(f'Failed to create registration publisher: {e}')
            else:
                print(f'Failed to create registration publisher: {e}')
        if logger:
            logger.info("All service clients initialized and available.")
        else:
            print("All service clients initialized and available.")
