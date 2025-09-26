from __future__ import annotations

import typing
from typing import Optional, Type, TypeVar

import attrs
import rclpy
import rclpy.node
from arena_rclpy_mixins.shared import Namespace
from arena_simulation_setup.shared import (  # noqa
    CustomDynamicObstacle,
    Door,
    DynamicObstacle,
    Entity,
    Floor,
    Obstacle,
    Wall,
)
from arena_simulation_setup.shared import Robot as Robot_
from arena_simulation_setup.utils.geometry import (  # noqa
    Orientation,
    Pose,
    Position,
    PositionRadius,
)
from arena_simulation_setup.utils.models import Model, ModelType, ModelWrapper  # noqa


def configure_node(node: rclpy.node.Node):
    global _node
    _node = node


T = TypeVar("T")


def rosparam_get(
    cast: Type[T], param_name: str, default: T
) -> T:
    """
    # TODO deprecate in favor of ROSParamServer.rosparam[T].get
    Get typed ros parameter (strict)
    @cast: Return type of function
    @param_name: Name of parameter on parameter server
    @default: Default value. Raise ValueError is default is unset and parameter can't be found.
    """
    return _node.rosparam[cast].get(param_name, default)


def rosparam_set(
    param_name: str, value: typing.Any
) -> bool:
    """
    # TODO deprecate in favor of ROSParamServer.rosparam[T].set
    """
    return _node.rosparam.set(param_name, value)


@attrs.define()
class Robot(Robot_):
    inter_planner: str
    local_planner: str
    global_planner: str
    agent: str
    record_data_dir: Optional[str] = None

    def compatible(self, value: Robot) -> bool:
        return self.model.name == value.model.name and self.local_planner == value.local_planner and self.global_planner == value.global_planner and self.agent == value.agent

    def __eq__(self, value: object) -> bool:
        if not isinstance(value, Robot):
            return False

        return self.compatible(value) and self.name == value.name and self.record_data_dir == value.record_data_dir

    @property
    def frame(self) -> Namespace:
        if not self.name:
            return Namespace('')
        return Namespace(self.name)

    @classmethod
    def parse(cls, value: dict) -> "Robot":
        name = str(value.get("name", ""))
        pose = Pose(value.get("pos", (0, 0, 0)))
        inter_planner = str(
            value.get("inter_planner", rosparam_get(str, "inter_planner", ""))
        )
        local_planner = str(
            value.get("local_planner", rosparam_get(str, "local_planner", ""))
        )
        global_planner = str(
            value.get("global_planner", rosparam_get(str, "global_planner", ""))
        )
        model = str(value.get("model", rosparam_get(str, "model", "")))
        agent = str(value.get("agent", rosparam_get(str, "agent_name", "")))
        record_data = value.get(
            "record_data_dir", rosparam_get(str, "record_data_dir", None)
        )

        return cls(
            name=name,
            pose=pose,
            inter_planner=inter_planner,
            local_planner=local_planner,
            global_planner=global_planner,
            model=model,
            agent=agent,
            record_data_dir=record_data,
            extra=value,
        )

    def __attrs_post_init__(self):
        # Inject robot model parameters into model loader

        def inject_args(f):
            """
            Inject common robot attributes into model get call
            """
            def inner(only: list[ModelType], loader_args: dict | None = None):
                return f(only, loader_args=self.asdict() if loader_args is None else {**self.asdict(), **loader_args})
            return inner
        self.model._get = inject_args(self.model._get)
