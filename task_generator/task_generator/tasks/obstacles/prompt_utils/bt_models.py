from abc import ABC
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Union, Literal, Any, Annotated
from pydantic import BaseModel, Field, field_validator


CONTROL_NODE_ID_MAP = {}

DECORATION_NODE_ID_MAP = {
    "TimeDelayDecorator": "TimeDelay"
}

ACTION_NODE_ID_MAP = {
    "GroupWalk": "SetGroupWalk",
}

CONDITION_NODE_ID_MAP = {}


# TreeNodesModel
# --------------
class InputPort(BaseModel):
    name: str
    type: str
    default: Optional[str] = None
    description: Optional[str] = None

    def to_xml(self) -> ET.Element:
        element = ET.Element(
            "input_port",
            attrib={
                "name": self.name,
                "type": self.type
            }
        )
        if self.default:
            element.set("default", self.default)
        if self.description:
            element.text = self.description

        return element


class OutputPort(BaseModel):
    name: str
    type: str
    default: Optional[str] = None
    description: Optional[str] = None

    def to_xml(self) -> ET.Element:
        element = ET.Element(
            "output_port",
            attrib={
                "name": self.name,
                "type": self.type
            }
        )
        if self.default:
            element.set("default", self.default)
        if self.description:
            element.text = self.description

        return element


class Condition(BaseModel):
    ID: str
    input_ports: Optional[List[InputPort]] = []
    output_port: Optional[List[OutputPort]] = []

    def to_xml(self) -> ET.Element:
        element = ET.Element(
            "Condition",
            attrib={
                "ID": self.ID
            }
        )

        for ip in self.input_ports or []:
            ip: InputPort
            element.append(ip.to_xml())

        for op in self.output_port or []:
            op: OutputPort
            element.append(op.to_xml())

        return element


class Action(BaseModel):
    ID: str
    input_ports: Optional[List[InputPort]] = []
    output_port: Optional[List[OutputPort]] = []

    @field_validator("ID")
    @classmethod
    def normalize_id(cls, v):
        if v in ACTION_NODE_ID_MAP:
            return ACTION_NODE_ID_MAP[v]
        return v

    def to_xml(self) -> ET.Element:
        element = ET.Element(
            "Action",
            attrib={
                "ID": self.ID
            }
        )

        for ip in self.input_ports or []:
            ip: InputPort
            element.append(ip.to_xml())

        for op in self.output_port or []:
            op: OutputPort
            element.append(op.to_xml())

        return element


class TreeNodesModel(BaseModel):
    conditions: Optional[List[Condition]] = []
    actions: Optional[List[Action]] = []

    def to_xml(self) -> ET.Element:
        element = ET.Element(
            "TreeNodesModel",
            attrib={}
        )

        for action in self.actions:
            element.append(action.to_xml())

        for condition in self.conditions:
            element.append(condition.to_xml())

        return element

# BehaviorTree
# ------------


class TreeNode(BaseModel):
    ID: str
    name: str
    attributes: Dict[str, Any] = {}

    def to_xml(self) -> ET.Element:
        ...


class DecorationNode(TreeNode):
    ID: Literal[
        "TimeDelayDecorator",
        "RetryUntilSuccessful"
    ]
    child_node: "NodeUnion"

    @field_validator("ID")
    @classmethod
    def normalize_id(cls, v):
        if v in DECORATION_NODE_ID_MAP:
            return DECORATION_NODE_ID_MAP[v]
        return v

    def to_xml(self):
        element = ET.Element(
            self.ID,
            attrib=self.attributes
        )

        element.append(self.child_node.to_xml())

        return element


class ControlNode(TreeNode):
    ID: Literal[
        "Sequence",
        "Fallback"
    ]
    children_nodes: List["NodeUnion"]

    def to_xml(self):
        element = ET.Element(
            self.ID,
            attrib=self.attributes
        )

        for node in self.children_nodes:
            element.append(node.to_xml())

        return element


class LeafNode(ABC, TreeNode):
    def to_xml(self):
        element = ET.Element(
            self.ID,
            attrib=self.attributes
        )

        return element


class ActionNode(LeafNode):
    ID: Literal[
        # Old nodes
        "UpdateGoal",
        "RegularNav",
        "SurprisedNav",
        "CuriousNav",
        "ScaredNav",
        "ThreateningNav",
        # New nodes
        "FindNearestAgent",
        "SaySomething",
        "SetGroupId",
        "SetGoal",
        "StopMovement",
        "ResumeMovement",
        "StopAndWaitTimerAction",
        "ConversationFormation",
        "GoTo",
        "ApproachAgent",
        "ApproachRobot",
        "BlockRobot",
        "BlockAgent",
        "GroupWalk",
        "LookAtPoint",
        "LookAtAgent",
        "LookAtRobot",
        "FollowAgent",
    ]

    @field_validator("ID")
    @classmethod
    def normalize_id(cls, v):
        if v in ACTION_NODE_ID_MAP:
            return ACTION_NODE_ID_MAP[v]
        return v


class ConditionNode(LeafNode):
    ID: Literal[
        # Old nodes
        "IsGoalReached",
        "IsRobotVisible",
        # New nodes
        "RandomChanceCondition",
        "IsRobotFacingAgent",
        "IsAgentVisible",
        "IsRobotClose",
        "IsAgentClose",
        "IsAtPosition",
        "IsAnyoneSpeaking",
        "IsSpeaking",
        "IsAnyoneLookingAtMe",
        "IsLookingAtMe"
    ]


class BehaviorTree(BaseModel):
    ID: str
    child_node: "NodeUnion"

    def to_xml(self) -> ET.Element:
        element = ET.Element(
            "BehaviorTree",
            attrib={
                "ID": self.ID
            }
        )

        element.append(self.child_node.to_xml())

        return element


class Root(BaseModel):
    main_tree_to_execute: str
    BTCPP_format: str
    tree_nodes_model: TreeNodesModel
    behavior_trees: List[BehaviorTree]

    def to_xml(
        self,
        include_ros_pkg: str = "arena_simulation_setup",
        include_path: str = "configs/hunav/behavior_trees/BTRegularNav.xml"
    ) -> ET.Element:
        element = ET.Element(
            "root",
            attrib={
                "main_tree_to_execute": self.main_tree_to_execute,
                "BTCPP_format": self.BTCPP_format
            }
        )

        element.append(self.tree_nodes_model.to_xml())

        element.append(
            ET.Element(
                "include",
                attrib={
                    "ros_pkg": include_ros_pkg,
                    "path": include_path
                }
            )
        )

        for behavior_tree in self.behavior_trees:
            element.append(behavior_tree.to_xml())

        return element


NodeUnion = Annotated[
    Union[ControlNode, DecorationNode, ActionNode, ConditionNode],
    Field(discriminator="ID")
]

if __name__ == "__main__":
    from xml.dom import minidom
    from context import behavior_tree_format

    json_str = behavior_tree_format.strip().strip("Output must strictly follow this structure:").strip("\n    ```json").strip("\n    ```\n    Do NOT explain anything. Output JSON only.")

    test = Root.model_validate_json(json_str)

    xml_str = test.to_xml()
    pretty_bytes = minidom.parseString(
        ET.tostring(xml_str, encoding="UTF-8")
    ).toprettyxml(indent="  ", encoding="UTF-8")

    pretty_str = pretty_bytes.decode("UTF-8")
    print(pretty_str)
