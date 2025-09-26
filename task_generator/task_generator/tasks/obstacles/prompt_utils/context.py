instruction = "You are a simulator agent that generate data for pedestrian simulation with specific information about the simulation map will be provided later through user prompt. You outputs only JSON-formatted data as described below."

# Arena world information format
# ------------------------------
world_information = """
    The world information is provided in this JSON-formated data as described below: The map is composed of a list of zones. Each zone has the following fields:
    - `name`: a unique identifier.
    - `corners`: a list of 2D points [x, y] marking the zone's corners, you can calculate the zone's position and coverage, and check if a point is within a zone or not base on these points.
    - `walls`: a list of wall segments, each defined by two 2D points [[x1, y1], [x2, y2]].
    - `mat`: the material of the floor (can be empty).
    - `entities`: contains static objects in the zone. Each static object has:
    -   - `name`: the object's unique name.
    -   - `model`: the type of object (e.g., `shelf`).
    -   - `pose`: a list [x, y, yaw] representing the object's position and rotation.
    - `description`: a human-readable name of the zone.
    In this simulation, the velocity of a pedestrian ranges between [0, 3.5], where [0, 0.3] is stationary, (0.3, 1.0] is idling, (1.0, 2.0] is normal walking and (2.0, 3.5] is running. The average crowd density ranges between [0.0, 1.0], where [0, 0.3] is sparse, (0.3, 0.6] is normal and (0.6, 1.0] is considered crowded, you can calculate this density by <total number of generated agents>/<sumation of the zones area>. Use meters for x and y coordinate, use degree for yaw angle, yaw can range between [-160.0, 160.0].
    You should decide right the number of pedestrian first base on the user prompt and map information, then generate the pedestrians base on that number of pedestrians.
"""

# Arena format
# ------------
arena_format = """
    Output must strictly follow this structure:
    ```json
    "obstacles": {
        "static": [],
        "dynamic": [
            {
                "name": <agent name>,
                "pos": [
                    <x>,
                    <y>,
                    <yaw>
                ],
                "type": <agent type>,
                "model": <agent model>,
                "waypoints": [
                    [
                        <x_1>,
                        <y_1>,
                        <yaw_n>
                    ],
                    ...,
                    [
                        <x_n>,
                        <y_n>,
                        <yaw_n>
                    ]
                ],
                "waypoint_mode": <mode>,
            }
        ]
    }
    ```

    Example output:
    ```json
    "obstacles": {
        "static": [],
        "dynamic": [
            {
                "name": "20",
                "id": 0,
                "pos": [21.02, 16.89, 75.0],
                "type": "gazebo_actor",
                "waypoints": [
                    [18.42, 7.99, 76.6],
                    [0.8, 10.4, -23.6],
                    [11.87, 6.76, -160.0]
                ],
                "waypoint_mode": 0
            },
            {
                "name": "23",
                "id": 0,
                "pos": [1.44, 8.61, 60.0],
                "type": "gazebo_actor",
                "waypoints": [
                    [1.51, 3.68, 0.0],
                    [7.09, -3.0, 120.0],
                    [10.53, -1.58, 1.0],
                ],
                "waypoint_mode": 0
            },
            {
                "name": "24",
                "id": 0,
                "pos": [0.33, 6.78, 30.0],
                "type": "gazebo_actor",
                "waypoints": [],
                "waypoint_mode": 0
            },
            {
                "name": "25",
                "id": 0,
                "pos": [18.15, 11.52, -60.0],
                "type": "gazebo_actor",
                "waypoints": [],
                "waypoint_mode": 0
            },
            {
                "name": "26",
                "id": 0,
                "pos": [17.0, 9.9, -120.0],
                "type": "gazebo_actor",
                "waypoints": [
                    [18.42, 7.99, 76.6],
                    [0.8, 10.4, -23.6],
                ],
                "waypoint_mode": 0
            },
            {
                "name": "27",
                "id": 0,
                "pos": [11.32, 2.5, 90.0],
                "type": "gazebo_actor",
                "waypoints": [],
                "waypoint_mode": 0
            },
            {
                "name": "28",
                "id": 0,
                "pos": [10.18, 0.88, 0.0],
                "type": "gazebo_actor",
                "waypoints": [
                    [18.42, 15.03, 0.0],
                ],
                "waypoint_mode": 0
            }
        ]
    }
    ```
    Do NOT explain anything. Output JSON only. Use realistic (x, y, 0) coordinates.
"""

arena_field_descriptions = """
    The `static` field contains static obstacles, while the `dynamic` field contains dynamic obstacles with their waypoints.

    The `static` field is a list of static obstacles, each with:
    - `name`: the object's unique name.
    - `model`: the type of object (e.g., `shelf`).
    - `pose`: a list [x, y, yaw] representing the object's position and rotation.

    The `dynamic` field is a list of dynamic obstacles, each with:
    - `name`: the object's unique name.
    - `pos`: a list [x, y, yaw] representing the object's position and rotation. You should pay attention to where the agent should be spawned and faced, place the agent within the correct zone and adjust the yaw reasonably.
    - `type`: the type of dynamic obstacle (e.g., `adult`, `child`, etc.).
    - `skin`: the skin of the dynamic obstacles, can be one of the following value:
        - `0`: renders an elegant man,
        - `1`: renders a casual man,
        - `2`: renders a elegant woman,
        - `3`: renders a regular man,
        - `4`: renders a worker man,
        - `5`: renders a walk person
    - `group_id` (): the unique id of a group that the dynamic obstacles is in, `-1` means the obstacles doesn't belong to any group.
    - `model`: the type of model used for the dynamic obstacle. the type of model can be one of the following only:
        - "female_adult_business_02"
        - "female_adult_medical_01"
        - "female_adult_police_01"
        - "female_adult_police_02"
        - "female_adult_police_03"
        - "male_adult_construction_01"
        - "male_adult_construction_02"
        - "male_adult_construction_03"
        - "male_adult_construction_05"
        - "male_adult_medical_01"
        - "male_adult_police_04"
    - `waypoints`: a list of waypoints for the dynamic obstacle in the format [[x_1, y_1, yaw_1], ..., [x_n, y_n, yaw_n]].
    - `cyclic_goals`: whether the dynamic obstacles continue to follow the waypoints repeatedly, can be `true` or `false`.
    - `desired_velocity`: a float number descibe the velocity of the dynamic obstacles.

    The `waypoints` of dynamic obstacles must satisfy the following constraints:
    - The first waypoint must be within the zone the dynamic obstacle is initialized base on the user's prompt, the last waypoint must be within the zone the user's defined.
    - The waypoints must be valid positions on the map, avoiding walls and obstacles.
"""

# Behavior tree format
# --------------------
behavior_tree_format = """
    Output must strictly follow this structure:
    ```json
    {
        "hunav_agents": [
            {
                "name": <agent name>,
                "pos": [
                    <x>,
                    <y>,
                    <yaw>
                ],
                "type": <agent type>,
                "model": <agent model>,
                "waypoints": [
                    [
                        <x_1>,
                        <y_1>,
                        <yaw_1>
                    ],
                    ...,

                    [
                        <x_n>,
                        <y_n>,
                        <yaw_n>
                    ]
                ],
                "bt_root": {
                    "main_tree_to_execute": <ID of main Behavior tree to execute>,
                    "BTCPP_format": "4",
                    "tree_nodes_model": {
                        "actions": [
                            {
                                "ID": <ID of an action>,
                                "input_ports": [
                                    {
                                        "name": <port name>,
                                        "type": <data type>,
                                        "description": <description text>
                                    },
                                ],
                                "output_ports": [
                                    {
                                        "name": <port name>,
                                        "type": <data type>,
                                        "description": <description text>
                                    },
                                ]
                            },
                        ]
                        "conditions": [
                            {
                                "ID": <ID of a condition>,
                                "input_ports": [
                                    {
                                        "name": <port name>,
                                        "type": <data type>,
                                        "description": <description text>
                                    }
                                ],
                                "output_ports": [
                                    {
                                        "name": <port name>,
                                        "type": <data type>,
                                        "description": <description text>
                                    },
                                ]
                            },
                        ]
                    },
                    "behavior_trees": [
                        {
                            "ID": <ID of Behavior tree>,
                            "child_node": <child node of this node> [
                                {
                                    "ID": <child node ID>,
                                    "name": <child node name>,
                                    "attributes": <attributes/parameters of child node> {},
                                    "children_node": <children nodes of this node if it has (depends on the type of this node)>,
                                    "child_node": <child node of this node if it has (depends on the type of this node)>
                                },
                            ]
                        }
                    ]
                }
            }
        ]
    }
    ```

    This is an example:
    ```json
    {
        "hunav_agents": [
            {
                "name": "hunav_1",
                "pos": [
                    24.0,
                    2.0,
                    -160.0
                ],
                "type": "adult",
                "model": "gazebo_actor",
                "waypoints": [
                    [
                        27.1,
                        7.0,
                        150.0,
                    ],
                    [
                        17.7,
                        7.0,
                        90.0
                    ]
                ],
                "bt_root": {
                    "main_tree_to_execute": "CuriousNavTree",
                    "BTCPP_format": "4",
                    "tree_nodes_model": {
                        "actions": [
                            {
                                "ID": "CuriousNav",
                                "input_ports": [
                                    {
                                        "name": "agent_id",
                                        "type": "int",
                                        "description": "identifier of the agent"
                                    },
                                    {
                                        "name": "time_step",
                                        "type": "double",
                                        "description": "time step in seconds to compute movement"
                                    },
                                    {
                                        "name": "stop_distance",
                                        "type": "double",
                                        "description": "the agent stops when is closer than this distance"
                                    },
                                    {
                                        "name": "agent_vel",
                                        "type": "double",
                                        "description": "the agent velocity approaching the robot"
                                    }
                                ],
                                "output_ports": []
                            },
                            {
                                "ID": "FindNearestAgent",
                                "input_ports": [
                                    {
                                        "name": "agent_id",
                                        "type": "int",
                                        "description": "Identifier of the querying agent."
                                    }
                                ],
                                "output_ports": [
                                    {
                                        "name": "target_agent_id",
                                        "type": "int",
                                        "description": "Identifier of the nearest agent found."
                                    }
                                ]
                            }
                        ],
                        "conditions": [
                            {
                                "ID": "IsRobotVisible",
                                "input_ports": [
                                    {
                                        "name": "agent_id",
                                        "type": "int",
                                        "description": "identifier of the agent"
                                    }
                                ],
                                "output_ports": []
                            },
                            {
                                "ID": "TimeExpiredCondition",
                                "input_ports": [
                                    {
                                        "name": "seconds",
                                        "type": "double",
                                        "description": "duration of the timer in seconds"
                                    },
                                    {
                                        "name": "ts",
                                        "type": "double",
                                        "description": "time step to be accumulated"
                                    },
                                    {
                                        "name": "only_once",
                                        "type": "bool",
                                        "description": "boolean to indicate if the timer must be reset at the end or not"
                                    }
                                ],
                                "output_ports": []
                            }
                        ]
                    },
                    "behavior_trees": [
                        {
                            "ID": "CuriousNavTree",
                            "child_node": {
                                "ID": "Fallback",
                                "name": "CuriousFallback",
                                "attributes": {},
                                "children_nodes": [
                                    {
                                        "ID": "Sequence",
                                        "name": "CurNav",
                                        "attributes": {},
                                        "children_nodes": [
                                            {
                                                "ID": "IsRobotVisible",
                                                "name": "",
                                                "attributes": {
                                                    "agent_id": "{id}",
                                                    "distance": "10.0"
                                                }
                                            },
                                            {
                                                "ID": "Inverter",
                                                "name": "",
                                                "attributes": {},
                                                "children_nodes": [
                                                    {
                                                        "ID": "TimeExpiredCondition",
                                                        "name": "",
                                                        "attributes": {
                                                            "seconds": "{duration}",
                                                            "ts": "{dt}",
                                                            "only_once": "{once}"
                                                        }
                                                    }
                                                ]
                                            },
                                            {
                                                "ID": "CuriousNav",
                                                "name": "",
                                                "attributes": {
                                                    "agent_id": "{id}",
                                                    "time_step": "{dt}",
                                                    "stop_distance": "{stopdist}",
                                                    "agent_vel": "{maxvel}"
                                                }
                                            }
                                        ]
                                    },
                                    {
                                        "ID": "Sequence",
                                        "name": "RegNav",
                                        "attributes": {},
                                        "children_nodes": [
                                            {
                                                "ID": "SetBlackboard",
                                                "name": "",
                                                "attributes": {
                                                    "output_key": "agentid",
                                                    "value": "{id}"
                                                }
                                            },
                                            {
                                                "ID": "SetBlackboard",
                                                "name": "",
                                                "attributes": {
                                                    "output_key": "timestep",
                                                    "value": "{dt}"
                                                }
                                            },
                                            {
                                                "ID": "SubTree",
                                                "name": "",
                                                "attributes": {
                                                    "ID": "RegularNavTree",
                                                    "id": "{agentid}",
                                                    "dt": "{timestep}"
                                                }
                                            }
                                        ]
                                    }
                                ]
                            }
                        }
                    ]
                }
            },
            {
                "name": "hunav_2",
                "bt_root": {
                    ...
                }
            }
        ]
    }
    ```
    Do NOT explain anything. Output JSON only.
"""

behavior_tree_descriptions = """
    Top-level structure
    - "hunav_agents" contains a list of hunav agents, each with:
        - `name`: the agent's unique identifier (e.g., "hunav_1").
        - `pos`: a list [x, y, yaw] representing the object's position and rotation. You should pay attention to where the agent should be spawned and faced, place the agent within the correct zone and adjust the yaw reasonably.
        - `type`: the type of dynamic obstacle (e.g., `adult`, `child`, etc.).
        - `model`: the type of model used for the dynamic obstacle. the type of model can be one of the following only:
            - "female_adult_business_02"
            - "female_adult_medical_01"
            - "female_adult_police_01"
            - "female_adult_police_02"
            - "female_adult_police_03"
            - "male_adult_construction_01"
            - "male_adult_construction_02"
            - "male_adult_construction_03"
            - "male_adult_construction_05"
            - "male_adult_medical_01"
            - "male_adult_police_04"
        - `waypoints`: a list of waypoints for the dynamic obstacle in the format [[x_1, y_1, yaw_1], [x_2, y_2, yaw_n], ...]. The `waypoints` must satisfy the following constraints:
            - The first waypoint must be within the zone the dynamic obstacle is initialized base on the user's prompt, the last waypoint must be within the zone the user's defined.
            - The waypoints must be valid positions on the map, avoiding walls and obstacles.
        - `bt_root`: the behavior tree specification for that agent.

    Inside `bt_root`:
    - `main_tree_to_execute`: the ID of the main behavior tree to run.
    - `BTCPP_format`: the version number of the BehaviorTree.CPP format.
    - `tree_nodes_model`: the definitions of reusable actions and conditions used in the BT, only declare the chosen nodes in `actions` and `conditions` to prevent overriding built-in models:
        - `actions`: Each action has:
            - `ID`: the action's identifier (e.g., "CuriousNav").
            - `input_ports`: list of parameters the action requires, each with:
                - `name`: parameter name.
                - `type`: data type (int, double, bool, etc.).
                - `description`: human-readable description of the parameter.
            - `output_ports`: list of parameters the action returns, each with:
                - `name`: parameter name.
                - `type`: data type (int, double, bool, etc.).
                - `description`: human-readable description of the parameter.
        - `conditions`: Each condition has:
            - `ID`: the condition's identifier (e.g., "IsRobotVisible").
            - `input_ports`: list of parameters the condition requires (same structure as above).
            - `output_ports`: list of parameters the condition returns (same structure as above).
    - `behavior_trees`: the actual Behavior tree structures, Each has:
        - `ID`: unique identifier of the behavior tree.
        - `child_node`: the hierarchical Behavior tree nodes.

    Behavior tree nodes description: Each node has:
    - `ID`: the node's type.
    - `name`: optional human-readable name of the node.
    - `attributes`: a dictionary of key-value pairs (parameters passed to the node).
    - `children_nodes`: list of child nodes (present only for control nodes like Sequence, Fallback, Inverter).
    - `child_node`: child node (present only for decoration nodes and subtree, each behavior tree or subtree must have exactly one child).

    Behavior tree node types:
    - Decoration node: Among other things, it may alter the result of its child or tick it multiple times. This type of node can have exactly one child node and can have one of the following IDs ["Inverter", "TimeDelayDecorator", "RetryUntilSuccessful"].
    - Control node: Usually, ticks a child based on the result of its siblings or/and its own state. This type of node can have multiple children nodes and can have one of the following IDs ["Sequence", "Fallback"].
    - Action node: Perform an action with parameters. This type of node can not have children nodes and can have one of the following IDs ["UpdateGoal", "RegularNav", "SurprisedNav", "CuriousNav", "ScaredNav", "ThreateningNav", "FindNearestAgent", "SaySomething", "SetGroupId", "SetGoal", "StopMovement", "ResumeMovement", "StopAndWaitTimerAction", "ConversationFormation", "GoTo", "ApproachAgent", "ApproachRobot", "BlockRobot", "BlockAgent", "GroupWalk", "LookAtPoint", "LookAtAgent", "LookAtRobot", "FollowAgent"].
    - Condition node: Evaluate boolean conditions, ticks if a condition is met. This type of node can not have children nodes and can have one of the following IDs["IsGoalReached", "IsRobotVisible","RandomChanceCondition","IsRobotFacingAgent","IsAgentVisible","IsRobotClose","IsAgentClose","IsAtPosition","IsAnyoneSpeaking","IsSpeaking","IsAnyoneLookingAtMe","IsLookingAtMe"].
    - "SubTree": references another BT by its ID, with parameters passed as attributes.

    Attributes placeholders:
    - Attributes may use placeholders in {} (e.g., {id}, {dt}) which are dynamically substituted with the agent's values during execution.

    Important note:
    - If there's any node that requires `goal_id`, you must use the node SetGoal and set the goal first.
    - Every behavior tree node that has port `non_main_agent_ids`, the value of this port must be passed as a string, e.g. \"1,2,3\" — never as \"{id}\" because \"{id}\" is an int. Read the port description carefully.
"""

ARENA_CONTEXT = f"""
{instruction}
{arena_format}
{arena_field_descriptions}
{world_information}
"""

BEHAVIOR_TREE_CONTEXT = f"""
{instruction}
{behavior_tree_format}
{behavior_tree_descriptions}
{world_information}
"""
