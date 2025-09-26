import os
import time
import traceback

import attrs
import geometry_msgs.msg
import numpy as np
from ament_index_python.packages import get_package_share_directory
from arena_people_msgs.msg import Pedestrian, Pedestrians
from arena_people_msgs.srv import DeleteActors
from arena_rclpy_mixins.shared import Namespace
from geometry_msgs.msg import Point
from hunav_msgs.msg import Agent, AgentBehavior, Agents, WallSegment
from hunav_msgs.srv import ComputeAgent, ComputeAgents, GetAgents, GetWalls, MoveAgent
from std_srvs.srv import Trigger

from task_generator.constants import Constants
from task_generator.shared import (
    Model,
    ModelType,
    ModelWrapper,
    Obstacle,
    Orientation,
    Pose,
    Position,
)
from task_generator.simulators.human.dummy import DummyHumanSimulator
from task_generator.simulators.sim import BaseSim

from . import HunavDynamicObstacle


def _create_robot_message():
    """Creates a standard robot message for HuNav communication"""
    robot_msg = Agent()
    robot_msg.id = 0
    robot_msg.name = "robot"
    robot_msg.type = Agent.ROBOT
    robot_msg.position.position.x = 0.0
    robot_msg.position.position.y = 0.0
    robot_msg.yaw = 0.0
    robot_msg.radius = 0.3
    return robot_msg


class _PedestrianHelper:

    _ANIMATION_MAP = {
        AgentBehavior.BEH_REGULAR: "07_01-walk.bvh",
        AgentBehavior.BEH_IMPASSIVE: "69_02_walk_forward.bvh",
        AgentBehavior.BEH_SURPRISED: "137_28-normal_wait.bvh",
        AgentBehavior.BEH_SCARED: "142_17-walk_scared.bvh",
        AgentBehavior.BEH_CURIOUS: "07_04-slow_walk.bvh",
        AgentBehavior.BEH_THREATENING: "17_01-walk_with_anger.bvh"
    }

    _SKIN_TYPES = {
        0: 'elegant_man.dae',
        1: 'casual_man.dae',
        2: 'elegant_woman.dae',
        3: 'regular_man.dae',
        4: 'worker_man.dae',
        5: 'walk.dae'
    }

    _HEIGHTS = {
        0: 0.96,  # Elegant man
        1: 0.97,  # Casual man
        2: 0.93,  # Elegant woman
        3: 0.93,  # Regular man
        4: 0.97,  # Worker man
        5: 1.05,  # Balds
        6: 1.05,
        7: 1.05,
        8: 1.05
    }

    @classmethod
    def hunav_plugin_entity(cls, namespace: str) -> Obstacle:

        sdf_content = f"""<?xml version="1.0" ?>
            <sdf version="1.9">
                <model name="hunav_plugin">
                    <static>true</static>
                    <link name="empty">
                        <visual name="visual">
                            <geometry>
                                <box>
                                    <size>0.01 0.01 0.01</size>
                                </box>
                            </geometry>
                        </visual>
                    </link>
                    <plugin name="HuNavSystemPluginIGN" filename="libHuNavSystemPluginIGN.so">
                        <update_rate>1000.0</update_rate>
                        <namespace>{namespace}</namespace>
                        <!-- <robot_name>jackal</robot_name> -->
                        <use_gazebo_obs>true</use_gazebo_obs>
                        <global_frame_to_publish>map</global_frame_to_publish>
                        <use_navgoal_to_start>false</use_navgoal_to_start>
                        <navgoal_topic>goal_pose</navgoal_topic>
                        <ignore_models>
                            <model>ground_plane</model>
                            <model>sun</model>
                        </ignore_models>
                    </plugin>
                </model>
            </sdf>"""

        return Obstacle(
            name="hunav_plugin",
            pose=Pose(Position(x=0.0, y=0.0, z=-1.0)),
            model=ModelWrapper.Constant("hunav_plugin", {
                ModelType.SDF: Model(
                    type=ModelType.SDF,
                    name="hunav_plugin",
                    description=sdf_content,
                    path="",
                )
            })
        )

    @classmethod
    def plugin_entity(cls, namespace: str) -> Obstacle:

        sdf_content = f"""<?xml version="1.0" ?>
            <sdf version="1.9">
                <model name="human_plugin">
                    <static>true</static>
                    <link name="empty">
                        <visual name="visual">
                            <geometry>
                                <box>
                                    <size>0.01 0.01 0.01</size>
                                </box>
                            </geometry>
                        </visual>
                    </link>
                    <plugin name="HumanSystemPlugin" filename="libHumanSystemPlugin.so">
                        <update_rate>1000.0</update_rate>
                        <namespace>{namespace}</namespace>
                        <global_frame_to_publish>map</global_frame_to_publish>
                        <pedestrians_topic>arena_peds</pedestrians_topic>
                    </plugin>
                </model>
            </sdf>"""

        return Obstacle(
            name="human_plugin",
            pose=Pose(Position(x=0.0, y=0.0, z=-1.0)),
            model=ModelWrapper.Constant("human_plugin", {
                ModelType.SDF: Model(
                    type=ModelType.SDF,
                    name="human_plugin",
                    description=sdf_content,
                    path="",
                )
            })
        )

    @classmethod
    def create_sdf(cls, agent_config: HunavDynamicObstacle) -> str:
        """Create SDF description for pedestrian using gz-sim actor format"""

        # Get skin type
        skin_type = cls._SKIN_TYPES.get(agent_config.skin, 'casual_man.dae')

        # Animation mapping based on behavior
        animation_file = cls._ANIMATION_MAP.get(agent_config.behavior.type, "07_01-walk.bvh")
        animation_file = '../models/walk.dae'  # temp

        # Construct paths
        mesh_path = os.path.join(
            get_package_share_directory('hunav_rviz2_panel'),
            'meshes/models',
            skin_type
        )

        animation_path = os.path.join(
            get_package_share_directory('hunav_rviz2_panel'),
            'meshes/animations',
            animation_file
        )

        # # Temporärer Logger für Debug
        # import rclpy
        # if not rclpy.ok():
        #     rclpy.init()
        # temp_node = rclpy.create_node('temp_debug_node')
        # logger = temp_node.get_logger()

        #         # DEBUG
        # logger.warn(f"DEBUG: Looking for animation at: {animation_path}")
        # logger.warn(f"DEBUG: Animation exists: {os.path.exists(animation_path)}")

        # # Cleanup
        # temp_node.destroy_node()

        # Create the SDF
        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.9">
            <actor name="{agent_config.name}">
                <pose>{agent_config.init_pose.x} {agent_config.init_pose.y} {cls._HEIGHTS.get(agent_config.skin, 1.0)} 0 0 {agent_config.yaw}</pose>

                <skin>
                    <filename>{mesh_path}</filename>
                    <scale>1.0</scale>
                </skin>

                <animation name="walking">
                    <filename>{animation_path}</filename>
                    <scale>1.0</scale>
                    <interpolate_x>true</interpolate_x>
                </animation>
            </actor>
        </sdf>"""
        return sdf


class HunavHumanSimulator(DummyHumanSimulator):
    """HunavManager with debug logging for tracking execution flow"""

    _pedestrians: dict[int, dict]
    _agents_container: Agents
    _get_agents_container: Agents

    # Service Names
    SERVICE_COMPUTE_AGENT = 'compute_agent'
    SERVICE_COMPUTE_AGENTS = 'compute_agents'
    SERVICE_MOVE_AGENT = 'move_agent'
    SERVICE_CLEAR_AGENTS = 'clear_agents'
    SERVICE_GET_AGENTS = 'get_agents'
    SERVICE_GET_WALLS = 'get_walls'
    SERVICE_DELETE_ACTORS = 'delete_actors'

    def __init__(self, namespace: Namespace, simulator: BaseSim):
        """Initialize HunavManager with debug logging"""
        super().__init__(namespace=namespace, simulator=simulator)
        # Detect Simulator Type to decide between Plugin or move_entity callback
        self._logger.info(f"Detected simulator type: {self._simulator_type}")

        self._logger.info("=== HUNAVMANAGER INIT START ===")
        self._logger.debug("Parent class initialized")

        # Initialize collections
        self._logger.debug("Initializing collections...")
        self._pedestrians = {}
        self._wall_segments: list[WallSegment] = []
        self._wall_points: list[Point] = []
        self._agents_container = Agents()  # Container to hold all registered agents
        self._get_agents_container = Agents()  # Container specifically just to send the Agent attributes to Hunavsystemplugin
        self._agents_container.header.frame_id = "map"
        self._arena_pedestrians_container = Pedestrians()
        self._arena_pedestrians_container.header.frame_id = "map"
        self._logger.debug("Collections initialized")

        self._obstacle_subscriber = self.node.create_subscription(
            Agents,
            '/task_generator_node/hunav_closest_obstacles',
            self._obstacle_callback,
            10
        )
        # Setup services
        self._logger.debug("Setting up services...")
        setup_success = self._setup_services()
        if not setup_success:
            self._logger.error("Service setup failed!")
        else:
            self._logger.info("Services setup complete")
        arena_peds_success = self._setup_arena_peds_publisher()
        if not arena_peds_success:
            self._logger.error("Arena peds publisher setup failed!")
        else:
            self._logger.info("Arena peds publisher setup complete")

            # Setup obstacle subscriber
        if not self._setup_obstacle_subscriber():
            self._logger.error("Failed to setup obstacle subscriber")

        self._logger.debug("Waiting for services to be ready...")
        time.sleep(2.0)
        self._logger.debug("Service wait complete")

        self._logger.info("=== HUNAVMANAGER INIT COMPLETE ===")

        self._gz_plugin_spawned: bool = False
        self._last_updated_agents = None
        self._last_smooth_yaws = {}
        # Orientation smoothing (wie Isaac Wrapper)
        self._agent_previous_orientations = {}
        self._orientation_smoothing_factor = 0.15  # 0.05-0.3 range

    @property
    def _simulator_type(self) -> Constants.SimSimulator:
        """Detect which simulator is being used"""
        return self.node.conf.Arena.SIM.value

    def _setup_services(self):
        """Initialize all required services with debug logging"""
        self._logger.info("=== SETUP_SERVICES START ===")

        # Debug namespace information
        self._logger.debug(f"Node namespace: {self.node.get_namespace()}")
        self._logger.debug(f"Task generator namespace: {self._namespace}")

        # Create service names with full namespace path
        class service_names:
            compute_agent = self.node.service_namespace(self.SERVICE_COMPUTE_AGENT)
            compute_agents = self.node.service_namespace(self.SERVICE_COMPUTE_AGENTS)
            move_agent = self.node.service_namespace(self.SERVICE_MOVE_AGENT)
            clear_agents = self.node.service_namespace(self.SERVICE_CLEAR_AGENTS)
            get_agents = self.node.service_namespace(self.SERVICE_GET_AGENTS)
            get_walls = self.node.service_namespace(self.SERVICE_GET_WALLS)
            delete_actors = self.node.service_namespace(self.SERVICE_DELETE_ACTORS)

        # Create service clients
        self._logger.debug("Creating compute_agent client...")
        self._compute_agent_client = self.node.create_client(
            ComputeAgent,
            service_names.compute_agent
        )

        self._logger.debug("Creating compute_agents client...")
        self._compute_agents_client = self.node.create_client(
            ComputeAgents,
            service_names.compute_agents
        )

        self._logger.debug("Creating move_agent client...")
        self._move_agent_client = self.node.create_client(
            MoveAgent,
            service_names.move_agent,
        )

        self._logger.debug("Creating clear_agents client...")
        self._clear_agents_client = self.node.create_client(
            Trigger,
            service_names.clear_agents,
        )

        self._logger.debug("Creating delete_actors client...")
        self._delete_actors_client = self.node.create_client(
            DeleteActors,
            service_names.delete_actors,
        )
       # Create GetAgents service provider
        self._logger.debug(f"Creating get_agents service provider at: {service_names.get_agents}")
        self._get_agents_server = self.node.create_service(
            GetAgents,
            service_names.get_agents,
            self._get_agents_callback,
        )
        self._logger.debug("GetAgents service provider created")

        # Create GetWalls service provider
        self._logger.debug(f"Creating get_walls service provider at: {service_names.get_walls}")
        self._get_walls_service = self.node.create_service(
            GetWalls,
            service_names.get_walls,
            self._get_walls_callback,
        )
        self._logger.debug("GetWalls service provider created")

        # Wait for Services
        required_services = [
            (self._compute_agent_client, service_names.compute_agent),
            (self._compute_agents_client, service_names.compute_agents),
            (self._move_agent_client, service_names.move_agent),
            (self._clear_agents_client, service_names.clear_agents),

        ]

        max_attempts = float('inf')
        for client, service in required_services:
            attempts = 0
            self._logger.debug(f"Waiting for service {service}...")

            while attempts < max_attempts:
                if client.wait_for_service(timeout_sec=2.0):
                    self._logger.debug(f'Service {service} is available')
                    break
                attempts += 1
                self._logger.debug(
                    f'Waiting for service {service} (attempt {attempts}/{max_attempts})\n'
                    f'Looking for service at: {service}'
                )

            if attempts >= max_attempts:
                self._logger.error(
                    f'Service {service} not available after {max_attempts} attempts\n'
                    f'Was looking for service at: {service}'
                )
                self._logger.error("=== SETUP_SERVICES FAILED ===")
                return False

        self._logger.info("=== SETUP_SERVICES COMPLETE ===")
        return True

    def _setup_arena_peds_publisher(self):
        """Setup arena_peds publisher and timer - separate from services"""
        try:
            self._logger.info("=== ARENA PEDS PUBLISHER SETUP START ===")

            # Create publisher
            self._arena_peds_publisher = self.node.create_publisher(
                Pedestrians,
                self._namespace('arena_peds'),
                10
            )

            # Create timer
            self._arena_peds_timer = self.node.create_timer(
                0.1,  # 10 Hz
                self._publish_arena_peds_callback
            )

            self._logger.info("=== ARENA PEDS PUBLISHER SETUP COMPLETE ===")
            return True

        except Exception as e:
            self._logger.error(f"Arena peds publisher setup failed: {e}")
            return False

    def _setup_obstacle_subscriber(self):
        """Setup obstacle subscriber for closest_obs from HuNavSystemPlugin"""
        try:
            self._logger.info("=== OBSTACLE SUBSCRIBER SETUP START ===")

            # Create subscriber
            obstacle_topic = self._namespace('hunav_closest_obstacles')
            self._obstacle_subscriber = self.node.create_subscription(
                Agents,
                obstacle_topic,
                self._obstacle_callback,
                10
            )

            # Store latest obstacle data
            self._latest_obstacles = {}

            self._logger.info(f"Subscribed to {obstacle_topic}")
            self._logger.info("=== OBSTACLE SUBSCRIBER SETUP COMPLETE ===")
            return True

        except Exception as e:
            self._logger.error(f"Obstacle subscriber setup failed: {e}")
            return False

    def _obstacle_callback(self, msg):
        """Store latest obstacle data from HuNavSystemPlugin"""
        try:
            self._latest_obstacles.clear()

            for obs_agent in msg.agents:
                self._latest_obstacles[obs_agent.name] = obs_agent.closest_obs

            self._logger.debug(f"Updated obstacle data for {len(self._latest_obstacles)} agents")

        except Exception as e:
            self._logger.error(f"Error in obstacle callback: {e}")

    def _update_agent_obstacles(self, current_agents):
        """Update agent closest_obs with latest obstacle data before HuNav call"""
        if not self._latest_obstacles:
            return

        for agent in current_agents.agents:
            if agent.name in self._latest_obstacles:
                agent.closest_obs = self._latest_obstacles[agent.name]
                agent.closest_obs.extend(self._wall_points)
                self._logger.debug(f"Updated agent {agent.name} with {len(agent.closest_obs)} obstacles")
                self._logger.debug(f"Wall Points: {self._wall_points}")

    def _get_agents_callback(self, request, response):
        """Handle get_agents service request - return UNMODIFIED agents"""
        try:
            self._logger.debug("=== GET AGENTS CALLBACK ===")
            self._logger.debug(f"Returning {len(self._get_agents_container.agents)} agents")

            # Update timestamp
            self._get_agents_container.header.stamp = self.node.get_clock().now().to_msg()
            self._get_agents_container.header.frame_id = "map"

            # Return the UNMODIFIED container
            response.agents = self._get_agents_container

            # Debug output
            # for agent in self._get_agents_container.agents:
            #     self._logger.debug(f"Sending agent {agent.name}: desired_velocity={agent.desired_velocity}, goal_force_factor={agent.behavior.goal_force_factor}")

            return response

        except Exception as e:
            self._logger.error(f"Error in get_agents_callback: {e}")
            response.agents = Agents()  # Empty response on error
            return response

    def _get_walls_callback(self, request, response):
        """Service callback für Wall-Segments"""
        response.walls = self._wall_segments
        self._logger.debug(f"Sent {len(self._wall_segments)} wall segments")
        return response

    def _move_entity_callback(self):
        """Pedestrian Move Entity Callback for non gazebo simulators"""

        try:
            self._simulator.pedestrian_update(self._arena_pedestrians_container)

        except Exception as e:
            self._logger.error(f"Failed to update agent positions: {e}")

    def _spawn_dynamic_obstacles_impl(self, obstacles):

        results = []

        for obstacle in obstacles:
            try:
                # Get unique ID
                unique_id = len(self._agents_container.agents) + 1
                self._logger.debug(f"Preparing to spawn dynamic obstacle '{obstacle.name}' with ID {unique_id}")
                hunav_obstacle = HunavDynamicObstacle.from_dynamic_obstacle(obstacle)
                hunav_obstacle = attrs.evolve(hunav_obstacle, id=unique_id)

                agent_msg = self._create_agent_msg(hunav_obstacle)

                # Add to container - NO ComputeAgents call here!
                self._get_agents_container.agents.append(agent_msg)
                self._agents_container.agents.append(agent_msg)
                # self._logger.error(f"spawn_dynamic_obstacle_agents_container {self._agents_container}")

                # Create separate arena pedestrian
                arena_pedestrian = self._create_arena_pedestrian(hunav_obstacle, unique_id)
                self._arena_pedestrians_container.pedestrians.append(arena_pedestrian)
                self._logger.debug(f"Added arena pedestrian {arena_pedestrian.name} - Total: {len(self._arena_pedestrians_container.pedestrians)}")

                # Store in pedestrians dictionary
                self._pedestrians[agent_msg.id] = {
                    'last_update': time.time(),
                    'current_state': agent_msg.behavior.state,
                    'agent': agent_msg,
                    'animation_time': 0.0
                }

                self._logger.debug(f"Added agent {agent_msg.name} to container. Total agents: {len(self._agents_container.agents)}")

                if self._simulator_type == Constants.SimSimulator.GAZEBO:
                    # spawn plugin if not already spawned
                    if not self._gz_plugin_spawned:
                        self._simulator.obstacle_spawn((_PedestrianHelper.plugin_entity(self.node.service_namespace()),))
                        self._simulator.obstacle_spawn((_PedestrianHelper.hunav_plugin_entity(self.node.service_namespace()),))
                        self._gz_plugin_spawned = True

                    # Create SDF with plugin for Gazebo
                    sdf = _PedestrianHelper.create_sdf(hunav_obstacle)
                    obstacle.model = obstacle.model.override(
                        ModelType.SDF,
                        lambda model: model.replace(description=sdf), noload=True
                    )
                    obstacle.pose.orientation = Orientation.from_yaw(hunav_obstacle.yaw)
                    self._logger.info(f"Created SDF and loaded System Plugin for: {agent_msg.name}")
                else:
                    # For other simulators: use simple model without plugin
                    self._logger.info(f"Using simple spawning for simulator: {self._simulator_type}")
                results.append(obstacle)

            except Exception as e:
                self._logger.error(f"Error preparing agent: {str(e)}")
                self._logger.error(traceback.format_exc())
                results.append(None)

        # Now all obstacles have been prepared - register them with HuNav

        if self._agents_container.agents:
            self._logger.debug(f"All spawns complete. Registering {len(self._agents_container.agents)} agents with HuNav")

            # Update timestamp
            self._agents_container.header.stamp = self.node.get_clock().now().to_msg()

            # Create request
            request = ComputeAgents.Request()
            request.robot = _create_robot_message()
            request.current_agents = self._agents_container

            # Call HuNav service
            response = self._compute_agents_client.call(request)

            if response:
                self._logger.debug(f"Successfully registered {len(response.updated_agents.agents)} agents")

                # # Update local agents with response data
                # for updated_agent in response.updated_agents.agents:

                #     for i, agent in enumerate(self._agents_container.agents):
                #         if agent.id == updated_agent.id:
                #             self._agents_container.agents[i] = updated_agent
                #             break

                #     # Update pedestrians dictionary if exists
                #     if updated_agent.id in self._pedestrians:
                #         self._pedestrians[updated_agent.id]['agent'] = updated_agent

                if self._simulator_type != Constants.SimSimulator.GAZEBO:
                    self._logger.debug("Non-Gazebo detected - starting movement timer")
                    self._update_timer = self.node.create_timer(
                        0.1,  # 10 Hz
                        self._move_entity_callback
                    )
            else:
                self._logger.error("Failed to register agents with HuNav")
        else:
            self._logger.debug("No agents to register")

        return results

    def _wall_to_points(self, start: Position, end: Position, spacing: float = 0.01) -> list[Point]:
        points: list[Point] = []
        v = (end - start).normalized()
        for i in np.arange(0, (end - start).norm(), spacing):
            points.append(start + v * i)
        points.append(end)
        return points

    def _spawn_walls_impl(self, walls) -> bool:

        self._wall_segments = []
        self._wall_points = []

        for i, wall in enumerate(walls):
            segment = WallSegment()
            segment.id = i
            segment.start = wall.start.to_msg()
            segment.end = wall.end.to_msg()
            segment.length = (wall.start - wall.end).norm()

            self._wall_segments.append(segment)
            self._wall_points.extend(self._wall_to_points(wall.start, wall.end))

        self._logger.debug(f"Cached {len(self._wall_segments)} wall segments")
        self._logger.debug(f"Wallsegments{self._wall_segments} ")
        return True

    def _remove_obstacles_impl(self):
        """Remove all spawned pedestrians from simulation safely"""
        self._logger.debug(f"=== REMOVING {len(self._pedestrians)} PEDESTRIANS ===")

        # Phase 1: Delete Actors from ECM first
        success = self._call_delete_actors_service()

        if not success:
            self._logger.info("Failed to delete  Pedestrians from ECM - continuing anyway")
            # Don't return False - continue with deletion

        # Phase 2: Clear local agents container
        self._agents_container = Agents()
        self._get_agents_container = Agents()
        self._logger.debug("Cleared local agents container")

        # Phase 3: Reset HunavSim
        success = self._clear_hunav_agents()
        if not success:
            self._logger.error("Failed to reset HuNav agents - continuing anyway")

        # Phase 4: Clean up local data
        self._clear_local_data()

        self._logger.debug(f"Complete reset completed: {success}")
        return success

    def _clear_hunav_agents(self):
        """Reset HuNav by calling ClearAgents service"""
        try:
            if not self._clear_agents_client.wait_for_service(timeout_sec=2.0):
                self._logger.error("ClearAgents service not available")
                return False

            request = Trigger.Request()

            self._logger.debug("Calling HuNav ClearAgents service...")
            response = self._clear_agents_client.call(request)

            if response and response.success:
                self._logger.debug("HuNav clear successful - ready for new agents")
                return True
            else:
                self._logger.error("HuNav clear failed")
                return False

        except Exception as e:
            self._logger.error(f"Error calling ClearAgents: {e}")
            return False

    def _call_delete_actors_service(self):
        """Call the plugin's delete actors service"""
        try:
            if not self._delete_actors_client.wait_for_service(timeout_sec=2.0):
                self._logger.error("Delete actors service currently not available")
                return False

            request = DeleteActors.Request()

            self._logger.error("Calling delete_actors service...")

            response = self._delete_actors_client.call(request)

            if response and response.success:
                self._logger.debug(f"Successfully deleted {response.deleted_count} actors")
                return True
            else:
                self._logger.error("Delete actors service failed")
                return False

        except Exception as e:
            self._logger.error(f"Error calling delete_actors service: {e}")
            return False

    def _clear_local_data(self):
        """Clear all local data structures safely"""
        # Clear pedestrians dictionary
        self._pedestrians.clear()

        # Clear agents container (if not already done)
        self._agents_container.agents.clear()
        self._arena_pedestrians_container.pedestrians.clear()

        self._last_updated_agents = None
        self._last_smooth_yaws = {}
        self._agent_previous_orientations = {}

        # Stop and cleanup movement timer if running (for non-gazebo simulators)
        if hasattr(self, '_update_timer') and self._update_timer:
            try:
                self._update_timer.destroy()
                self._update_timer = None
                self._logger.debug("Movement timer stopped and cleaned up")
            except Exception as e:
                self._logger.error(f"Error stopping movement timer: {e}")

        # if hasattr(self, '_arena_peds_timer') and self._arena_peds_timer:
        #     self._arena_peds_timer.destroy()
        #     self._arena_peds_timer = None

        self._logger.debug("All local data structures cleared")

    def _create_agent_msg(self, hunav_obstacle: HunavDynamicObstacle) -> Agent:
        self._logger.debug(f"Preparing agent {hunav_obstacle.name} (ID: {hunav_obstacle.id})")

        # Create agent message
        agent_msg = hunav_obstacle.to_msg()

        # Add wall obstacles
        # self._logger.debug(f"Hunav Manager Wallpoints: {self._wall_points}")
        # agent_msg.closest_obs.extend(self._wall_points)
        # self._logger.debug(f"Hunav Manager Closest Obstacles: {agent_msg.closest_obs}")

        # After creating the agent message:
        self._logger.debug(f"""            ##Complete Debug for the set attributes
                Full HunavObstacle Details:
                ID: {agent_msg.id}
                Name: {agent_msg.name}
                Type: {agent_msg.type}
                Skin: {agent_msg.skin}
                Group ID: {agent_msg.group_id}

                Position:
                - X: {agent_msg.position.position.x}
                - Y: {agent_msg.position.position.y}
                - Z: {agent_msg.position.position.z}
                - Yaw: {agent_msg.yaw}

                Velocities:
                - Desired: {agent_msg.desired_velocity}
                - Linear: {agent_msg.linear_vel}
                - Angular: {agent_msg.angular_vel}

                Physical:
                - Radius: {agent_msg.radius}

                Behavior:
                - Type: {agent_msg.behavior.type}
                - Configuration: {agent_msg.behavior.configuration}
                - Duration: {agent_msg.behavior.duration}
                - Once: {agent_msg.behavior.once}
                - Velocity: {agent_msg.behavior.vel}
                - Distance: {agent_msg.behavior.dist}
                - Goal Force: {agent_msg.behavior.goal_force_factor}
                - Obstacle Force: {agent_msg.behavior.obstacle_force_factor}
                - Social Force: {agent_msg.behavior.social_force_factor}
                - Other Force: {agent_msg.behavior.other_force_factor}

                Goals:
                - Count: {len(agent_msg.goals)}
                - Cyclic: {agent_msg.cyclic_goals}
                - Radius: {agent_msg.goal_radius}
                - Goals List: {[f'({g.position.x}, {g.position.y})' for g in agent_msg.goals]}
                """)

        return agent_msg

    def _create_arena_pedestrian(self, hunav_obstacle: HunavDynamicObstacle, unique_id: int) -> Pedestrian:
        """Create arena_people_msgs.Pedestrian (separate from hunav)"""

        arena_ped = Pedestrian()

        arena_ped.name = hunav_obstacle.name
        arena_ped.id = unique_id

        arena_ped.pose.position.x = hunav_obstacle.init_pose.x
        arena_ped.pose.position.y = hunav_obstacle.init_pose.y
        arena_ped.pose.position.z = 1.25

        arena_ped.pose.orientation = Orientation.from_yaw(hunav_obstacle.yaw).to_msg()

        # Initial twist (zero at spawn)
        arena_ped.twist.linear.x = 0.0
        arena_ped.twist.linear.y = 0.0
        arena_ped.twist.angular.z = 0.0

        # Animation state from behavior
        arena_ped.animation_state = self._map_hunav_behavior_to_arena_state(hunav_obstacle.behavior.type)

        self._logger.debug(f"Created arena pedestrian: {arena_ped.name}")
        return arena_ped

    def _map_hunav_behavior_to_arena_state(self, behavior_type: int) -> int:
        """Map hunav behavior to arena animation state"""

        # Import AgentBehavior constants
        from hunav_msgs.msg import AgentBehavior

        behavior_mapping = {
            AgentBehavior.BEH_REGULAR: Pedestrian.WALKING,
            AgentBehavior.BEH_IMPASSIVE: Pedestrian.IDLE,
            AgentBehavior.BEH_SURPRISED: Pedestrian.SURPRISED,
            AgentBehavior.BEH_SCARED: Pedestrian.PANIC,
            AgentBehavior.BEH_CURIOUS: Pedestrian.CURIOUS,
            AgentBehavior.BEH_THREATENING: Pedestrian.THREATENING,
        }

        return behavior_mapping.get(behavior_type, Pedestrian.WALKING)

    def _publish_arena_peds_callback(self):
        """Use last updated agents as current agents (like Plugin does)"""

        self._logger.debug(f"arena_peds_callback: publishing {len(self._arena_pedestrians_container.pedestrians)} pedestrians")

        if not self._arena_pedestrians_container.pedestrians:
            return

        try:
            # Use last updated agents as current agents
            if self._last_updated_agents:
                current_agents = self._last_updated_agents
            else:
                current_agents = self._agents_container

            # Ensure frame_id is set
            current_agents.header.frame_id = "map"
            current_agents.header.stamp = self.node.get_clock().now().to_msg()

            # Update obstacles BEFORE sending to HuNav
            self._update_agent_obstacles(current_agents)

            # Smooth yaw values before sending to HuNav
            current_agents = self._smooth_agents_before_hunav(current_agents)

            # Create request
            request = ComputeAgents.Request()
            request.current_agents = current_agents
            request.robot = _create_robot_message()

            response = self._compute_agents_client.call(request)

            if response and response.updated_agents:
                # Fix frame_id
                response.updated_agents.header.frame_id = "map"
                response.updated_agents.header.stamp = self.node.get_clock().now().to_msg()

                self._last_updated_agents = response.updated_agents

                self._logger.debug(f"Updated agents: {self._last_updated_agents}")

                # Update arena pedestrians
                for arena_ped in self._arena_pedestrians_container.pedestrians:
                    for updated_agent in response.updated_agents.agents:
                        if updated_agent.id == arena_ped.id:

                            calculated_vel_x, calculated_vel_y = self._calculate_velocity_from_position_change(updated_agent, arena_ped)

                            arena_ped.pose = self._round_coordinates(updated_agent.position, 2)

                            arena_ped.twist.linear.x = updated_agent.velocity.linear.x
                            arena_ped.twist.linear.y = updated_agent.velocity.linear.y
                            arena_ped.twist.linear.z = 0.0

                            arena_ped.twist.angular.x = 0.0
                            arena_ped.twist.angular.y = 0.0
                            arena_ped.twist.angular.z = 0.0

                            import math

                            # Orientation through CALCULATED VELOCITY!
                            vel_x = calculated_vel_x
                            vel_y = calculated_vel_y
                            velocity_magnitude = math.sqrt(vel_x**2 + vel_y**2)

                            if velocity_magnitude > 0.05:
                                target_yaw = math.atan2(vel_y, vel_x)
                            else:
                                target_yaw = updated_agent.yaw

                            smoothed_yaw = self._smooth_yaw_slerp(target_yaw, arena_ped.id)

                            arena_ped.pose.orientation = Orientation.from_yaw(smoothed_yaw).to_msg()

                            updated_agent.velocity.linear.x = calculated_vel_x
                            updated_agent.velocity.linear.y = calculated_vel_y
                            updated_agent.yaw = smoothed_yaw
                            updated_agent.position.orientation = Orientation.from_yaw(smoothed_yaw).to_msg()

                            break

            # Publish
            for ped in self._arena_pedestrians_container.pedestrians:
                self._logger.debug(f"Publishing pedestrian {ped.name} at ({ped.pose.position.x}, {ped.pose.position.y}) with velocity ({ped.twist.linear.x}, {ped.twist.linear.y})")

            self._arena_peds_publisher.publish(self._arena_pedestrians_container)

        except Exception as e:
            self._logger.error(f"Error: {e}")
            import traceback
            self._logger.error(traceback.format_exc())

    def _smooth_yaw(self, new_yaw, current_yaw):
        import math

        def normalize_angle(angle):
            return math.atan2(math.sin(angle), math.cos(angle))

        new_yaw = normalize_angle(new_yaw)
        current_yaw = normalize_angle(current_yaw)
        diff = normalize_angle(new_yaw - current_yaw)

        if abs(diff) > math.radians(25):
            return normalize_angle(current_yaw + (diff * 0.1))
        else:
            return current_yaw

    def _smooth_agents_before_hunav(self, agents):
        """Yaw smoothing before sending back to hunav"""
        for agent in agents.agents:
            if agent.id in self._last_smooth_yaws:
                agent.yaw = self._smooth_yaw(agent.yaw, self._last_smooth_yaws[agent.id])
            self._last_smooth_yaws[agent.id] = agent.yaw
        return agents

    def _round_coordinates(self, pose: geometry_msgs.msg.Pose, decimals=2):
        """Round coordinates to avoid floating point errors"""
        pose.position.x = round(pose.position.x, decimals)
        pose.position.y = round(pose.position.y, decimals)
        pose.position.z = round(pose.position.z, decimals)
        return pose

    def _calculate_movement_yaw(self, agent):
        import math

        # Velocity-basierte OrientTION
        vel_x = agent.velocity.linear.x
        vel_y = agent.velocity.linear.y

        # Only if the Agent moves
        velocity_magnitude = math.sqrt(vel_x**2 + vel_y**2)
        if velocity_magnitude > 0.05:
            movement_yaw = math.atan2(vel_y, vel_x)
            return movement_yaw
        else:
            # If no Movement, use same Orientation dont change it
            return None

    def _calculate_velocity_from_position_change(self, updated_agent, arena_ped, dt=0.1):
        """Calculate Position from the velocity change"""
        import math

        # Previous position
        prev_x = arena_ped.pose.position.x
        prev_y = arena_ped.pose.position.y

        # Current position
        curr_x = updated_agent.position.position.x
        curr_y = updated_agent.position.position.y

        vel_x = (curr_x - prev_x) / dt
        vel_y = (curr_y - prev_y) / dt

        # Speed limiting
        velocity_magnitude = math.sqrt(vel_x**2 + vel_y**2)
        if velocity_magnitude > updated_agent.desired_velocity:

            scale_factor = updated_agent.desired_velocity / velocity_magnitude
            vel_x *= scale_factor
            vel_y *= scale_factor

        return vel_x, vel_y

    def _slerp_quaternions(self, q1_list, q2_list, t):
        """Spherical linear interpolation between two quaternions"""
        import math

        # Manual quaternion normalization
        def normalize_quat(q):
            norm = math.sqrt(sum(x * x for x in q))
            return [x / norm for x in q] if norm > 0 else [0, 0, 0, 1]

        # Convert lists to normalized quaternions
        q1 = normalize_quat(q1_list)
        q2 = normalize_quat(q2_list)

        # Calculate dot product
        dot = sum(a * b for a, b in zip(q1, q2))

        # If dot product is negative, negate one quaternion for shorter path
        if dot < 0.0:
            q2 = [-x for x in q2]
            dot = -dot

        # If quaternions are very close, use linear interpolation
        if dot > 0.9995:
            result = [q1[i] + t * (q2[i] - q1[i]) for i in range(4)]
            return normalize_quat(result)

        # Calculate spherical interpolation
        theta_0 = math.acos(abs(dot))
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * t
        sin_theta = math.sin(theta)

        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        result = [s0 * q1[i] + s1 * q2[i] for i in range(4)]
        return normalize_quat(result)

    def _smooth_yaw_slerp(self, target_yaw, agent_id):
        """Smooth yaw transitions to avoid orientation errors of Hunavs inner calculations"""
        import math

        def normalize_angle(angle):
            return math.atan2(math.sin(angle), math.cos(angle))

        target_yaw = normalize_angle(target_yaw)

        if agent_id in self._agent_previous_orientations:
            prev_yaw = self._agent_previous_orientations[agent_id]
            yaw_diff = normalize_angle(target_yaw - prev_yaw)

            # Smooth interpolation
            smoothed_yaw = prev_yaw + yaw_diff * self._orientation_smoothing_factor
            smoothed_yaw = normalize_angle(smoothed_yaw)
        else:
            smoothed_yaw = target_yaw

        # Store for next frame
        self._agent_previous_orientations[agent_id] = smoothed_yaw

        return smoothed_yaw
