import rclpy
from rclpy.node import Node

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose, ControlWorld
from ros_gz_interfaces.msg import EntityFactory, WorldControl
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
import sys

from task_generator.simulators import SimulatorFactory
from task_generator.shared import rosparam_get
from tf_transformations import quaternion_from_euler
from task_generator.constants import Config, Constants
from task_generator.simulators import BaseSimulator

from task_generator.shared import ModelType, Namespace, PositionOrientation, RobotProps

class GazeboSimulator(BaseSimulator, Node):

    def __init__(self, namespace):
        # Ensure we have a valid node name
        node_name = namespace.strip('/').replace('/', '_')
        if not node_name:
            node_name = "gazebo_simulator"  # Default name if namespace is empty
        
        Node.__init__(self, node_name)
        super().__init__(namespace)
        
        self._goal_pub = self.create_publisher(
            PoseStamped,
            self._namespace("/goal"),
            10
        )
        self.declare_parameter('robot_model', '')
        self._robot_name = self.get_parameter('robot_model').value

        # self._spawn_model = {
        #     ModelType.URDF: self.create_client(SpawnModel, '/gazebo/spawn_urdf_model'),
        #     ModelType.SDF: self.create_client(SpawnModel, '/gazebo/spawn_sdf_model'),
        # }
        
        # Note: There's no direct equivalent for set_model_state in the new Gazebo
        # self._move_model_srv = self.create_client(SetModelState, '/gazebo/set_model_state')
        
        # Pause and unpause are not directly available in new Gazebo
        # self._unpause = self.create_client(Empty, '/gazebo/unpause_physics')
        # self._pause = self.create_client(Empty, '/gazebo/pause_physics')
        # self._remove_model_srv = self.create_client(DeleteModel, '/gazebo/delete_model')
        
        # Initialize service clients
        # https://gazebosim.org/api/sim/8/entity_creation.html
        self._spawn_entity = self.create_client(SpawnEntity, '/world/default/create')
        self._delete_entity = self.create_client(DeleteEntity, '/world/default/remove')
        self._set_entity_pose = self.create_client(SetEntityPose, '/world/default/set_pose')
        self._control_world = self.create_client(ControlWorld, '/world/default/control')
        
        self.get_logger().info("Waiting for gazebo services...")
        # Wait for services to be available
        self._spawn_entity.wait_for_service()
        self._delete_entity.wait_for_service()
        self._set_entity_pose.wait_for_service()
        self._control_world.wait_for_service()

        self.get_logger().info("Gazebo services are available now.")
        

    def before_reset_task(self):
        self.pause_simulation()

    def after_reset_task(self):
        try:
            self.unpause_simulation()
        except rospy.service.ServiceException as e:  # gazebo isn't the most reliable
            rospy.logwarn(e)

    # ROBOT

    def move_entity(self, name, position):
        request = SetEntityPose.Request()
        request.entity = name
        request.pose = Pose(
            position=Point(x=position.x, y=position.y, z=0),
            orientation=Quaternion(*quaternion_from_euler(0.0, 0.0, position.orientation, axes="sxyz"))
        )
        future = self._set_entity_pose.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def spawn_entity(self, entity):
        request = SpawnEntity.Request()
        request.entity_factory = EntityFactory()
        request.entity_factory.name = entity.name
        request.entity_factory.type = 'sdf'  # or 'urdf' depending on your model type
        request.entity_factory.sdf = entity.model.get(self.MODEL_TYPES).description
        request.entity_factory.pose = Pose(
            position=Point(x=entity.position.x, y=entity.position.y, z=0),
            orientation=Quaternion(*quaternion_from_euler(0.0, 0.0, entity.position.orientation, axes="sxyz"))
        )

        if isinstance(entity, RobotProps):
            self.declare_parameter(Namespace(entity.name)("robot_description"), entity.model.get(self.MODEL_TYPES).description)
            self.declare_parameter(Namespace(entity.name)("tf_prefix"), str(Namespace(entity.name)))

        future = self._spawn_entity.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success


    def delete_entity(self, name):
        request = DeleteEntity.Request()
        request.entity = name
        future = self._delete_entity.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
    
    def pause_simulation(self):
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.pause = True
        future = self._control_world.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def unpause_simulation(self):
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.pause = False
        future = self._control_world.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def step_simulation(self, steps):
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.multi_step = steps
        future = self._control_world.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def _publish_goal(self, goal: PositionOrientation):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, goal.orientation, axes="sxyz"))

        self._goal_pub.publish(goal_msg)
