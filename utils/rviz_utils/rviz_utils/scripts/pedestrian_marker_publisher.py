#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from arena_people_msgs.msg import Pedestrians, Pedestrian 
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import math


class PedestrianMarkerPublisher(Node):
    """
    Node that converts arena_people_msgs/Pedestrians to visualization_msgs/MarkerArray
    for better pedestrian visualization in RViz2
    """

    def __init__(self):
        super().__init__('pedestrian_marker_publisher')
        
        # Get namespace parameter for multi-environment support
        namespace = self.get_namespace()
        
        # QoS Settings for arena_peds topic
        pedestrians_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS Settings for marker output
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Build namespaced topic names
        pedestrians_topic = f'{namespace}/arena_peds' 
        marker_topic = f'{namespace}/pedestrian_markers'
        
        # Subscriber to arena_peds topic
        self.pedestrians_subscriber = self.create_subscription(
            Pedestrians,  
            pedestrians_topic,
            self.pedestrians_callback, 
            pedestrians_qos
        )
        
        # Publisher for pedestrian markers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            marker_topic,
            marker_qos
        )
        
        # Parameters for visualization
        self.declare_parameter('body_height', 1.6)  # Height of pedestrian body
        self.declare_parameter('body_radius', 0.25)  # Radius of pedestrian body
        self.declare_parameter('head_radius', 0.15)  # Radius of pedestrian head
        self.declare_parameter('arrow_length', 0.6)  # Length of direction arrow
        self.declare_parameter('show_labels', True)  # Show name labels
        self.declare_parameter('show_velocity_arrows', True)  # Show velocity arrows
        self.declare_parameter('show_orientation_arrows', True)  # Show orientation arrows
        
        self.get_logger().info('Pedestrian Marker Publisher initialized')
        self.get_logger().info(f'Subscribing to {pedestrians_topic}, publishing to {marker_topic}')

    def pedestrians_callback(self, msg: Pedestrians): 
        """Convert Pedestrians message to MarkerArray with stylized human figures"""
        
        marker_array = MarkerArray()
        markers = []
        
        # Get parameters
        body_height = self.get_parameter('body_height').value
        body_radius = self.get_parameter('body_radius').value
        head_radius = self.get_parameter('head_radius').value
        arrow_length = self.get_parameter('arrow_length').value
        show_labels = self.get_parameter('show_labels').value
        show_velocity_arrows = self.get_parameter('show_velocity_arrows').value
        show_orientation_arrows = self.get_parameter('show_orientation_arrows').value
        
        # Clear existing markers first (important for dynamic number of people)
        delete_marker = Marker()
        delete_marker.header = msg.header
        delete_marker.action = Marker.DELETEALL
        markers.append(delete_marker)
        
        for i, pedestrian in enumerate(msg.pedestrians): 
            # Use pedestrian ID directly
            pedestrian_id = pedestrian.id
            
            # Determine colors based on animation state
            body_color, head_color = self._get_pedestrian_colors(pedestrian)  
            
            # 1. Body (Cylinder)
            body_marker = self._create_body_marker(pedestrian, pedestrian_id, body_color, 
                                                 body_height, body_radius, msg.header)
            markers.append(body_marker)
            
            # 2. Head (Sphere)
            head_marker = self._create_head_marker(pedestrian, pedestrian_id, head_color,
                                                 head_radius, body_height, msg.header)
            markers.append(head_marker)
            
            # 3. Velocity Arrow (if pedestrian is moving)
            if show_velocity_arrows:
                velocity_magnitude = math.sqrt(pedestrian.twist.linear.x**2 + pedestrian.twist.linear.y**2) 
                if velocity_magnitude > 0.1:  # Only show if moving fast enough
                    arrow_marker = self._create_velocity_arrow(pedestrian, pedestrian_id, 
                                                             arrow_length, body_height, msg.header)
                    markers.append(arrow_marker)
            
            # 4. Orientation Arrow (shows where pedestrian is facing)
            if show_orientation_arrows:
                orientation_marker = self._create_orientation_arrow(pedestrian, pedestrian_id,
                                                                  arrow_length, body_height, msg.header)
                markers.append(orientation_marker)
            
            # 5. Name Label
            if show_labels:
                label_marker = self._create_name_label(pedestrian, pedestrian_id, 
                                                     body_height, msg.header)
                markers.append(label_marker)
        
        marker_array.markers = markers
        self.marker_publisher.publish(marker_array)

    def _get_pedestrian_colors(self, pedestrian: Pedestrian): 
        """Get colors based on pedestrian animation state"""
        
        # Map animation states to colors
        if pedestrian.animation_state == Pedestrian.WALKING:
            body_color = ColorRGBA(r=0.2, g=0.7, b=0.2, a=1.0)  # Green
            head_color = ColorRGBA(r=0.9, g=0.7, b=0.5, a=1.0)  # Skin tone
        elif pedestrian.animation_state == Pedestrian.IDLE:
            body_color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # Gray
            head_color = ColorRGBA(r=0.9, g=0.7, b=0.5, a=1.0)  # Skin tone
        elif pedestrian.animation_state == Pedestrian.PANIC:
            body_color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)  # Red
            head_color = ColorRGBA(r=1.0, g=0.6, b=0.6, a=1.0)  # Light red
        elif pedestrian.animation_state == Pedestrian.SURPRISED:
            body_color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0)  # Yellow
            head_color = ColorRGBA(r=0.9, g=0.7, b=0.5, a=1.0)  # Skin tone
        elif pedestrian.animation_state == Pedestrian.CURIOUS:
            body_color = ColorRGBA(r=0.2, g=0.2, b=1.0, a=1.0)  # Blue
            head_color = ColorRGBA(r=0.9, g=0.7, b=0.5, a=1.0)  # Skin tone
        elif pedestrian.animation_state == Pedestrian.THREATENING:
            body_color = ColorRGBA(r=0.8, g=0.0, b=0.8, a=1.0)  # Purple
            head_color = ColorRGBA(r=0.9, g=0.7, b=0.5, a=1.0)  # Skin tone
        else:
            # Default colors
            body_color = ColorRGBA(r=0.3, g=0.3, b=0.8, a=1.0)  # Blue
            head_color = ColorRGBA(r=0.9, g=0.7, b=0.5, a=1.0)  # Skin tone
            
        return body_color, head_color

    def _create_body_marker(self, pedestrian: Pedestrian, pedestrian_id: int, color: ColorRGBA, 
                           body_height: float, body_radius: float, header) -> Marker:
        """Create cylinder marker for pedestrian body"""
        marker = Marker()
        marker.header = header
        marker.ns = "pedestrian_bodies"
        marker.id = pedestrian_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position - use pedestrian position
        marker.pose.position.x = pedestrian.position.position.x 
        marker.pose.position.y = pedestrian.position.position.y  
        marker.pose.position.z = body_height / 2  # Center of cylinder
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale = Vector3(x=body_radius * 2, y=body_radius * 2, z=body_height)
        
        # Color
        marker.color = color
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker

    def _create_head_marker(self, pedestrian: Pedestrian, pedestrian_id: int, color: ColorRGBA,  
                           head_radius: float, body_height: float, header) -> Marker:
        """Create sphere marker for pedestrian head"""
        marker = Marker()
        marker.header = header
        marker.ns = "pedestrian_heads"
        marker.id = pedestrian_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position on top of body
        marker.pose.position.x = pedestrian.position.position.x  
        marker.pose.position.y = pedestrian.position.position.y  
        marker.pose.position.z = body_height + head_radius
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale = Vector3(x=head_radius * 2, y=head_radius * 2, z=head_radius * 2)
        
        # Color
        marker.color = color
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker

    def _create_orientation_arrow(self, pedestrian: Pedestrian, pedestrian_id: int,  
                                arrow_length: float, body_height: float, header) -> Marker:
        """Create arrow marker showing pedestrian orientation"""
        marker = Marker()
        marker.header = header
        marker.ns = "pedestrian_orientation"
        marker.id = pedestrian_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position at pedestrian location
        marker.pose.position.x = pedestrian.position.position.x  
        marker.pose.position.y = pedestrian.position.position.y  
        marker.pose.position.z = body_height / 2
        
        # Orientation from pedestrian orientation
        marker.pose.orientation = pedestrian.position.orientation  
        
        # Size
        marker.scale = Vector3(x=arrow_length, y=0.15, z=0.15)
        
        # Color (blue for orientation)
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)  # Semi-transparent blue
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker

    def _create_velocity_arrow(self, pedestrian: Pedestrian, pedestrian_id: int,  
                             arrow_length: float, body_height: float, header) -> Marker:
        """Create arrow marker showing pedestrian velocity direction"""
        marker = Marker()
        marker.header = header  
        marker.ns = "pedestrian_velocity"
        marker.id = pedestrian_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position at pedestrian location
        marker.pose.position.x = pedestrian.position.position.x  
        marker.pose.position.y = pedestrian.position.position.y  
        marker.pose.position.z = body_height / 2 + 0.2  
        
        # Orientation from velocity direction
        vel_x = pedestrian.twist.linear.x  
        vel_y = pedestrian.twist.linear.y  
        yaw = math.atan2(vel_y, vel_x)
        
        marker.pose.orientation.z = math.sin(yaw/2)
        marker.pose.orientation.w = math.cos(yaw/2)
        
        # Size (arrow length proportional to speed)
        velocity_magnitude = math.sqrt(vel_x**2 + vel_y**2)
        actual_length = min(arrow_length * velocity_magnitude, arrow_length)
        marker.scale = Vector3(x=actual_length, y=0.1, z=0.1)
        
        # Color (bright for visibility)
        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker

    def _create_name_label(self, pedestrian: Pedestrian, pedestrian_id: int,  
                          body_height: float, header) -> Marker:
        """Create text marker with pedestrian name"""
        marker = Marker()
        marker.header = header
        marker.ns = "pedestrian_labels"
        marker.id = pedestrian_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position above head
        marker.pose.position.x = pedestrian.position.position.x  
        marker.pose.position.y = pedestrian.position.position.y  
        marker.pose.position.z = 0.0 + body_height + 0.5  # Ground + body + label offset
        marker.pose.orientation.w = 1.0
        
        # Text content - use pedestrian name
        marker.text = f"Agent {pedestrian.name}"  
        
        # Size
        marker.scale.z = 0.3  # Text height
        
        # Color (white for visibility)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    
    node = PedestrianMarkerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down pedestrian marker publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()