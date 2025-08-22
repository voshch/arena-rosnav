# Standard pedestrian representation for Arena simulators

# Identification
string name
uint32 id

# Spatial information
geometry_msgs/Pose position
geometry_msgs/Twist twist

# Animation/behavior states (integrated)
uint8 IDLE = 0
uint8 WALKING = 1  
uint8 RUNNING = 2
uint8 PANIC = 3
uint8 SURPRISED = 4
uint8 CURIOUS = 5
uint8 THREATENING = 6
uint8 animation_state

