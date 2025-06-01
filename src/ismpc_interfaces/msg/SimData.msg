# Header with timestamp
std_msgs/Header header

# Time
float64 time

# Joint states
float64[] qpos
float64[] qvel  
float64[] qacc

# LIP State
geometry_msgs/Vector3 com_pos
geometry_msgs/Vector3 com_vel
geometry_msgs/Vector3 com_acc
geometry_msgs/Vector3 zmp_pos