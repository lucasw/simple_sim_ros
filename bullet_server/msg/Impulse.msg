string body
# http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=2693
# this is currently in the world frame
geometry_msgs/Vector3 impulse
# this is in the body frame (or in world frame, but relative to body center?)
geometry_msgs/Point location
# TODO(lucasw) later provide modes here to switch
# which frame for each of the above

# if a soft body can apply a force to a specific node
bool soft_body
bool node_only
uint32 node_ind
