uint8 SPHERE=0
# not supported yet
uint8 BOX=1
uint8 CYLINDER=2
uint8 CAPSULE=3
uint8 CONE=4
uint8 MULTI_SPHERE=5
# TODO mesh shapes, convex hull...
# http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Shapes
# has to be unique of all links
# if an existing link of this name exists it will be replaced
string name
int32 type
# starting pose and twist
# TODO should initial conditions be in a separate message
# than the body name and shape etc.?
geometry_msgs/Pose pose
geometry_msgs/Twist twist
# x defines the sphere radius, and the cylinder radius, rect length
# yz will be used for rect width and height
geometry_msgs/Vector3 scale
# not used yet
string text
string mesh_resource
bool mesh_use_embedded_materials
