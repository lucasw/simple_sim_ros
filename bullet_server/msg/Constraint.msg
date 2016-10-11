uint8 POINT2POINT=0
uint8 FIXED=1
uint8 HINGE=2
uint8 SLIDER=3
# http://bulletphysics.org/mediawiki-1.5.8/index.php/Constraints
uint8 type
string name
string body_a
string body_b
float32 lower_lim
float32 upper_lim
# if type is fixed pivot is just a point, not a pivot, rename later?
geometry_msgs/Point pivot_in_a
geometry_msgs/Point pivot_in_b
geometry_msgs/Vector3 axis_in_a
geometry_msgs/Vector3 axis_in_b
