string name
# point masses within the soft body
bullet_server/Node[] node
bullet_server/Link[] link
bullet_server/Face[] face
bullet_server/Tetra[] tetra
# if any of the above have non-zero material_in the material below
# needs to be defined - maybe instead of inds use names?
bullet_server/Material[] material
# these can attach the softbody to an already
# existing rigid body - TODO(lucasw) may need an addAnchor service
# to handle cases where the rigid body doesn't exist yet.
bullet_server/Anchor[] anchor
# bullet_server/Anchor[] anchor
# not sure about these
bool randomize_constraints
# uint8 bending_contraints_distance
# uint8 piterations
# how much space around the body- everything floats this far
# away from rigid bodies, but don't set it to zero or else instability
float32 margin
uint8 k_clusters
bullet_server/SoftConfig config
geometry_msgs/Vector3 scale
geometry_msgs/Pose pose
