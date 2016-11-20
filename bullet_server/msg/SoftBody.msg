string name
# point masses within the soft body
bullet_server/Node[] node
bullet_server/Link[] link
bullet_server/Face[] face
bullet_server/Tetra[] tetra
bullet_server/Material[] material
# these can attach the softbody to an already
# existing rigid body - TODO(lucasw) may need an addAnchor service
# to handle cases where the rigid body doesn't exist yet.
bullet_server/Anchor[] anchor
# bullet_server/Anchor[] anchor
# not sure about these
# bool randomize_constraints
# uint8 bending_contraints_distance
# uint8 piterations
bullet_server/SoftConfig config
geometry_msgs/Vector3 scale
geometry_msgs/Pose pose
