string name
# point masses within the soft body
bullet_server/Node[] node
bullet_server/Link[] link
bullet_server/Face[] face
bullet_server/Tetra[] tetra
# bullet_server/Anchor[] anchor
# not sure about these
# bool randomize_constraints
# uint8 bending_contraints_distance
# uint8 piterations
float32                                kVCF                   # Velocities correction factor (Baumgarte)
float32                                kDP                    # Damping coefficient [0,1]
float32                                kDG                    # Drag coefficient [0,+inf]
float32                                kLF                    # Lift coefficient [0,+inf]
float32                                kPR                    # Pressure coefficient [-inf,+inf]
float32                                kVC                    # Volume conversation coefficient [0,+inf]
float32                                kDF                    # Dynamic friction coefficient [0,1]
float32                                kMT                    # Pose matching coefficient [0,1]
float32                                kCHR                   # Rigid contacts hardness [0,1]
float32                                kKHR                   # Kinetic contacts hardness [0,1]
float32                                kSHR                   # Soft contacts hardness [0,1]
float32                                kAHR                   # Anchors hardness [0,1]
float32                                kSRHR_CL               # Soft vs rigid hardness [0,1] (cluster only)
float32                                kSKHR_CL               # Soft vs kinetic hardness [0,1] (cluster only)
float32                                kSSHR_CL               # Soft vs soft hardness [0,1] (cluster only)
float32                                kSR_SPLT_CL    # Soft vs rigid impulse split [0,1] (cluster only)
float32                                kSK_SPLT_CL    # Soft vs rigid impulse split [0,1] (cluster only)
float32                                kSS_SPLT_CL    # Soft vs rigid impulse split [0,1] (cluster only)
float32                                maxvolume              # Maximum volume ratio for pose
float32                                timescale              # Time scale
geometry_msgs/Vector3 scale
geometry_msgs/Pose pose
