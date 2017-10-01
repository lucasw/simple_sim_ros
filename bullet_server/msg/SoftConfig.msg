float32 kVCF # 1  Velocities correction factor (Baumgarte)
float32 kDP # 0  Damping coefficient [0,1]
float32 kDG # 0  Drag coefficient [0,+inf]
float32 kLF # 0  Lift coefficient [0,+inf]
float32 kPR # 0  Pressure coefficient [-inf,+inf], don't set this non-zero without faces
float32 kVC # 0  Volume conservation coefficient [0,+inf]
float32 kDF # 0.2  Dynamic friction coefficient [0,1]
float32 kMT # 0  Pose matching coefficient [0,1]
float32 kCHR # 1.0  Rigid contacts hardness [0,1]
float32 kKHR # 0.1  Kinetic contacts hardness [0,1]
float32 kSHR # 1.0  Soft contacts hardness [0,1]
float32 kAHR # 0.7  Anchors hardness [0,1]
float32 kSRHR_CL # 0.1  Soft vs rigid hardness [0,1] (cluster only)
float32 kSKHR_CL # 1.0  Soft vs kinetic hardness [0,1] (cluster only)
float32 kSSHR_CL # 0.5  Soft vs soft hardness [0,1] (cluster only)
float32 kSR_SPLT_CL # 0.5  Soft vs rigid impulse split [0,1] (cluster only)
float32 kSK_SPLT_CL # 0.5  Soft vs rigid impulse split [0,1] (cluster only)
float32 kSS_SPLT_CL # 0.5  Soft vs rigid impulse split [0,1] (cluster only)
float32 maxvolume # 1.0  Maximum volume ratio for pose
float32 timescale # 1.0  Time scale
