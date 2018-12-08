float32 kvcf 1  # kVCF  Velocities correction factor (Baumgarte)
float32 kdp 0  # kDP Damping coefficient [0,1]
float32 kdg 0  # kDG Drag coefficient [0,+inf]
float32 klf 0  # kLF Lift coefficient [0,+inf]
float32 kpr 0  # kPR Pressure coefficient [-inf,+inf], don't set this non-zero without faces
float32 kvc 0  # kVC Volume conservation coefficient [0,+inf]
float32 kdf 0.2  # kDF  Dynamic friction coefficient [0,1]
float32 kmt 0  # kMT  Pose matching coefficient [0,1]
float32 kchr 1.0  # kCHR Rigid contacts hardness [0,1]
float32 kkhr 0.1  # kKHR Kinetic contacts hardness [0,1]
float32 kshr 1.0  # kSHR Soft contacts hardness [0,1]
float32 kahr 0.7  # kAHR Anchors hardness [0,1]
float32 ksrhr_cl 0.1  # kSRHR_CL Soft vs rigid hardness [0,1] (cluster only)
float32 kskhr_cl 1.0  # kSKHR_CL Soft vs kinetic hardness [0,1] (cluster only)
float32 ksshr_cl 0.5  # kSSHR_CL Soft vs soft hardness [0,1] (cluster only)
float32 ksr_splt_cl 0.5  # kSR_SPLT_CL Soft vs rigid impulse split [0,1] (cluster only)
float32 ksk_splt_cl 0.5  # kSK_SPLT_CL Soft vs rigid impulse split [0,1] (cluster only)
float32 kss_splt_cl 0.5  # kSS_SPLT_CL Soft vs rigid impulse split [0,1] (cluster only)
float32 maxvolume 1.0  # Maximum volume ratio for pose
float32 timescale 1.0  # Time scale
