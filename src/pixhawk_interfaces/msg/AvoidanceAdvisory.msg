# Advisory computed by LiDAR mini-PC, not a PX4 movement command.
# PX4 flight controller owns mixing, age checks and RC priority.
float32 vx_limit_mps
float32 yaw_correction_radps
float32 front_clearance_m
float32 left_clearance_m
float32 right_clearance_m
bool valid
bool obstacle
bool stop
# Incremented only for a newly validated unique LiDAR scan; zero means legacy/unknown source.
uint64 source_scan_seq
