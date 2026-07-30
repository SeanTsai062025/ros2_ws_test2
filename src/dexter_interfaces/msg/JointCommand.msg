# Correlation metadata. Legacy publishers may leave these fields empty.
string show_run_id
string node_id
string command_id
uint32 attempt

# Joint targets in this order: base, part1, part2, part3, part4, part5.
float64[6] positions

# Fraction of the configured maximum joint velocity and acceleration.
# Valid command values are 0.2 through 1.0.
float64 speed_scaling 0.5
