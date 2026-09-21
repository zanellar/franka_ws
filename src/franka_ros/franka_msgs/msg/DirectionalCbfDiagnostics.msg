# One controller sample. Missing/unavailable numeric diagnostics are NaN.
std_msgs/Header header
uint64 sample_index
uint64 experiment_id
float64 dt
float64 cbf_h
# a*(tau_command-coriolis)+b+alpha*h, including bypass/abort commands.
float64 cbf_constraint_safe
# Residual of the QP candidate, including candidates rejected by the residual check.
float64 cbf_constraint_qp
float64 kinetic_energy_dir
# Six singular values of the full base-frame geometric 6x7 Jacobian, descending.
float64[6] svd_jacobian
float64 Kmax
float64 alpha
float64 cbf_residual_tolerance
float64[3] direction
bool cbf_active
bool task_aborted
bool cbf_solution_applied
uint8 solver_status

# Additional synchronous diagnostics; not used by the control law.
float64 kinetic_energy_total
# Measured EE and requested target positions, both in header.frame_id.
# Distances are sampled before this cycle's reference-filter update.
float64[3] ee_position
float64[3] target_position
float64 ee_target_distance
float64 ee_reference_distance
# NaN joint arrays when cbf_active is false. CSV debug columns are then empty.
float64[7] q
float64[7] dq
# Controller setCommand torque, before any Gazebo mode override or constraint force.
float64[7] tau_command
# Raw libfranka RobotMode; meaningful in the debug CSV only when cbf_active is true.
uint8 robot_mode
