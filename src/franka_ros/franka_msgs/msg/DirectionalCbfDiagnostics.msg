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
