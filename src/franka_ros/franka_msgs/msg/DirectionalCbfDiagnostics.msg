# One controller sample. Missing/unavailable numeric diagnostics are NaN.
std_msgs/Header header
uint64 sample_index
uint64 experiment_id
float64 dt
float64 cbf_h
# a*(effective_torque-coriolis)+b+alpha*h, including bypass/abort commands.
# Effective torque: tau_command in Gazebo, predicted actuator output on hardware.
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
# Gazebo: NaN joint arrays when cbf_active is false. Real: always populated.
float64[7] q
float64[7] dq
# Controller setCommand torque, before hardware/simulator processing.
float64[7] tau_command
# Raw libfranka RobotMode; see debug_valid.
uint8 robot_mode

# Hardware extension. 0=legacy Gazebo, 1=real. Existing CSV core columns stay stable.
uint8 runtime_environment
bool debug_valid
float64 qp_time_us
# Elapsed update work through diagnostic preparation; not a WCET guarantee.
float64 update_time_us
float64 torque_prediction_error
# Desired gravity-free torque returned by libfranka in the current state.
float64[7] tau_J_d
# Measured joint torque (includes gravity); not directly comparable to tau_J_d.
float64[7] tau_J
# Predicted NEXT desired gravity-free torque after soft limits/filter/rate limit.
float64[7] tau_predicted
float64 control_command_success_rate
# solver_status: 7=hardware envelope/period invalid, 8=compute budget exceeded.

# Common A/B experiment telemetry. Mode identifies the energy constrained by h.
uint8 TOTAL_ENERGY=0
uint8 DIRECTIONAL_ENERGY=1
uint8 energy_mode
# Nominal alternative with the SAME final limits and actuator prediction as safe.
float64 cbf_constraint_nom
# Both controls exclude Coriolis. u_safe is NaN during abort.
float64[7] u_nom
float64[7] u_safe
