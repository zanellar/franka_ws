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

# Frozen FIRST failed real-hardware projection (solver_status=5).
# Reset by controller starting() or the next consumed start_experiment request.
# Inspect only when valid=true; other diagnostics still describe CURRENT state.
# reason: 1 nonfinite input, 2 invalid box, 3 zero normal infeasible,
# 4 box/CBF infeasible, 5 numerical failure, 6 invalid envelope.
# Raw decision x is torque including Coriolis, excluding gravity.
# Constraint: coefficients*x >= rhs. max_residual=max(coefficients*x-rhs)
# over lower<=x<=upper; same units as h_dot+alpha*h (J/s).
bool qp_failure_valid
time qp_failure_stamp
uint64 qp_failure_sample_index
uint64 qp_failure_experiment_id
uint8 qp_failure_reason
uint8 qp_failure_energy_mode
float64 qp_failure_h
float64 qp_failure_alpha
float64 qp_failure_Kmax
float64 qp_failure_barrier_b
float64 qp_failure_rhs
float64 qp_failure_max_lhs
float64 qp_failure_max_residual
float64 qp_failure_filter_gain
float64 qp_failure_torque_rate
float64 qp_failure_dt
float64 qp_failure_kinetic_energy_total
float64 qp_failure_kinetic_energy_dir
float64 qp_failure_command_success_rate
float64[7] qp_failure_q
float64[7] qp_failure_dq
float64[7] qp_failure_coriolis
float64[7] qp_failure_barrier_a
float64[7] qp_failure_nominal_tau
float64[7] qp_failure_previous_tau
float64[7] qp_failure_lower
float64[7] qp_failure_upper
float64[7] qp_failure_soft_lower
float64[7] qp_failure_soft_upper
float64[7] qp_failure_coefficients

# Synchronous model for offline replay (not the frozen failure snapshot).
bool model_valid
# Eigen column-major order: M(row,col)=model_mass[row+7*col].
float64[49] model_mass
float64[7] model_coriolis
float64[7] model_gravity
# Diagnostic legacy slew limiter, seeded with measured dq at controller starting.
# Updated every callback, including CBF off; no reset on experiment requests.
float64[7] model_dq_rate_limited
# Raw torque bounds including Coriolis, excluding gravity (real runtime only).
bool replay_qp_envelope_valid
float64[7] replay_qp_lower
float64[7] replay_qp_upper
float64 replay_qp_filter_gain
float64[7] replay_soft_lower
float64[7] replay_soft_upper

# QP cost: sum((tau_i-tau_nom_i)^2/replay_qp_inverse_weights_i).
# 0 identity (original), 1 normalized diagonal mass (real total-energy only).
uint8 total_energy_qp_metric
float64[7] replay_qp_inverse_weights
