# Directional CBF: Gazebo experiment fixes

Base: zanellar/franka_ws, branch franka_ws_013_gazebo,
commit 91ad391a9900c8fd673e3564fc4029257a87ab87.

## Apply and build

From the franka_ws root, with the patch downloaded there:

```bash
git apply --check directional_cbf_gazebo_fixes.patch
git apply directional_cbf_gazebo_fixes.patch
catkin_make
source devel/setup.bash
```

If disable_gazebo_joint_damping.patch was already applied, use these two
commands instead of the first two above. The complete patch still contains
that same seven-joint damping change.

```bash
git apply --check --exclude=src/franka_ros/franka_description/robots/common/franka_arm.xacro directional_cbf_gazebo_fixes.patch
git apply --exclude=src/franka_ros/franka_description/robots/common/franka_arm.xacro directional_cbf_gazebo_fixes.patch
```

Rebuild all affected packages: the patch adds a service to franka_msgs and
updates the controller and trajectory node. Stop the old Gazebo process and
start a fresh world so the model and hardware plugin are reloaded.

## Run

```bash
roslaunch franka_example_controllers cartesian_impedance_directional_kinetic_energy_cbf_controller_gazebo.launch start_trajectory:=true trajectory:=linear cbf_active:=true Kmax:=0.1 alpha:=5.0
```

The existing public command interface is unchanged:

```bash
rosservice call /trajectory_publisher/set_experiment_command "{x_move: 0.1, cbf_active: true, Kmax: 0.1, alpha: 5.0}"
```

After an abort, try new parameters for the SAME previous target using x_move=0:

```bash
rosservice call /trajectory_publisher/set_experiment_command "{x_move: 0.0, cbf_active: true, Kmax: 0.2, alpha: 10.0}"
```

x_move is still relative to the previous commanded target, as in the original
linear node. A nonzero x_move changes that target again. A successful service
response means the request was validated and queued, not that the subsequent
QP or trajectory will succeed. Inspect /cbf_info during execution.

## Behavior

1. Launch values for Kmax, alpha and cbf_active seed dynamic_reconfigure before
   its initial callback. Runtime updates remain supported. Values outside the
   existing generated configuration ranges are rejected at startup/experiment
   submission rather than silently accepted with a different value.
2. Only OSQP optimal is accepted. Inaccurate, iteration-limit, infeasible,
   invalid-solution and QP update/setup failures latch an abort. No wall-clock
   timeout is assigned; the OSQP default is unlimited. The finite max_iter=8000
   still applies. The node and its ROS services remain alive.
3. The residual a*u+b+alpha*h must be finite and at least -1e-5 by default.
   Failure latches status 6 and the rejected solution is never sent to the joints.
   OSQP eps_abs and eps_rel are explicitly 1e-7. A numerical residual tolerance
   is still necessary; this is not a claim of exact continuous-time invariance.
4. The directional QP has only the CBF row. Both torque-magnitude and torque-rate
   limits are removed, including the bypass path. The directional Gazebo launch
   sets /disable_gazebo_torque_limits=true. FrankaHWSim sets Gazebo effort and
   velocity limits to -1 for the seven arm joints, disabling both effort clipping
   and the velocity-triggered clipping inside CheckAndTruncateForce. Its cached
   effort limits are also disabled. Finger joints are untouched. Joint position
   stops and contacts are still part of the simulated physics. Other launches
   retain their default behavior if this global parameter is absent/false.
5. The seven joint damping values in common/franka_arm.xacro become zero, exactly
   as in the previously supplied patch. The change affects models using this
   shared macro. The impedance controller's commanded damping is retained.

## What an abort does

The controller stops following the experiment target, latches the original
failure code, and actively brakes. With a valid positive-definite inertia it
commands tau=-k*M(q)*dq, with k=20 /s by default. Gravity compensation is still
provided by FrankaHWSim; Coriolis feed-forward is omitted while braking. This
makes the viscous brake dissipative for total kinetic energy under the modeled
continuous-time dynamics. If the inertia is invalid, a model-independent
brake tau=-dq is used; if velocity is not finite, zero commanded torque is used.

This is an interruption of the task with physical deceleration, not an
instantaneous stop, a Gazebo pause, or a certified directional-energy fallback.
It does not guarantee K_dir <= Kmax during the abort. That distinction is
visible in solver_status. Large simulation steps/model errors can also affect
braking behavior and must be checked in Gazebo.

Periodic pose messages and ordinary dynamic_reconfigure edits do not clear the
abort latch. A new valid set_experiment_command does, even if all values are
identical. It resets the QP and derivative history and starts interpolation from
the measured pose. If the new attempt fails, it aborts again.

The public linear service forwards parameters and target together to the new
internal controller service:

/cartesian_impedance_directional_kinetic_energy_cbf_controller/start_experiment

This prevents a retry from briefly resuming an old target. After the first such
command, that controller accepts targets through this service; queued/periodic
pose topic messages cannot overwrite them. Before the first command, the
existing pose topic path remains available. The legacy total-energy linear
service path is retained when directional_experiment_service is not configured.
If starting linear manually, supply its private parameter:

_directional_experiment_service:=/cartesian_impedance_directional_kinetic_energy_cbf_controller/start_experiment

## Diagnostics

```bash
rostopic echo /cbf_info
rosrun dynamic_reconfigure dynparam get /cartesian_impedance_directional_kinetic_energy_cbf_controller/dynamic_reconfigure_compliance_param_node
```

| solver_status | Meaning |
| --- | --- |
| 0 | Intentional CBF bypass, task not aborted |
| 1 | Optimal solution with accepted residual |
| 3 | Model/state/control invalid or directional mobility too small; aborted |
| 4 | QP setup/update failure; aborted |
| 5 | Nonoptimal solver status or invalid solution; aborted |
| 6 | Nonfinite or negative residual beyond tolerance; aborted |

An abort retains its failure code even if cbf_active is subsequently changed.
u_cbf and u_saturated report the command actually sent by this controller,
including braking while aborted. h and directional_kinetic_energy continue to
be evaluated; unavailable model diagnostics are NaN rather than a false zero.

Optional private controller parameters, read at startup:

- abort_damping: positive inertia-weighted braking gain, default 20 /s.
- cbf_residual_tolerance: nonnegative acceptance tolerance, default 1e-5.

## Validation

The patch includes a standalone C++ test of the actual latch and residual
acceptance code. It checks all abort categories, persistence over update cycles,
explicit retry, failure during retry, repeated retry, tolerance boundaries,
nonoptimal status, NaN and infinity. Run without ROS:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -I src/franka_ros/franka_example_controllers/include src/franka_ros/franka_example_controllers/test/directional_cbf_task_state_test.cpp -o /tmp/directional_cbf_task_state_test
/tmp/directional_cbf_task_state_test
```

The test is also registered with CTest when CATKIN_ENABLE_TESTING is enabled.
These tests, XML parsing and patch application checks were performed when
preparing this patch. Full catkin compilation and Gazebo execution were not
available in that environment and remain to be verified in your ROS workspace.
