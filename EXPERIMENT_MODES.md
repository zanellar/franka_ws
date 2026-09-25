# Total, directional, and unfiltered kinetic energy trials (FR3)

This change extends the existing Cartesian impedance controller. The launch
argument `energy_mode` is fixed for the lifetime of a launch:

| mode | barrier used when `cbf_active=true` | experiment |
| --- | --- | --- |
| `total` | `Kmax - kinetic_energy_total` | A |
| `directional` | `Kmax - kinetic_energy_dir` | B |
| `none` | no QP filter (`cbf_active=false` required) | C |

Restart the launch to switch modes. `set_experiment_command` can change
`cbf_active`, `Kmax`, and `alpha` within a mode, but cannot change the mode.
Use one initial configuration, one commanded displacement and the same
Cartesian gains for all seven trials (three A, three B, one C). Initialize
the robot afresh before *each* trial. Record the exact requested joint pose
as shown below; the CSV separately stores measured `experiment_initial_q`.

Apply the accompanying patch from the root of the `franka_ws_013_gazebo2real`
checkout, rebuild and restart every node that uses the modified ROS messages:

```bash
git apply --check energy_modes.patch
git apply energy_modes.patch
source /opt/ros/noetic/setup.bash
catkin_make install
source install/setup.bash
```

For each mode, start a fresh launch in its own terminal. Example for A:

```bash
source ~/Riccardo/franka_ws_013/install/setup.bash
RUN_DIR="$HOME/Riccardo/franka_ws_013/data/total_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" > "$HOME/Riccardo/franka_ws_013/data/active_run_dir.txt"
roslaunch franka_example_controllers \
  cartesian_impedance_directional_kinetic_energy_cbf_controller.launch \
  robot_ip:=172.16.0.2 robot:=fr3 load_gripper:=false \
  energy_mode:=total cbf_active:=false \
  record_cbf:=true log_dir:="$RUN_DIR" \
  rosbag:=true bag_prefix:="$RUN_DIR/run" \
  rviz:=false
```

For B, use `energy_mode:=directional` and a `directional_...` run directory.
For C, use `energy_mode:=none`, `cbf_active:=false` and a `none_...` run
directory. The other launch arguments, including the direction vector and
`derivative_filter_alpha`, should be the same for A and B; write their
values into the run metadata. The directional energy is a measured diagnostic
also during A and C. An invalid directional mobility makes this diagnostic
NaN during A or C without preventing the total energy calculation.

In a second terminal, save configuration and execute each trial. Edit the
requested pose, duration, gains, displacement, Kmax and three alpha values
for your experiment. The example pose and displacement below are illustrative.
The first initialization may reject a duration that is too short; use the
minimum reported by preflight with some margin.

```bash
source ~/Riccardo/franka_ws_013/install/setup.bash
RUN_DIR="$(cat "$HOME/Riccardo/franka_ws_013/data/active_run_dir.txt")"
rosparam dump "$RUN_DIR/controller_parameters.yaml" \
  /cartesian_impedance_directional_kinetic_energy_cbf_controller
cat > "$RUN_DIR/protocol.yaml" <<'YAML'
requested_initial_q: [0.0, -0.7, 0.0, -2.5, 0.0, 1.8, 0.785398163397]
requested_duration_s: 17.0
commanded_displacement_m: [0.25, 0.2, 0.0]
Kmax_J: 0.01
alphas: [1.0, 10.0, 100.0]
nominal_translational_stiffness_N_per_m: 200.0
nominal_rotational_stiffness_Nm_per_rad: 10.0
nominal_nullspace_stiffness: 0.5
YAML

# Repeat initialization immediately before each of the three A/B trials.
rosservice call /trajectory_publisher/initialize_joint_pose \
  "q: [0.0, -0.7, 0.0, -2.5, 0.0, 1.8, 0.785398163397]
duration: 17.0"

# Repeat with 1.0, 10.0, 100.0, initializing again each time.
rosservice call /trajectory_publisher/set_experiment_command \
  "x_move: 0.25
y_move: 0.2
z_move: 0.0
cbf_active: true
Kmax: 0.01
alpha: 1.0"
```

For C, initialize once and issue the same `set_experiment_command` with
`cbf_active: false`; retain a finite positive `alpha` and the same `Kmax` to
make the diagnostic comparison straightforward. The mode is `none` because
of the launch selection, and the service rejects `cbf_active: true`.

After each service call, inspect `/directional_cbf/diagnostics` for the
expected `experiment_id`, `energy_mode` (0 directional, 1 total, 2 none),
`alpha`, `cbf_active`, `task_aborted` and `cbf_solution_applied`. An aborted
sample uses braking torque: its `u_safe` is only a candidate, and should
not be plotted as the applied command. `tau_command` is the actual
`setCommand` value before libfranka processing.

The triggered recorder saves `cbf.csv`, `robot_state.csv`, `metadata.json`
and `events.csv` in a session subdirectory of `RUN_DIR`. The rosbag is saved
directly in `RUN_DIR`. The CSV contains both kinetic energies, EE distance,
EE XYZ, q/dq, nominal/safe control, mode, alpha, Kmax, relative move,
measured start q and actual stiffness for each sample. The requested
initialization pose and intended protocol are in `protocol.yaml`. Preserve
the service responses and record any changes to the protocol in that file.

Model for both CBFs, in the existing bias-compensated coordinates
`M(q) qdd = u`:

```
Ktot = 0.5 * dq' * M * dq
htot = Kmax - Ktot
h_dot_total = -dq' * u - 0.5 * dq' * M_dot * dq

Jd = direction' * Jlinear
Kdir = 0.5 * (Jd*dq)^2 / (Jd*M^-1*Jd')
hdir = Kmax - Kdir

QP constraint: a*u + b + alpha*h >= 0
```

The existing estimated `M_dot` is used by total mode, including any
`derivative_filter_alpha` selected at launch. The instantaneous total energy
uses the unfiltered mass and measured joint velocity. For valid data,
`cbf_h + kinetic_energy_total = Kmax` in total mode, and
`cbf_h + kinetic_energy_dir = Kmax` in directional mode.
