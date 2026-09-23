# Directional kinetic-energy experiments on the real robot

Patch base: `franka_ws_013_gazebo2real`, commit
`5bbd63539986486fd2cc053ae556627dd694ed8f`.

This adds an explicit `real` backend for the existing directional controller,
`linear` experiment service and joint initializer. It reuses `franka_control`,
`FrankaHW` and `position_controllers/JointTrajectoryController`. It does not
implement a robot driver or replace the Franka protections.

## Build and verify

Apply the patch from the workspace's Git root with the robot controller stopped:

```bash
git status --short
git apply --check /path/to/directional_cbf_gazebo2real.patch
git apply /path/to/directional_cbf_gazebo2real.patch
sudo apt install ros-noetic-joint-trajectory-controller ros-noetic-rostest
```

Rebuild the **whole workspace**, using the compiler, libfranka prefix and build
options in the main README's installation section. `DirectionalCbfDiagnostics`
has new fields: rebuilding only the controller leaves incompatible ROS message
MD5s in other nodes. Source the rebuilt environment in every terminal and restart
all nodes. Do not run nodes from a second, older overlay.

Use libfranka **0.13.3**, the version already required by this fork. Real mode
refuses a different compile-time Franka version because the torque predictor
was derived from that version's filter and rate-limiter implementation. Verify
`ldd install/lib/franka_control/franka_control_node` and the controller library
both resolve the intended installation. The runtime shared library must match.

Offline tests, from the workspace root:

```bash
python3 -m unittest discover \
  -s src/franka_ros/franka_example_controllers/test -p 'test_*.py'
g++ -std=c++14 -O2 -I src/franka_ros/franka_example_controllers/include \
  src/franka_ros/franka_example_controllers/test/directional_hardware_qp_test.cpp \
  -o /tmp/directional_hardware_qp_test
/tmp/directional_hardware_qp_test
g++ -std=c++14 -O2 -I src/franka_ros/franka_trajectory/include \
  src/franka_ros/franka_trajectory/test/initialization_policy_test.cpp \
  -o /tmp/initialization_policy_test
/tmp/initialization_policy_test
```

For the included ROS integration test, rebuild with `-DCATKIN_ENABLE_TESTING=ON`
using the same README build options, source the result, then run on an isolated
ROS master (with the robot launch stopped):

```bash
rostest franka_trajectory real_initialization.test
```

It uses fake controller-manager, joint-action, experiment-service and Franka-state
endpoints. It checks the position-controller handoff, invalid target rejection,
zero endpoint velocities/accelerations, measured reference synchronization,
action failure, blocked Cartesian commands after failure and explicit recovery.
It never connects to a robot and does not validate physical trajectory tracking.

Patch preparation verified the standalone C++ tests, Python recorder/plot tests,
XML syntax and application to the pinned sources. ROS compilation, the rostest,
Gazebo execution and robot execution were **not available in that environment**.

## Before the first physical experiment

Record the robot model, system/FCI version, libfranka version, real-time kernel,
controller build commit, tool mass/centre of mass/inertia, and EE/stiffness frames.
Verify the configured load and frames on the robot. `load_gripper:=false` only
controls ROS launch/model loading; it does not prove no tool is physically mounted
or reset the robot's configured load. The first hardware state is saved in
`robot_model_state.json` for later comparison.

The dedicated `franka_control/config/directional_cbf_real.yaml` enables real-time
mode, the existing rate limiter and 100 Hz filter. Its collision thresholds restore
the lower example values already documented in the fork, instead of inheriting
its active 750 Nm upper Cartesian moment thresholds. These are starting settings
to review for your robot/tool/task, not experimentally validated thresholds.
The general driver's default config is unchanged. Do not change the filter,
rate-limiter or robot model during a recording: the predictor snapshots them at
controller initialization. Restart after changes.

Use a clear workspace, a reachable non-singular initial pose and an accessible
stop. Validate initial position hold and a short displacement before repeating
the full Gazebo displacement. The software has no collision-free path planner.
An endpoint inside joint limits does not establish clearance along its path.

## Start and initialize

Start with the robot in FCI mode, using the same ROS installation as your existing
hardware impedance test:

```bash
roslaunch franka_example_controllers \
  cartesian_impedance_directional_kinetic_energy_cbf_controller.launch \
  robot_ip:=172.16.0.2 robot:=fr3 load_gripper:=false \
  cbf_active:=false Kmax:=0.02 alpha:=1.0 \
  direction_x:=1.0 direction_y:=0.0 direction_z:=0.0 \
  record_cbf:=true rosbag:=false rviz:=false
```

This launch uses the **directional** controller. Your previous
`cartesian_impedance_cbf_controller.launch` tests the total-energy controller.
The new real launch starts `linear` internally, with no autonomous trajectory.
It holds the measured starting pose and ignores external equilibrium-pose topic
commands. No hardcoded Gazebo pose is sent to the real arm. Commands remain
blocked until explicit joint initialization succeeds. There is no `trajectory`
argument in this launch; the old simulation launch keeps it.

In another sourced terminal:

```bash
rosservice call /trajectory_publisher/initialize_joint_pose \
"q: [0.0, -0.2, 0.0, -2.2, 0.0, 3.2, 0.785398163397]
duration: 12.0"
```

The first initialization starts recording before requesting motion. It checks
fresh state, MOVE/IDLE mode, rest, URDF hard/soft position limits with a 0.02 rad
margin and the required duration. The real launch caps the nominal quintic at
0.2 rad/s (also 10% of URDF velocity), 0.5 rad/s² and 1 rad/s³. A distant starting
pose may need **more than 12 s**; the service rejects a short duration and reports
the minimum instead of silently changing the motion. These are profile limits;
measured motion still depends on the hardware controller.

The initializer switches strictly from Cartesian effort to the existing position
joint-trajectory controller, sends a rest-to-rest quintic, waits for action success
and measured settling, prepares a CBF-off Cartesian hold, and switches back.
The next experiment's reference is the measured EE pose. Initialization is
blocking and a successful response means the reset finished. A failure after
handoff keeps Cartesian experiment commands blocked and requests a joint hold;
inspect the error and controller states before an explicit retry. There is no
automatic error recovery or automatic restart of a failed experiment.

## Run each experiment and return to the same joint pose

The service interface is unchanged. This is the existing full test displacement;
use it only after validating the smaller commissioning motion and path:

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: 0.1
y_move: 0.2
z_move: 0.0
cbf_active: false
Kmax: 0.02
alpha: 1.0"
```

A successful response acknowledges an atomic target/parameter request. It does
**not** mean the movement has finished. Watch `task_aborted`, `solver_status`,
`ee_target_distance` and measured `dq`; wait for settling, then call the exact
`initialize_joint_pose` command above. The initializer rejects motion that is
still underway. Repeat the movement with `cbf_active: true` and each of your three
chosen `alpha` values, keeping the same `Kmax`, direction, gains, start pose and
XYZ displacement. Call joint initialization after **every** movement, including
the last. It resets the redundant arm's joints, unlike simply reversing XYZ.

Displacements are relative to the previous commanded target, in the robot base
frame, in metres. Without a reset they accumulate. `Kmax` is in joules and `alpha`
in inverse seconds. The constrained direction is the fixed normalized launch
vector, not automatically the movement vector. The impedance reference filter,
not `publish_rate`, determines the movement's reference evolution.

`initialize_joint_pose` explicitly disables CBF for the joint return. No
directional-energy bound is claimed during that return. Afterward the controller
stays in measured-pose hold until another explicit experiment request.

Live plots:

```bash
rqt_plot /cbf_info/kinetic_energy \
  /cbf_info/directional_kinetic_energy /cbf_info/Kmax
rostopic echo /directional_cbf/diagnostics
```

The correct field name is `directional_kinetic_energy`. The total kinetic energy
can exceed `Kmax`: the barrier constrains the directional quantity. The launch
also accepts `plot_energies:=true` and `plot_joint_limits:=true`.

## Recorded data

The first initialization/experiment creates a unique session under
`~/.ros/directional_cbf_real` (override `log_dir:=...`). Subsequent services use
the same session. Stop the launch cleanly to drain queues and finalize health
counters. `record_cbf:=false` explicitly disables this recorder gate.

| File | Meaning |
| --- | --- |
| `cbf.csv` | Same original columns, plus trailing hardware diagnostics. Joint debug is valid with CBF on **and off** on hardware. |
| `robot_state.csv` | Joint position/velocity, measured and desired torque, EE transform, external torque/wrench estimates, contact/collision flags, errors, mode and communication success. Continues during the joint reset. |
| `events.csv` | Initialization and experiment-request events, stamped at ROS reception. These are not synchronous controller samples or automatic motion-completion events. |
| `metadata.json`, `robot.urdf` | Parameters and model used by the session. |
| `robot_model_state.json` | Initial robot-reported load, tool and frame configuration. |
| `health.json` | Queue drops, diagnostics gaps, state sequence gaps, stream age and clean-shutdown state. |

There is **no `contacts.csv` on hardware**: Gazebo collision-pair contact forces
have no identical physical sensor counterpart. Franka's estimated external wrench
and contact flags are recorded as their own signals. With `rosbag:=true`, a bag
of CBF, state, phase and ROS log topics is recorded from launch as well; by default
only CSV is enabled. `bag_prefix` specifies its output prefix.

CBF diagnostics pause while the joint controller owns the arm. Do not interpolate
that gap as zero energy or evidence that the CBF remained active. Use the continuous
state CSV and phase events to identify resets. They have different sampling and
timestamp semantics; a reception timestamp is not an exact synchronized pairing.

`tau_command` is the controller's raw gravity-free request. `tau_predicted` is the
predicted next libfranka desired torque after soft limits, filtering and rate
limiting. `tau_J_d` is the current desired torque reported by the robot;
`torque_prediction_error` compares it with the previous cycle's prediction.
`tau_J` is measured joint torque and includes gravity. Do not compare `tau_J`
directly with the gravity-free values as an actuator tracking error.

The existing root plotting script accepts the unchanged core columns:

```bash
python3 plot_directional_cbf.py /path/to/experiment_session --no-show
```

It retains the multiple-alpha comparison. Use its experiment-ID selection when
you want to exclude hold/reset intervals; IDs also increment for prepared holds,
so don't assume consecutive experiment IDs are all movement trials.

## Control behavior and limits of the result

Gazebo retains its existing OSQP problem, raw derivative estimates, gravity-shifted
URDF torque bounds and effort-controller reset. Torque/velocity limit enabling
was already in this branch (`disable_gazebo_torque_limits=false`) and stays enabled.
The Gazebo launch explicitly selects `runtime_environment=gazebo` for controller,
trajectory and recorder, avoiding parameters left by a previous hardware launch.
Its existing CSV core/debug/contact behavior is retained. The new message requires
rebuilding Gazebo nodes too. Regression tests support compatibility; physical
repeatability still needs rerunning the same Gazebo tests after the rebuild.

In real mode the decision variable is raw gravity-free torque `x`. For each joint,
FrankaHW's measured-state soft-limit envelope is intersected with the rate bound:

```
beta = 0.001 / (0.001 + 1/(2*pi*cutoff))  # beta=1 with cutoff>=1000
predicted = beta*x + (1-beta)*previous_tau_J_d
abs(predicted-previous_tau_J_d) <= 999.0 * 0.001
```

The QP imposes `a*(predicted-coriolis)+b+alpha*h >= 0` in that box. Gravity is not
added to the command or shifted into this hardware box; the robot provides gravity
compensation. The 1 ms filter/rate sample comes from libfranka 0.13.3's implementation,
even if a received controller period spans multiple milliseconds. Numerical
derivatives use received periods, with an EMA only in real mode.

The same strictly convex box-plus-halfspace QP is solved by its scalar KKT
multiplier (clipped affine solution, at most 80 bisections), avoiding OSQP
reinitialization/dynamic solver setup in the hardware callback. Gazebo continues
using OSQP. The physical-model residual is checked again on the predicted bounded
command. A failed/infeasible problem does not fall back to unfiltered nominal
impedance: it latches an abort and sends bounded viscous braking. With CBF off,
the hardware torque/rate envelope still applies.

Callback configuration uses a realtime buffer; torque publication avoids blocking
ROS publishing and the hardware update does not acquire the configuration mutex.
The code reports QP/update time and latches status 8 when pre-command computation
exceeds 900 microseconds. This is an overrun detector, **not a proven execution-time
bound** or a substitute for measuring the complete 1 kHz loop on the control PC.

| Status | Interpretation |
| --- | --- |
| 0 | Intentional CBF bypass |
| 1 | Feasible QP accepted by the model residual check |
| 3 | Invalid model/state/barrier quantities |
| 4 | Gazebo OSQP setup/update error |
| 5 | Infeasible/failed QP |
| 6 | Rejected torque bound or residual |
| 7 | Invalid hardware envelope or controller period |
| 8 | Pre-command computation budget exceeded |

`task_aborted` stays latched until an explicit experiment/reset request. After
saturation or infeasibility, the braking fallback does **not** guarantee the
barrier or energy dissipation. Independent robot reflexes remain enabled.

This patch does not make the continuous-time, nominal-model CBF a certified
hardware safety function. Discrete sampling, derivative estimation, actuator
prediction error, friction, payload errors, external contact and joint-limit
interventions still matter. Soft-limit torque constraints model FrankaHW's
intervention; they are **not a joint-position/velocity CBF with a proof of forward
invariance**. Start from `h>=0`, compare measured energy and predicted/returned
torque, and treat negative `h`, repeated prediction error, aborts or missed cycles
as failed trials to investigate. A positive model residual alone is insufficient.

Reference implementations used for the actuator prediction:
- [libfranka 0.13.3 control loop](https://github.com/frankarobotics/libfranka/blob/0.13.3/src/control_loop.cpp),
  [low-pass filter](https://github.com/frankarobotics/libfranka/blob/0.13.3/include/franka/lowpass_filter.h),
  [rate limits](https://github.com/frankarobotics/libfranka/blob/0.13.3/include/franka/rate_limiting.h).
- [ROS Noetic effort soft-limit implementation](https://github.com/ros-controls/ros_control/blob/noetic-devel/joint_limits_interface/include/joint_limits_interface/joint_limits_interface.h)
  and this fork's
  `FrankaHW::initROSInterfaces`/`enforceLimits`.
