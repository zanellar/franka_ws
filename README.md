# Franka ROS 1 Workspace with Control Barrier Function Controllers

This repository is a complete ROS 1 Catkin workspace for controlling Franka research robots with `ros_control`. It is based on `franka_ros` 0.10.1 and extends the original Cartesian impedance example controller with several Control Barrier Function (CBF) safety filters, trajectory generators, diagnostic messages, OSQP integration, and rosbag-based experiment logging.

The workspace supports both the Panda and FR3 model names used by this version of `franka_ros`:

- `robot:=panda`
- `robot:=fr3`

The newest controller in this repository is:

```text
cartesian_impedance_directional_kinetic_energy_cbf_controller
```

It limits translational kinetic energy along a configurable direction expressed in the robot base frame while retaining the nominal Cartesian impedance and null-space behavior.

> **Safety notice**
>
> This is experimental research software that commands joint torques on a physical robot. Validate every controller in a safe environment, start with conservative gains and energy limits, keep the emergency stop accessible, and follow the Franka operating and safety documentation. A CBF implementation, numerical solver, or model estimate does not by itself make an experiment safe.

---

## Contents

- [Repository overview](#repository-overview)
- [Workspace structure](#workspace-structure)
- [Available controllers](#available-controllers)
- [Directional kinetic-energy CBF](#directional-kinetic-energy-cbf)
- [Trajectory generators](#trajectory-generators)
- [Software requirements](#software-requirements)
- [Robot and network preparation](#robot-and-network-preparation)
- [Installation](#installation)
- [Required local configuration](#required-local-configuration)
- [Running a controller](#running-a-controller)
- [Directional controller examples](#directional-controller-examples)
- [Controller parameters](#controller-parameters)
- [Topics and diagnostics](#topics-and-diagnostics)
- [Dynamic reconfigure](#dynamic-reconfigure)
- [Recording experiments](#recording-experiments)
- [Development workflow](#development-workflow)
- [Troubleshooting](#troubleshooting)
- [Known limitations](#known-limitations)

---

## Repository overview

The repository contains a ROS 1 workspace rather than only a single ROS package. The original `franka_ros` packages are located under `src/franka_ros`, together with the added CBF-related packages and files.

Main additions relative to the upstream ROS 1 Franka integration include:

- Cartesian impedance controllers with CBF safety filters;
- an OSQP C++ dependency vendored inside `franka_example_controllers`;
- a custom `franka_msgs/Cbf` diagnostic message;
- a `franka_trajectory` package with several reference generators;
- launch files that start the robot interface, controller, trajectory publisher, RViz, dynamic reconfigure, and optional rosbag recording;
- a directional task-space kinetic-energy CBF controller.

The original Cartesian impedance controller is still available as a separate plugin. The CBF controllers are additional controller classes rather than replacements for the upstream controller.

---

## Workspace structure

```text
franka_ws/
├── README.md
├── README.txt
└── src/
    ├── CMakeLists.txt
    └── franka_ros/
        ├── franka_control/
        ├── franka_description/
        ├── franka_example_controllers/
        │   ├── cfg/
        │   ├── config/
        │   │   └── franka_example_controllers.yaml
        │   ├── include/franka_example_controllers/
        │   ├── launch/
        │   ├── lib/
        │   │   └── osqp-cpp/
        │   ├── msg/
        │   ├── scripts/
        │   ├── src/
        │   ├── CMakeLists.txt
        │   ├── franka_example_controllers_plugin.xml
        │   └── package.xml
        ├── franka_hw/
        ├── franka_msgs/
        │   └── msg/Cbf.msg
        ├── franka_ros/
        ├── franka_trajectory/
        │   ├── launch/
        │   └── src/
        └── franka_visualization/
```

### Important files

| File | Purpose |
|---|---|
| `src/franka_ros/franka_example_controllers/CMakeLists.txt` | Builds the controller library and links OSQP. |
| `src/franka_ros/franka_example_controllers/franka_example_controllers_plugin.xml` | Registers controller classes with `pluginlib`. |
| `src/franka_ros/franka_example_controllers/config/franka_example_controllers.yaml` | Defines controller names, plugin types, arm ID, and joint names. |
| `src/franka_ros/franka_example_controllers/launch/*.launch` | Starts each controller and its supporting nodes. |
| `src/franka_ros/franka_msgs/msg/Cbf.msg` | Diagnostic message used by the CBF controllers. |
| `src/franka_ros/franka_trajectory/src/*.cpp` | Cartesian reference trajectory publishers. |

---

## Available controllers

The controller names used by `controller_manager` are listed below.

| Controller name | Plugin class | Description |
|---|---|---|
| `cartesian_impedance_example_controller` | `CartesianImpedanceExampleController` | Original Cartesian spring-damper controller with null-space regulation. |
| `cartesian_impedance_cbf_controller` | `CartesianImpedanceCBFController` | Cartesian impedance controller with a total joint-space kinetic-energy CBF. |
| `cartesian_impedance_dead_zone_cbf_controller` | `CartesianImpedanceDZCBFController` | Total kinetic-energy CBF plus a Cartesian position dead zone. |
| `cartesian_impedance_cbf_interaction_controller` | `CartesianImpedanceCBFInteractionController` | Total kinetic-energy CBF including an estimated external-interaction term. |
| `cartesian_impedance_dead_zone_cbf_interaction_controller` | `CartesianImpedanceDZCBFInteractionController` | Dead-zone controller combined with external-interaction handling. |
| `cartesian_impedance_cbf_interaction_power_controller` | `CartesianImpedanceCBFInteractionPowerController` | Kinetic-energy and external-power limiting. |
| `cartesian_impedance_directional_kinetic_energy_cbf_controller` | `CartesianImpedanceDirectionalKineticEnergyCBFController` | Limits translational kinetic energy along a configurable task-space direction. |

Each CBF controller has a launch file with the same base name under:

```text
src/franka_ros/franka_example_controllers/launch/
```

---

## Directional kinetic-energy CBF

The directional controller projects the translational end-effector Jacobian along a unit direction \(\hat d\):

```text
J_dir = d_hat^T J_translation
```

It then computes:

```text
v_dir      = J_dir q_dot
lambda_dir = 1 / (J_dir M^-1 J_dir^T)
K_dir      = 0.5 lambda_dir v_dir^2
h          = Kmax - K_dir
```

The safe set is:

```text
h >= 0  <=>  K_dir <= Kmax
```

A quadratic program minimally modifies the nominal torque command while enforcing the CBF condition and joint torque/rate limits. The nominal command remains the Cartesian impedance plus null-space controller.

### Direction convention

The parameters

```text
direction_x
direction_y
direction_z
```

define a vector in the robot base frame. The controller normalizes the vector internally. For example:

```text
[1, 0, 0]   positive base-frame x
[0, 1, 0]   positive base-frame y
[0, 0, 1]   positive base-frame z
[-1, 0, 0]  negative base-frame x
```

A zero or near-zero vector is invalid.

### Numerical derivatives

The controller estimates the directional Jacobian derivative and effective-mass derivative numerically. These estimates are filtered using an exponential moving average controlled by:

```text
derivative_filter_alpha
```

A smaller value produces stronger smoothing and more delay. A larger value follows changes more quickly but is more sensitive to noise.

---

## Trajectory generators

The `franka_trajectory` package publishes Cartesian equilibrium poses on:

```text
/trajectory_publisher/equilibrium_pose
```

Available executables are:

| Trajectory | Description |
|---|---|
| `hold` | Publishes a fixed Cartesian pose. |
| `circular` | Circular motion in the Cartesian `y-z` plane. |
| `chirp` | Oscillatory motion with changing frequency. |
| `square_wave` | Discontinuous square-wave reference along one Cartesian axis. |
| `tension` | Vertical displacement sequence intended for tension/contact experiments. |

Select a trajectory with:

```text
trajectory:=hold
```

The reference publication rate is controlled by:

```text
publish_rate:=100
```

> `square_wave`, aggressive chirps, and large circular trajectories can generate abrupt or large commands. Inspect the source constants and test with conservative controller gains before using them on hardware.

---

## Software requirements

This code is based on the ROS 1 `franka_ros` 0.10.1 generation. Use a system compatible with that software stack.

Required components include:

- Linux;
- ROS 1 with Catkin;
- a compatible `libfranka` installation;
- Eigen 3;
- a C++14 compiler for the Franka packages;
- CMake 3.16 or newer for the vendored OSQP integration;
- ROS packages used by `franka_ros`, including `ros_control`, `controller_manager`, `dynamic_reconfigure`, `pluginlib`, `realtime_tools`, `geometry_msgs`, `sensor_msgs`, `tf`, and `tf_conversions`;
- RViz and `rqt_reconfigure` if the corresponding launch nodes are enabled;
- `rosbag` for experiment recording.

A typical ROS dependency installation step is:

```bash
cd ~/franka_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
```

This command may not install all non-ROS dependencies used by the vendored OSQP build. Review any unresolved dependencies printed by `rosdep`.

### libfranka compatibility

The included packages search for Franka versions compatible with the original 0.10.x workspace. Do not mix arbitrary current versions of `libfranka`, robot firmware, and this ROS 1 snapshot. Confirm the compatibility matrix for the exact robot and firmware used in the laboratory.

---

## Robot and network preparation

Before starting a hardware controller:

1. Connect the control computer to the robot control network.
2. Configure the computer network interface so it can reach the robot IP address.
3. Confirm connectivity, for example:

   ```bash
   ping 172.16.0.2
   ```

4. Unlock the robot brakes and enable the external control interface using the Franka web interface.
5. Confirm that no other process is already controlling the robot.
6. Put the robot in a collision-free initial configuration.
7. Keep the emergency stop and operator interface accessible.

The example launch commands in this README use:

```text
robot_ip:=172.16.0.2
```

Change this value to the robot address in your laboratory.

---

## Installation

### 1. Extract or clone the workspace

```bash
cd ~
unzip franka_ws-directional-kinetic-energy-cbf.zip
cd franka_ws
```

The workspace root is the directory containing `src/`.

### 2. Source ROS

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

### 3. Install dependencies

```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
```

### 4. Build and install

The original workflow uses the Catkin install space:

```bash
catkin_make install
```

For a clean rebuild:

```bash
rm -rf build devel install
catkin_make install
```

### 5. Source the workspace

```bash
source install/setup.bash
```

The old README used `install/setup.sh`; `setup.bash` is generally more convenient in Bash. Either is valid when generated by Catkin.

To source the workspace automatically in new terminals:

```bash
echo 'source ~/franka_ws/install/setup.bash' >> ~/.bashrc
```

Adjust the path if the workspace is located elsewhere.

---

## Required local configuration

### Rosbag output paths

The CBF launch files contain an absolute rosbag output path inherited from the original experimental workspace:

```text
/home/dlogmans/Desktop/Master_Thesis_DDLogmans/Software/Rundata/
```

This path must be changed before using rosbag recording on another computer.

Search for it with:

```bash
grep -R "/home/dlogmans" -n \
  src/franka_ros/franka_example_controllers/launch
```

Edit each launch file you intend to use and replace the prefix with an existing writable directory, for example:

```text
/home/<user>/franka_bags/
```

Create the directory:

```bash
mkdir -p ~/franka_bags
```

Alternatively, disable recording at launch time:

```text
rosbag:=false
```

### Abseil shared-library workaround

The vendored `osqp-cpp` build depends on Abseil. In the original workspace, some `libabsl*.so` files may appear in the Catkin development space but not in the install space. If the controller fails at runtime with an error such as:

```text
error while loading shared libraries: libabsl_*.so: cannot open shared object file
```

copy the generated libraries into the install space:

```bash
find devel/lib -maxdepth 1 -name 'libabsl*.so*' -exec cp -av {} install/lib/ \;
```

Then refresh the shell environment:

```bash
source install/setup.bash
```

You can verify the controller library dependencies with:

```bash
ldd install/lib/libfranka_example_controllers.so | grep 'not found'
```

If the command prints nothing, all linked shared libraries were resolved.

### Build-directory artifacts

The repository contains a vendored OSQP source tree and may also contain generated build artifacts from the original development machine. When diagnosing build problems, remove top-level Catkin build products first:

```bash
rm -rf build devel install
catkin_make install
```

Avoid manually editing files under generated `build/` directories.

---

## Running a controller

General pattern:

```bash
roslaunch franka_example_controllers \
  <controller_launch_file>.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=panda \
  trajectory:=hold \
  Kmax:=1.5 \
  cbf_active:=true \
  alpha:=1.0 \
  rosbag:=false
```

Use `robot:=fr3` for an FR3 model. The launch files derive `arm_id` from `robot` by default.

### Run the original Cartesian impedance controller

```bash
roslaunch franka_example_controllers \
  cartesian_impedance_example_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=panda
```

### Run the total kinetic-energy CBF controller

```bash
roslaunch franka_example_controllers \
  cartesian_impedance_cbf_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=panda \
  trajectory:=hold \
  Kmax:=1.5 \
  alpha:=1.0 \
  cbf_active:=true \
  rosbag:=false
```

### Compare CBF enabled and disabled

The launch interface permits the same controller to be run with filtering disabled:

```text
cbf_active:=false
```

This is useful for controlled comparisons, but disabling the CBF removes the corresponding safety constraint.

---

## Directional controller examples

### Panda, positive x direction

```bash
roslaunch franka_example_controllers \
  cartesian_impedance_directional_kinetic_energy_cbf_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=panda \
  trajectory:=hold \
  Kmax:=0.5 \
  alpha:=1.0 \
  direction_x:=1.0 \
  direction_y:=0.0 \
  direction_z:=0.0 \
  cbf_active:=true \
  rosbag:=false
```

### FR3, negative y direction

```bash
roslaunch franka_example_controllers \
  cartesian_impedance_directional_kinetic_energy_cbf_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=fr3 \
  trajectory:=hold \
  Kmax:=0.5 \
  alpha:=1.0 \
  direction_x:=0.0 \
  direction_y:=-1.0 \
  direction_z:=0.0 \
  cbf_active:=true \
  rosbag:=false
```

### Diagonal direction

```bash
roslaunch franka_example_controllers \
  cartesian_impedance_directional_kinetic_energy_cbf_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=fr3 \
  trajectory:=circular \
  Kmax:=0.75 \
  alpha:=2.0 \
  direction_x:=1.0 \
  direction_y:=1.0 \
  direction_z:=0.0 \
  derivative_filter_alpha:=0.05 \
  mobility_epsilon:=1e-8 \
  cbf_active:=true \
  rosbag:=true
```

The vector `[1, 1, 0]` is normalized by the controller.

---

## Controller parameters

### Common parameters

| Parameter | Type | Typical default | Meaning |
|---|---:|---:|---|
| `robot_ip` | string | none | Robot control address. Required by hardware launch. |
| `robot` | string | `panda` | Robot model: `panda` or `fr3`. |
| `arm_id` | string | value of `robot` | Prefix used for model, state, and joint handles. |
| `load_gripper` | bool | inherited | Whether to start the gripper integration. |
| `trajectory` | string | `circular` | Executable from `franka_trajectory`. |
| `publish_rate` | double | `100` | Reference trajectory publication rate in hertz. |
| `rosbag` | bool | `true` | Enables or disables `rosbag record -a`. |
| `alpha` | double | `1.0` | CBF class-K gain/aggressiveness parameter. |
| `cbf_active` | bool | `true` | Applies the CBF-filtered torque when true. |
| `Kmax` | double | `1.5` | Energy limit in joules. Meaning depends on controller. |
| `damping_ratio` | double | `1.0` | Scale used for Cartesian damping gains. |

### Directional controller parameters

| Parameter | Type | Default | Meaning |
|---|---:|---:|---|
| `direction_x` | double | `1.0` | Base-frame x component of the constrained direction. |
| `direction_y` | double | `0.0` | Base-frame y component of the constrained direction. |
| `direction_z` | double | `0.0` | Base-frame z component of the constrained direction. |
| `mobility_epsilon` | double | `1e-8` | Lower numerical regularization threshold for directional mobility. |
| `derivative_filter_alpha` | double | `0.05` | EMA coefficient for numerical derivative estimates; must be in `(0, 1]`. |

### Other controller-specific parameters

| Controller family | Additional parameters |
|---|---|
| Dead-zone controllers | `deadzone` |
| Dead-zone test input | `power` |
| Interaction-power controller | `Pmax` |

Inspect the selected launch file for the authoritative list:

```bash
grep '<arg name=' \
  src/franka_ros/franka_example_controllers/launch/<file>.launch
```

---

## Topics and diagnostics

### Reference input

The CBF Cartesian impedance controllers subscribe to:

```text
/trajectory_publisher/equilibrium_pose
```

Message type:

```text
geometry_msgs/PoseStamped
```

### CBF diagnostics

The controllers publish:

```text
/cbf_info
```

or the equivalent name resolved from the controller node namespace.

Message type:

```text
franka_msgs/Cbf
```

The message contains:

```text
std_msgs/Header header
float64[7] u_des
float64[7] u_cbf
float64[7] u_measured
float64[7] u_saturated
float64[7] u_ext
float64 h
uint8 solver_status
```

For the directional controller:

- `u_des` is the nominal torque command;
- `u_cbf` is the torque after directional CBF filtering;
- `u_measured` is the measured joint torque;
- `u_saturated` is the final rate-limited command;
- `h = Kmax - K_dir`;
- `solver_status` reports the controller's QP outcome.

Inspect live data with:

```bash
rostopic echo /cbf_info
```

List active topics with:

```bash
rostopic list
```

Plot selected fields with:

```bash
rqt_plot /cbf_info/h
```

---

## Dynamic reconfigure

The controller launch files start `rqt_reconfigure`. The Cartesian compliance callback controls:

- translational stiffness;
- rotational stiffness;
- null-space stiffness.

Damping is derived from stiffness and the `damping_ratio` launch parameter.

The directional CBF parameters are currently read during controller initialization. Restart the controller after changing:

- `Kmax`;
- `alpha`;
- direction components;
- `mobility_epsilon`;
- `derivative_filter_alpha`.

---

## Recording experiments

The CBF launch files can start:

```bash
rosbag record -a
```

This records all active ROS topics. Enable recording with:

```text
rosbag:=true
```

Disable it with:

```text
rosbag:=false
```

Before enabling it, replace the hard-coded output path as described in [Required local configuration](#required-local-configuration).

Inspect a bag:

```bash
rosbag info <bagfile>.bag
```

Replay a bag without commanding a robot:

```bash
rosbag play <bagfile>.bag
```

Suggested signals for validation include:

- `/cbf_info/h`;
- nominal and filtered torque vectors;
- robot joint velocity and torque state;
- end-effector pose;
- the published equilibrium pose.

For a valid directional-energy experiment, verify that `h` remains non-negative within an acceptable numerical tolerance and that solver failures are absent.

---

## Development workflow

### Add a controller source file

New controller implementations must be added to:

```text
src/franka_ros/franka_example_controllers/src/
```

with the corresponding header under:

```text
src/franka_ros/franka_example_controllers/include/franka_example_controllers/
```

Then update all of the following:

1. `franka_example_controllers/CMakeLists.txt`;
2. `franka_example_controllers_plugin.xml`;
3. `config/franka_example_controllers.yaml`;
4. add a launch file under `launch/`.

### Rebuild after changing messages or controllers

```bash
cd ~/franka_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rm -rf build devel install
catkin_make install
source install/setup.bash
```

### Confirm plugin registration

```bash
rospack plugins --attrib=plugin controller_interface \
  | grep franka_example_controllers
```

### Confirm the built library exists

```bash
ls -l install/lib/libfranka_example_controllers.so
```

---

## Troubleshooting

### `roslaunch` cannot find a package

Ensure both ROS and the workspace are sourced:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/franka_ws/install/setup.bash
```

Check:

```bash
rospack find franka_example_controllers
```

### Controller type is not found

Check that:

- the class exists in `franka_example_controllers_plugin.xml`;
- the YAML `type` exactly matches the plugin XML name;
- the source is included in `CMakeLists.txt`;
- the controller library was rebuilt and the new install space was sourced.

### Controller cannot obtain model, state, or joint handles

Check that `robot` and `arm_id` match the loaded robot model:

```text
robot:=panda  arm_id:=panda
```

or:

```text
robot:=fr3  arm_id:=fr3
```

The YAML joint names are generated from `arm_id`.

### Robot connection fails

Check:

- robot IP address;
- Ethernet interface configuration;
- firewall rules;
- Franka web-interface state;
- robot brakes and external-control activation;
- firmware and `libfranka` compatibility.

### Abseil library is missing

Apply the copy workaround in [Abseil shared-library workaround](#abseil-shared-library-workaround), then run:

```bash
ldd install/lib/libfranka_example_controllers.so | grep 'not found'
```

### OSQP does not return an optimal solution

Possible causes include:

- an infeasible energy constraint combined with torque/rate limits;
- an excessively small `Kmax`;
- noisy numerical derivatives;
- directional mobility near zero;
- an excessively strict or aggressive experiment;
- the solver time limit being reached.

Start with:

- `trajectory:=hold`;
- a moderate `Kmax`;
- `alpha:=1.0`;
- a principal-axis direction such as `[1,0,0]`;
- conservative Cartesian stiffness;
- `rosbag:=false` while debugging startup.

### Directional mobility warning

The selected direction may be weakly controllable at the current configuration. Change the robot configuration or direction and inspect the regularization parameter. Do not simply increase `mobility_epsilon` without understanding the physical effect on the effective-mass estimate.

### `h` becomes negative

A small negative value can result from sampling, model mismatch, solver tolerance, torque-rate limiting, or numerical derivative error. A sustained or large violation requires stopping the experiment and investigating:

- QP status;
- torque saturation/rate limits;
- derivative filtering;
- model and frame conventions;
- `Kmax` and `alpha`;
- timing overruns.

### RViz fails but the controller starts

RViz is not required for torque control. Launch RViz separately or comment out its node while diagnosing visualization configuration.

---

## Known limitations

- This is an experimental snapshot, not a maintained upstream Franka distribution.
- The repository is based on ROS 1 and `franka_ros` 0.10.1-era interfaces.
- The OSQP and Abseil integration is vendored and may require the shared-library workaround described above.
- Several launch files contain a developer-specific absolute rosbag output path.
- Existing controllers initialize OSQP inside the control workflow; real-time guarantees should be evaluated carefully.
- The directional controller estimates derivatives numerically, which introduces noise and delay.
- The directional vector is static during a controller run. A time-varying direction would require the derivative of the normalized direction to be included consistently.
- The directional energy uses only translational task-space motion. Rotational energy is not constrained by this controller.
- `franka_msgs/Cbf` contains only one scalar `h`; it does not expose directional velocity or effective mass directly.
- Rosbag logging uses `record -a`, which can generate large files and increase system load.
- The workspace should be validated in simulation or a controlled test setup before physical experiments, but the provided hardware launch files are the primary tested workflow.

---

## Minimal startup checklist

```text
[ ] Compatible ROS 1 and libfranka installed
[ ] Dependencies installed with rosdep
[ ] Rosbag path changed or rosbag:=false
[ ] catkin_make install completed
[ ] install/setup.bash sourced
[ ] No missing libraries in ldd output
[ ] Robot is reachable over Ethernet
[ ] Correct robot and arm_id selected
[ ] Conservative stiffness, Kmax, and trajectory selected
[ ] Emergency stop accessible
[ ] Controller first tested with trajectory:=hold
```

---

## Quick-start command

After completing installation and robot preparation:

```bash
cd ~/franka_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

roslaunch franka_example_controllers \
  cartesian_impedance_directional_kinetic_energy_cbf_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=fr3 \
  trajectory:=hold \
  Kmax:=0.5 \
  alpha:=1.0 \
  direction_x:=1.0 \
  direction_y:=0.0 \
  direction_z:=0.0 \
  cbf_active:=true \
  rosbag:=false
```

Start with the robot stationary and verify `/cbf_info`, controller state, and solver status before commanding a moving trajectory.
