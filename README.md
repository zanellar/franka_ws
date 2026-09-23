# Franka ROS 1 Workspace with Kinetic-Energy CBF Controllers

ROS Noetic / Gazebo experiments with Cartesian impedance and total or directional
kinetic-energy Control Barrier Functions (CBFs).

This guide targets branch **`franka_ws_013_gazebo`**, workspace
`~/Riccardo/franka_ws_013`, and libfranka **0.13.3** installed at
`~/Riccardo/libfranka-0.13.3/install`. The build environment is Ubuntu 20.04,
GCC/G++ 10, C++17, Gazebo 11 and SDFormat 9. Examples use an FR3 without a gripper.

- [Installation](#installation)
- [Tests in Gazebo](#tests-in-gazebo)
- [Test in Real robot](#test-in-real-robot)
- [Plot data](#plot-data)

## Installation

### 1. Install ROS Noetic and build tools

``` bash
sudo apt update

sudo apt install -y \
  ros-noetic-desktop-full \
  python3-rosdep \
  python3-catkin-tools \
  python3-vcstool \
  python3-numpy \
  python3-matplotlib \
  build-essential \
  gcc-10 \
  g++-10 \
  cmake \
  git \
  pkg-config \
  libeigen3-dev \
  libpoco-dev \
  libfmt-dev \
  libsdformat9-dev \
  gazebo11 \
  libgazebo11-dev \
  ethtool \
  rt-tests
```

Initialize rosdep once:

``` bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi

rosdep update
```

Source ROS:

``` bash
source /opt/ros/noetic/setup.bash
```

### 2. Clone the workspace

``` bash
mkdir -p ~/Riccardo

git clone \
  --branch franka_ws_013_gazebo \
  --single-branch \
  https://github.com/zanellar/franka_ws.git \
  ~/Riccardo/franka_ws_013

cd ~/Riccardo/franka_ws_013
git branch --show-current
```

Expected:

``` text
franka_ws_013_gazebo
```

### 3. Build libfranka 0.13.3

``` bash
git clone \
  --recursive \
  --branch 0.13.3 \
  --single-branch \
  https://github.com/frankarobotics/libfranka.git \
  ~/Riccardo/libfranka-0.13.3

cd ~/Riccardo/libfranka-0.13.3
git submodule update --init --recursive
```

Configure and install:

``` bash
rm -rf build install

cmake -S . -B build \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-10 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-10 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install" \
  -DBUILD_TESTS=OFF \
  -DBUILD_EXAMPLES=ON

cmake --build build -j"$(nproc)"
cmake --install build
```

Verify:

``` bash
ls -l "$HOME/Riccardo/libfranka-0.13.3/install/lib/libfranka.so"*
```

The installed ABI must include:

``` text
libfranka.so.0.13
```

### 4. Install ROS package dependencies

The joint initialization service requires the standard effort trajectory controller:

```bash
sudo apt install ros-noetic-joint-trajectory-controller
```

This package provides `effort_controllers/JointTrajectoryController`. If it is
installed while Gazebo is running, restart the launch once so the controller
manager discovers it. Installing this binary package alone does not require
recompiling this workspace.

The custom libfranka installation replaces the ROS binary `libfranka`
package for this workspace.

``` bash
cd ~/Riccardo/franka_ws_013
source /opt/ros/noetic/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro noetic \
  --skip-keys libfranka \
  -y
```

### 5. Required CMake configuration

This branch must compile the controller stack and vendored dependencies
as C++17.

In `src/franka_ros/franka_example_controllers/CMakeLists.txt`, ensure:

``` cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Franka 0.13.3 REQUIRED)

set(OSQP-CPP_BUILD_TESTS OFF CACHE BOOL
    "Disable embedded osqp-cpp tests" FORCE)

set(ABSL_BUILD_TESTING OFF CACHE BOOL
    "Disable embedded Abseil tests" FORCE)
```

In
`src/franka_ros/franka_example_controllers/lib/osqp-cpp/CMakeLists.txt`,
ensure:

``` cmake
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ language standard" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON CACHE BOOL "Require selected C++ standard" FORCE)
set(CMAKE_CXX_EXTENSIONS OFF CACHE BOOL "Disable compiler-specific extensions" FORCE)

set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL
    "Propagate the selected C++ standard to Abseil targets" FORCE)
```

Pin Abseil instead of fetching `origin/master`:

``` cmake
FetchContent_Declare(
  abseil-cpp
  GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
  GIT_TAG        20240116.2
  GIT_SHALLOW    TRUE
)
```

In `src/franka_ros/franka_gazebo/CMakeLists.txt`, ensure:

``` cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(sdformat9 REQUIRED CONFIG)
```

If Gazebo exports stale SDFormat 9.8 paths, remove them before targets
are created:

``` cmake
list(REMOVE_ITEM catkin_INCLUDE_DIRS
  "/usr/include/sdformat-9.8"
  "/usr/include/sdformat-9.8/sdf/.."
)

list(REMOVE_ITEM catkin_LIBRARIES
  "/usr/lib/x86_64-linux-gnu/libsdformat9.so.9.8.0"
)
```

Link SDFormat through its imported target:

``` cmake
target_link_libraries(franka_hw_sim
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  sdformat9::sdformat9
)
```

### 6. Optional clean build

``` bash
cd ~/Riccardo/franka_ws_013

rm -rf \
  src/franka_ros/franka_example_controllers/lib/osqp-cpp/build \
  build \
  devel \
  install
```

The cleanup removes generated build/install directories. Skip it for normal incremental builds.

### 7. Build and install the Catkin workspace

``` bash
cd ~/Riccardo/franka_ws_013

source /opt/ros/noetic/setup.bash

export CC=/usr/bin/gcc-10
export CXX=/usr/bin/g++-10
export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

catkin_make install \
  -DCMAKE_INSTALL_PREFIX="$PWD/install" \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-10 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-10 \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_CXX_STANDARD_REQUIRED=ON \
  -DCMAKE_CXX_EXTENSIONS=OFF \
  -DABSL_PROPAGATE_CXX_STD=ON \
  -DABSL_BUILD_TESTING=OFF \
  -DOSQP-CPP_BUILD_TESTS=OFF \
  -DCMAKE_PREFIX_PATH="$FRANKA_013_PREFIX;/opt/ros/noetic" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_ENABLE_TESTING=OFF
```

A successful Abseil configuration should report:

``` text
Performing Test ABSL_INTERNAL_AT_LEAST_CXX17 - Success
```

### 8. Copy Abseil shared libraries

``` bash
cd ~/Riccardo/franka_ws_013
cp -a devel/lib/libabsl*.so* install/lib/
```

Run this after every clean build.


### Runtime environment

Use this block in every fresh terminal:

``` bash
cd ~/Riccardo/franka_ws_013

source /opt/ros/noetic/setup.bash
source install/setup.bash

export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"
export LD_LIBRARY_PATH="$PWD/install/lib:$FRANKA_013_PREFIX/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
```

Create a helper script:

``` bash
cat > ~/Riccardo/franka_ws_013/env.sh <<'EOS'
#!/usr/bin/env bash

cd "$HOME/Riccardo/franka_ws_013" || return 1

source /opt/ros/noetic/setup.bash
source install/setup.bash

export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"
export LD_LIBRARY_PATH="$PWD/install/lib:$FRANKA_013_PREFIX/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
EOS

chmod +x ~/Riccardo/franka_ws_013/env.sh
```

Use:

``` bash
source ~/Riccardo/franka_ws_013/env.sh
```

Do not source an older Franka workspace in the same terminal.


### Build verification

``` bash
source ~/Riccardo/franka_ws_013/env.sh

rospack find franka_control
rospack find franka_example_controllers
rospack find franka_gazebo
```

Check unresolved libraries:

``` bash
ldd install/lib/franka_control/franka_control_node | grep "not found"
ldd install/lib/libfranka_example_controllers.so | grep "not found"
ldd install/lib/libfranka_hw_sim.so | grep "not found"
```

Expected: no output.

Verify libfranka:

``` bash
ldd install/lib/franka_control/franka_control_node | grep libfranka
ldd install/lib/libfranka_example_controllers.so | grep libfranka
ldd install/lib/libfranka_hw_sim.so | grep libfranka
```

Expected ABI:

``` text
libfranka.so.0.13
```

Verify SDFormat:

``` bash
ldd install/lib/libfranka_hw_sim.so | grep sdformat
```

Expected ABI:

``` text
libsdformat9.so.9
```

Check that stale paths are absent:

``` bash
grep -RIn \
  -E 'sdformat-9\.8|libsdformat9\.so\.9\.8\.0' \
  build/franka_ros/franka_gazebo \
  2>/dev/null
```

Expected: no output.



Check controller registration, the installed launch, and service definitions:

```bash
rospack find joint_trajectory_controller
rospack plugins --attrib=plugin controller_interface | grep franka_example_controllers
ls -l install/lib/libfranka_example_controllers.so install/lib/libfranka_hw_sim.so
ls -l install/share/franka_example_controllers/launch/*cbf*gazebo.launch
rossrv show franka_trajectory/SetLinearCommand
rossrv show franka_trajectory/InitializeJointPose
```

`SetLinearCommand` must include `x_move`, `y_move`, `z_move`, `cbf_active`,
`Kmax`, and `alpha`. `InitializeJointPose` must include `q` and `duration`.
`rospack find franka_example_controllers` should resolve to this workspace's
`install/share/franka_example_controllers` directory.

After changing C++, service definitions or launch files, rerun the full
`catkin_make install` command above, copy the Abseil libraries, and source
`env.sh` again. Restart affected nodes after installation. Editing files in
`src` alone does not update the copies used from `install`.

If a library is missing, inspect the `ldd` output above. For Abseil, repeat the
copy step. If compilation still refers to an old C++ standard or SDFormat path,
perform the optional clean build. Do not create fake SDFormat compatibility
symlinks. Do not source another Franka workspace in the same terminal.

## Tests in Gazebo

Run one Gazebo experiment at a time. Use a separate terminal for the launch,
live plots, and service calls. In **every new terminal** run:

```bash
source "$HOME/Riccardo/franka_ws_013/env.sh"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/ros/noetic/lib/x86_64-linux-gnu"
```

Wait for the controllers to start. Send movement commands individually and wait
for the robot to settle between them. `success: true` from an experiment command
means the target was accepted, not that the movement has finished.

With `trajectory:=linear start_trajectory:=true`, the service accepts XYZ target
increments in meters in the robot base frame. Each request increments the
**previous target**, not the current measured EE position. Repeated calls
accumulate displacement; orientation is preserved. The reference is filtered
by the Cartesian controller, so this is not a constant-speed trajectory with a
specified duration. Use zero for unused axes.

### Total energy

The total-energy controller constrains the robot's joint-space kinetic energy.
`Kmax` is in joules and `cbf_active` selects whether the CBF torque filter is used.

#### Launch and live plot

**Terminal 1:**

```bash
roslaunch franka_example_controllers \
  cartesian_impedance_cbf_controller_gazebo.launch \
  trajectory:=linear \
  start_trajectory:=true \
  cbf_active:=false \
  Kmax:=20.0 \
  alpha:=1.0 \
  headless:=false \
  rviz:=false
```

**Terminal 2:**

```bash
rqt_plot /cbf_info/kinetic_energy /cbf_info/Kmax
```

#### Execute the test

**Terminal 3 - forward, CBF off:**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: 0.20
y_move: 0.0
z_move: 0.0
cbf_active: false
Kmax: 20.0
alpha: 1.0"
```

**After settling - return, CBF off:**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: -0.20
y_move: 0.0
z_move: 0.0
cbf_active: false
Kmax: 20.0
alpha: 1.0"
```

**After settling - forward, CBF on:**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: 0.20
y_move: 0.0
z_move: 0.0
cbf_active: true
Kmax: 0.01
alpha: 1.0"
```

The request changes `cbf_active`, `Kmax` and `alpha` through the service; no
launch restart is needed for these fields. Use a positive `Kmax` and `alpha`.
Returning the Cartesian target does not guarantee identical joint angles on a
redundant robot.

#### Launch arguments

Pass arguments as `name:=value` on the `roslaunch` command.

| Argument | Default | Usage |
| --- | --- | --- |
| `robot` | `fr3` | Robot identifier; this launch loads the FR3 URDF. |
| `arm_id` | value of `robot` | Joint and frame prefix; keep consistent with the model. |
| `use_gripper` | `false` | Include the hand in the Gazebo model. |
| `headless` | `false` | `true` hides the Gazebo GUI. |
| `paused` | `false` | Keep physics paused after spawning when `true`. |
| `rviz` | `true` | Start RViz. Independent of the Gazebo GUI. |
| `cbf_active` | `false` | Initial CBF state; overridden by experiment requests. |
| `Kmax` | `1.0` | Initial kinetic-energy threshold in J. |
| `alpha` | `1.0` | Initial CBF gain. |
| `damping_ratio` | `1.0` | Cartesian impedance damping setting; not Gazebo joint friction. |
| `qp_time_limit` | `0.01` | QP time budget in seconds for the total-energy controller. |
| `start_trajectory` | `false` | Start the selected trajectory publisher. |
| `trajectory` | `hold` | Use `linear` for XYZ experiment service commands. |
| `publish_rate` | `100` | Trajectory publisher frequency in Hz, not controller frequency. |

The total-energy launch does not expose the directional CSV recorder or the
joint initialization enable argument. Use its live energy topics for this test;
the CSV workflow below belongs to the directional controller.

### Directional kinetic energy

The directional controller constrains translational kinetic energy along the
selected direction in the robot base frame:

```text
h = Kmax - kinetic_energy_dir
```

`h >= 0` means the recorded energy is at or below `Kmax`. The direction vector
is normalized internally and must be nonzero. It is independent of the commanded
XYZ displacement: `direction_x:=1 direction_y:=0 direction_z:=0` limits x-directed
energy even when the target moves along both x and y. It does not limit total
kinetic energy or energy along every direction.

#### Launch and live plot

**Terminal 1:**

```bash
roslaunch franka_example_controllers \
  cartesian_impedance_directional_kinetic_energy_cbf_controller_gazebo.launch \
  trajectory:=linear \
  start_trajectory:=true \
  cbf_active:=false \
  Kmax:=20.0 \
  alpha:=1.0 \
  direction_x:=1.0 \
  direction_y:=0.0 \
  direction_z:=0.0 \
  headless:=false \
  rviz:=false \
  enable_joint_initialization:=true \
  record_cbf:=true \
  log_dir:="$HOME/Riccardo/franka_ws_013/data"
```

Keep the trailing backslash on every continued line except the last. Use an
absolute `log_dir`; `home/user/...` without a leading `/` is a relative path.

**Terminal 2:**

```bash
rqt_plot /cbf_info/kinetic_energy /cbf_info/directional_kinetic_energy /cbf_info/Kmax
```

#### Execute the test

**Terminal 3 - forward, CBF off:**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: 0.20
y_move: 0.0
z_move: 0.0
cbf_active: false
Kmax: 0.05
alpha: 1.0"
```

**After settling - return, CBF off:**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: -0.20
y_move: 0.0
z_move: 0.0
cbf_active: false
Kmax: 0.05
alpha: 1.0"
```

**After settling - forward, CBF on:**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: 0.20
y_move: 0.0
z_move: 0.0
cbf_active: true
Kmax: 0.05
alpha: 1.0"
```

These reproduce the commands in `how_to_run.md`; the displacement is an example,
not a guarantee of avoiding joint stops or collisions for every initial posture.
A failed directional CBF calculation aborts the task while keeping the node
available. Read the reported error and submit a new experiment with revised
parameters once the cause is resolved.

#### Launch arguments

The common arguments in the total-energy table also apply here **except
`qp_time_limit`**, which is not an argument of the directional launch. Its
additional arguments are:

| Argument | Default | Usage |
| --- | --- | --- |
| `direction_x` | `1.0` | x component of the base-frame energy direction. |
| `direction_y` | `0.0` | y component of the base-frame energy direction. |
| `direction_z` | `0.0` | z component of the base-frame energy direction. |
| `mobility_epsilon` | `1e-8` | Numerical threshold used for directional mobility validity. |
| `derivative_filter_alpha` | `0.05` | Filter weight for numerical derivatives; separate from the CBF gain `alpha`. |
| `enable_joint_initialization` | `true` | Enable joint initialization through `linear`; other trajectory publishers do not implement it. |
| `record_cbf` | `true` | Start the recorder node and contact bridge; files open only when recording is triggered. |
| `log_dir` | `$HOME/.ros/directional_cbf` | Root directory for automatically created session folders. |
| `state_publish_rate` | `1000` | Franka state publication frequency in Hz; not a CSV downsampling setting. |
| `gazebo_world` | `default` | World name used by the contact bridge; does not select a world file. |
| `joint_limit_diagnostics` | `true` | Publish live distances of joint positions from URDF limits. |
| `plot_joint_limits` | `false` | Open the corresponding preconfigured rqt plot; needs diagnostic topics and a desktop. |

The launch also sets `disable_gazebo_torque_limits=true` directly. It is a ROS
parameter, not a `roslaunch` argument. This simulation configuration is not a
hardware deployment configuration.

#### Services and initialization

| Service | Request | Result / purpose |
| --- | --- | --- |
| `/trajectory_publisher/set_experiment_command` | `x_move`, `y_move`, `z_move` in m; `cbf_active`; positive `Kmax` in J; positive `alpha` | Returns `success`, `applied_pose`, `message`; accepts a new target and CBF settings. Starts recording first when enabled. |
| `/trajectory_publisher/initialize_joint_pose` | `q`: 7 angles in rad; `duration`: simulated seconds | Moves to a joint configuration without CBF, waits for settling and returns to Cartesian control. |
| `/directional_cbf_recorder/start` | Empty `std_srvs/Trigger` request | Starts recording manually; repeated calls reuse the same session. |
| `/controller_manager/list_controllers` | Empty request | Inspect loaded controllers and their states. |
| `/controller_manager/list_controller_types` | Empty request | Verify available controller plugins. |

For joint initialization, keep Gazebo unpaused and wait until the robot is
stationary. Example using the standard posture:

```bash
rosservice call /trajectory_publisher/initialize_joint_pose \
"q: [0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397]
duration: 6.0"
```

Angles are ordered `fr3_joint1` through `fr3_joint7`. Edit the seven values to
select another posture without restarting Gazebo. This call is synchronous:
wait for `success: true` before continuing. It does not start recording. On
success, the Cartesian target is reset to the measured EE pose and CBF remains
off until enabled by a new experiment request. Reinitializing to the same `q`
provides a more repeatable joint starting configuration than reversing an XYZ
increment. If the duration is too short, use at least the minimum reported by
the service.

On failure, inspect the message and controller states, correct the cause and
retry initialization. Cartesian experiment commands can remain blocked until a
successful retry. An all-zero `applied_pose` in a failure response is not a
measured robot pose.

```bash
rosservice call /controller_manager/list_controllers "{}"
rosservice call /controller_manager/list_controller_types "{}"
```

The type list must include `effort_controllers/JointTrajectoryController`.
If missing, install `ros-noetic-joint-trajectory-controller` and restart Gazebo.

Initialization uses these private parameters of `trajectory_publisher`, read at
node startup. They are not exposed as launch arguments in the current launch:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `initialization_joint_margin` | `0.02` rad | Minimum target distance from position bounds. |
| `initialization_velocity_scale` | `0.2` | Fraction of the URDF velocity limit for the reference. |
| `initialization_velocity_cap` | `0.5` rad/s | Maximum reference velocity. |
| `initialization_position_tolerance` | `0.01` rad | Required final position accuracy for every joint. |
| `initialization_velocity_tolerance` | `0.02` rad/s | Stationary/settled speed threshold for every joint. |
| `initialization_settle_time` | `0.2` s | Consecutive simulated time within the tolerances. |
| `initialization_wall_timeout` | `120` s | Wall-clock timeout for the trajectory and, separately, final settling. |

If measured stationary velocity chatter slightly exceeds `0.02 rad/s`, a tested
adjustment is `0.03 rad/s`. Add this **inside the `trajectory_publisher` node**
of the launch, install the updated launch and restart it:

```xml
<param name="initialization_velocity_tolerance" type="double" value="0.03"/>
```

Do not interpret a visually stationary robot as proof that both tolerances are
met. Read measured `q` and `dq` before relaxing thresholds. Changing these
startup parameters with `rosparam set` after startup does not update the values
already held by the node.

The checked-in launch still spawns the standard joint configuration, and
`linear` starts from a fixed Cartesian reference. Merely changing spawn `-J`
values does not synchronize that reference. Use the initialization service to
set a different posture and synchronize the target.

#### Pausing and recording

If launched with `paused:=true`, unpause before initialization or recording:

```bash
rosservice call /gazebo/unpause_physics "{}"
```

To run without experiment files, replace `record_cbf:=true` with
`record_cbf:=false` in the launch. `log_dir` is then unused. This does not disable
live energy topics or ordinary ROS logs.

With recording enabled, the first `set_experiment_command` call creates the
session before forwarding the command. A recording-start failure rejects that
experiment request. Later experiment calls reuse the same session. To record
stationary history before the first movement, start manually and wait at least
one second before sending the experiment:

```bash
rosservice call /directional_cbf_recorder/start "{}"
```

The response contains the session path. There is no recorder pause/stop service.
Setting `cbf_active: false` keeps core data recording active. End the launch with
**Ctrl+C** to flush and close the files; relaunch for a new session.

## Test in Real robot

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

### Before the first physical experiment

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

### Start and initialize

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

### Run each experiment and return to the same joint pose

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

### Recorded data

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

## Plot data

### Recorded files and fields

The directional recorder creates:

```text
<log_dir>/experiment_<UTC timestamp>_<ID>/
    cbf.csv
    contacts.csv
    metadata.json
    robot.urdf
    health.json
```

The root directory is created automatically. With no `log_dir` argument, it is
`$HOME/.ros/directional_cbf`. With `log_dir:="$(pwd)/data/cbf"`, the path is based
on the launching terminal's current directory. No rosbag is created by this
recorder. `contacts.csv` is created when needed for CBF-active data.

| Data | When saved |
| --- | --- |
| Timestamp, time segment, sample index, experiment ID, controller period | Every received CBF sample after recording starts. |
| `cbf_h`, `cbf_constraint_safe`, `cbf_constraint_qp`, directional and total kinetic energy | With CBF on or off; unavailable values can be empty/NaN. |
| `Kmax`, `alpha`, direction, residual tolerance, Jacobian singular values | With CBF on or off. |
| EE/target positions, distance to requested target and filtered reference | With CBF on or off. |
| CBF enable/applied flags, solver status, task-abort flag, sample-gap count | With CBF on or off. |
| `q_1...q_7`, `dq_1...dq_7`, `tau_command_1...tau_command_7`, robot mode | Only with CBF active; check `debug_valid`. Torques are controller commands, not measured Gazebo applied torques. |
| Contact pairs, forces, penetration and timestamp association | CBF-active intervals, in `contacts.csv`. |

`metadata.json` describes the columns and joint mapping; `robot.urdf` stores the
model and limits; `health.json` reports queue drops, sample gaps, unmatched
contacts and shutdown status. During joint initialization, the stopped Cartesian
controller does not publish CBF samples, so this CSV is not a complete log of
the initialization motion. Experiment IDs can therefore skip values.

A `CBF sample-index gaps detected` warning means samples are missing: inspect
`missing_samples_before` and `health.json`. An empty contacts file is not proof
of absence of contact. Blank joint columns during CBF-off trials are expected.

### Run the Python script

The repository-root [`plot_directional_cbf.py`](plot_directional_cbf.py) runs
without ROS, `rosrun`, a Catkin workspace or a ROS environment. Copy it and the
recording to any machine with Python 3, NumPy and Matplotlib:

```bash
python3 -m pip install numpy matplotlib
```

From the folder containing the script, enter the full **`cbf.csv`** path when
prompted, without surrounding quotes:

```bash
read -r -p "Full path to cbf.csv: " CBF_CSV
python3 plot_directional_cbf.py "$CBF_CSV" --show
```

Do not pass `contacts.csv` as the main input: it lacks the required CBF columns.
Companion files are discovered beside `cbf.csv`. The CSV alone can be plotted;
without URDF/metadata, joint bounds may be unavailable, and without contacts,
contact plots report missing data rather than assuming zero contact.

The version currently checked into this branch saves separate diagnostic figures
and `--show` opens them. It includes CBF/target-distance panels, joint positions
with dashed URDF bounds and velocities, joint margins, Jacobian SVD and condition
ratio, energy/torque, derivative checks, parameters/timing, and contact plots.
The later motion-aligned comparison script distributed separately is not yet
part of this branch; options such as `--experiments`, `--pre-motion` and
`--no-show` are not supported by this checked-in version.

To select one experiment and save PDF figures:

```bash
read -r -p "Experiment ID from cbf.csv: " EXPERIMENT_ID
python3 plot_directional_cbf.py "$CBF_CSV" \
  --experiment "$EXPERIMENT_ID" \
  --output-dir "$(dirname "$CBF_CSV")/plots" \
  --format pdf \
  --show
```

| Option | Usage |
| --- | --- |
| Positional CSV path | Required `cbf.csv` input. |
| `--experiment ID` | Select one experiment; omit to include all. |
| `--segment ID` | Select a time segment, useful after clock/controller resets. |
| `--start SECONDS`, `--end SECONDS` | Restrict CSV `time_s`; not time relative to motion onset. |
| `--output-dir PATH` | Destination folder; default: `plots` beside the CSV. |
| `--format png\|pdf\|svg` | Output format; default: PNG. |
| `--show` | Open saved figures interactively; omit for file-only output. |
| `--urdf PATH` | Override the automatic `robot.urdf` lookup. |
| `--metadata PATH` | Override `metadata.json`. |
| `--contacts PATH` | Override `contacts.csv`. |
| `--arm-id fr3` | Supply the joint-name prefix if metadata is absent/ambiguous. |
| `--output PATH` | Compatibility option: use the filename as a prefix for figure files. |

```bash
python3 plot_directional_cbf.py --help
```

Check both the measured energy against `Kmax` and the CBF residual against its
numerical tolerance. A nonnegative model residual alone does not establish
that the measured energy stayed below threshold. Review tracking error, joint
limits, contact data and recording gaps alongside the energy plot.


## Directional CBF on the real robot

See [the hardware experiment guide](docs/directional_cbf_real.md) for the real
launch, joint reset, experiment services, CSV/rqt plots, actuator-aware QP,
Gazebo compatibility and validation requirements. The hardware launch starts in
measured-pose hold and requires explicit initialization before experiments.
