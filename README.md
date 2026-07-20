# Franka ROS 1 Workspace with Cartesian Impedance and CBF Controllers

This repository is a ROS 1 Catkin workspace based on `franka_ros` and extended with Cartesian-impedance controllers, total and directional kinetic-energy Control Barrier Functions (CBFs), OSQP-based torque filtering, trajectory publishers, diagnostics, RViz support, and Gazebo integration.

The procedure below has been validated for the `franka_ws_013` branch with:

- Ubuntu 20.04
- ROS Noetic
- GCC/G++ 10
- C++17
- libfranka 0.13.3 installed in an isolated prefix
- Gazebo 11
- SDFormat 9
- Franka Research 3 (FR3)
- no Franka Hand (`load_gripper:=false`)

> **Safety notice**
>
> These controllers command joint torques. On physical hardware, clear the robot workspace, keep the user-stop accessible, enable FCI only when ready, and begin with a hold trajectory and `cbf_active:=false`. Stop immediately in case of oscillation, repeated solver failure, communication errors, or unexpected motion. A CBF or QP solver does not by itself make an experiment safe.

---

## Contents

- [Repository layout](#repository-layout)
- [Controllers](#controllers)
- [Installation](#installation)
- [Runtime environment](#runtime-environment)
- [Build verification](#build-verification)
- [Gazebo test](#gazebo-test)
- [Linear trajectory command service](#linear-trajectory-command-service)
- [Hardware preparation](#hardware-preparation)
- [Run controllers](#run-controllers)
- [Manual equilibrium pose](#manual-equilibrium-pose)
- [Diagnostics](#diagnostics)
- [Troubleshooting](#troubleshooting)
- [Development workflow](#development-workflow)

---

## Repository layout

The documented installation uses:

```text
~/Riccardo/
├── franka_ws_013/
└── libfranka-0.13.3/
    └── install/
```

The workspace is expected at:

```text
~/Riccardo/franka_ws_013
```

The custom libfranka installation is expected at:

```text
~/Riccardo/libfranka-0.13.3/install
```

---

## Controllers

| Controller | Purpose |
|---|---|
| `cartesian_impedance_example_controller` | Standard Cartesian impedance controller. |
| `cartesian_impedance_cbf_controller` | Cartesian impedance with a total joint-space kinetic-energy CBF. |
| `cartesian_impedance_dead_zone_cbf_controller` | Total-energy CBF with a Cartesian dead zone. |
| `cartesian_impedance_cbf_interaction_controller` | Total-energy CBF with interaction handling. |
| `cartesian_impedance_dead_zone_cbf_interaction_controller` | Dead-zone and interaction variant. |
| `cartesian_impedance_cbf_interaction_power_controller` | Energy and interaction-power limiting. |
| `cartesian_impedance_directional_kinetic_energy_cbf_controller` | Limits translational kinetic energy along a selected base-frame direction. |

For the directional controller:

```text
K_dir = 0.5 * lambda_dir * v_dir^2
h     = Kmax - K_dir
```

The safe set is:

```text
h >= 0
```

---

# Installation

## 1. Install ROS Noetic and build tools

```bash
sudo apt update

sudo apt install -y \
  ros-noetic-desktop-full \
  python3-rosdep \
  python3-catkin-tools \
  python3-vcstool \
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

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi

rosdep update
```

Source ROS:

```bash
source /opt/ros/noetic/setup.bash
```

## 2. Clone the workspace

```bash
mkdir -p ~/Riccardo

git clone \
  --branch franka_ws_013 \
  --single-branch \
  https://github.com/zanellar/franka_ws.git \
  ~/Riccardo/franka_ws_013

cd ~/Riccardo/franka_ws_013
git branch --show-current
```

Expected:

```text
franka_ws_013
```

## 3. Build libfranka 0.13.3

```bash
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

```bash
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

```bash
ls -l "$HOME/Riccardo/libfranka-0.13.3/install/lib/libfranka.so"*
```

The installed ABI must include:

```text
libfranka.so.0.13
```

## 4. Install ROS package dependencies

The custom libfranka installation replaces the ROS binary `libfranka` package for this workspace.

```bash
cd ~/Riccardo/franka_ws_013
source /opt/ros/noetic/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro noetic \
  --skip-keys libfranka \
  -y
```

## 5. Required CMake configuration

This branch must compile the controller stack and vendored dependencies as C++17.

In `src/franka_ros/franka_example_controllers/CMakeLists.txt`, ensure:

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Franka 0.13.3 REQUIRED)

set(OSQP-CPP_BUILD_TESTS OFF CACHE BOOL
    "Disable embedded osqp-cpp tests" FORCE)

set(ABSL_BUILD_TESTING OFF CACHE BOOL
    "Disable embedded Abseil tests" FORCE)
```

In `src/franka_ros/franka_example_controllers/lib/osqp-cpp/CMakeLists.txt`, ensure:

```cmake
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ language standard" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON CACHE BOOL "Require selected C++ standard" FORCE)
set(CMAKE_CXX_EXTENSIONS OFF CACHE BOOL "Disable compiler-specific extensions" FORCE)

set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL
    "Propagate the selected C++ standard to Abseil targets" FORCE)
```

Pin Abseil instead of fetching `origin/master`:

```cmake
FetchContent_Declare(
  abseil-cpp
  GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
  GIT_TAG        20240116.2
  GIT_SHALLOW    TRUE
)
```

In `src/franka_ros/franka_gazebo/CMakeLists.txt`, ensure:

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(sdformat9 REQUIRED CONFIG)
```

If Gazebo exports stale SDFormat 9.8 paths, remove them before targets are created:

```cmake
list(REMOVE_ITEM catkin_INCLUDE_DIRS
  "/usr/include/sdformat-9.8"
  "/usr/include/sdformat-9.8/sdf/.."
)

list(REMOVE_ITEM catkin_LIBRARIES
  "/usr/lib/x86_64-linux-gnu/libsdformat9.so.9.8.0"
)
```

Link SDFormat through its imported target:

```cmake
target_link_libraries(franka_hw_sim
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  sdformat9::sdformat9
)
```

## 6. Clean old dependency caches

```bash
cd ~/Riccardo/franka_ws_013

rm -rf \
  src/franka_ros/franka_example_controllers/lib/osqp-cpp/build \
  build \
  devel \
  install
```

Optional cleanup of editor backup files:

```bash
find src -name '*~' -delete
```

## 7. Build and install the Catkin workspace

```bash
cd ~/Riccardo/franka_ws_013

source /opt/ros/noetic/setup.bash

export CC=/usr/bin/gcc-10
export CXX=/usr/bin/g++-10
export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

catkin_make install \
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

```text
Performing Test ABSL_INTERNAL_AT_LEAST_CXX17 - Success
```

## 8. Copy Abseil shared libraries

```bash
cd ~/Riccardo/franka_ws_013
cp -a devel/lib/libabsl*.so* install/lib/
```

Run this after every clean build.

---

# Runtime environment

Use this block in every fresh terminal:

```bash
cd ~/Riccardo/franka_ws_013

source /opt/ros/noetic/setup.bash
source install/setup.bash

export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"
export LD_LIBRARY_PATH="$PWD/install/lib:$FRANKA_013_PREFIX/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
```

Create a helper script:

```bash
cat > ~/Riccardo/franka_ws_013/env.sh <<'EOS'
#!/usr/bin/env bash

cd "$HOME/Riccardo/franka_ws_013" || return 1

source /opt/ros/noetic/setup.bash
source install/setup.bash

export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"
export LD_LIBRARY_PATH="$PWD/install/lib:$FRANKA_013_PREFIX/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
EOS

chmod +x ~/Riccardo/franka_ws_013/env.sh
```

Use:

```bash
source ~/Riccardo/franka_ws_013/env.sh
```

Do not source an older Franka workspace in the same terminal.

---

# Build verification

```bash
source ~/Riccardo/franka_ws_013/env.sh

rospack find franka_control
rospack find franka_example_controllers
rospack find franka_gazebo
```

Check unresolved libraries:

```bash
ldd install/lib/franka_control/franka_control_node | grep "not found"
ldd install/lib/libfranka_example_controllers.so | grep "not found"
ldd install/lib/libfranka_hw_sim.so | grep "not found"
```

Expected: no output.

Verify libfranka:

```bash
ldd install/lib/franka_control/franka_control_node | grep libfranka
ldd install/lib/libfranka_example_controllers.so | grep libfranka
ldd install/lib/libfranka_hw_sim.so | grep libfranka
```

Expected ABI:

```text
libfranka.so.0.13
```

Verify SDFormat:

```bash
ldd install/lib/libfranka_hw_sim.so | grep sdformat
```

Expected ABI:

```text
libsdformat9.so.9
```

Check that stale paths are absent:

```bash
grep -RIn \
  -E 'sdformat-9\.8|libsdformat9\.so\.9\.8\.0' \
  build/franka_ros/franka_gazebo \
  2>/dev/null
```

Expected: no output.

---

# Gazebo test

## Validate the generated URDF

```bash
xacro \
  ~/Riccardo/franka_ws_013/src/franka_ros/franka_description/robots/fr3/fr3.urdf.xacro \
  gazebo:=true \
  hand:=false \
  arm_id:=fr3 \
  > /tmp/fr3_gazebo.urdf
```

```bash
check_urdf /tmp/fr3_gazebo.urdf
```

Verify the Gazebo ROS control plugin:

```bash
grep -A8 -B2 "gazebo_ros_control" /tmp/fr3_gazebo.urdf
```

Verify that no gripper joints are present:

```bash
grep -E "finger_joint|hand_joint|franka_hand" /tmp/fr3_gazebo.urdf
```

Expected: no output.

## Start the controller with the CBF disabled

```bash
source ~/Riccardo/franka_ws_013/env.sh

roslaunch franka_example_controllers \
  cartesian_impedance_cbf_controller_gazebo.launch \
  cbf_active:=false \
  start_trajectory:=false \
  headless:=false \
  rviz:=false
```

Verify:

```bash
rosservice call /controller_manager/list_controllers
rostopic hz /franka_state_controller/franka_states
rostopic hz /franka_state_controller/joint_states
```

---


# Linear trajectory command service

The workspace includes a `linear` trajectory publisher for repeatable Gazebo and hardware experiments.

Unlike `hold`, which continuously publishes one fixed equilibrium pose, `linear` stores the current target pose and allows it to be updated while the controller is running. The update is sent through:

```text
/trajectory_publisher/set_experiment_command
```

The service request contains:

```text
float64 x_move
bool cbf_active
float64 Kmax
float64 alpha
```

The service performs one coordinated experiment command:

1. update `cbf_active`, `Kmax`, and `alpha` through the controller's dynamic-reconfigure server;
2. add `x_move` to the currently stored Cartesian x target;
3. keep publishing the resulting equilibrium pose until the next service request.

The displacement is relative and cumulative:

```text
new_target_x = current_target_x + x_move
```

For example, starting from `x = 0.307`:

```text
x_move =  0.20  -> x = 0.507
x_move = -0.20  -> x = 0.307
```

The service response reports whether the update succeeded and returns the applied pose.

## Gazebo test with the linear trajectory

Start Gazebo with the `linear` trajectory enabled:

```bash
source ~/Riccardo/franka_ws_013/env.sh

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

Leave the launch terminal running.

In a second terminal:

```bash
source ~/Riccardo/franka_ws_013/env.sh
```

Verify that the trajectory service and the controller dynamic-reconfigure service are available:

```bash
rosservice list | grep -E \
  'set_experiment_command|dynamic_reconfigure_compliance_param_node/set_parameters'
```

Expected services include:

```text
/trajectory_publisher/set_experiment_command
/cartesian_impedance_cbf_controller/dynamic_reconfigure_compliance_param_node/set_parameters
```

Run the following three test commands in sequence.

### Test 1: move forward with the CBF disabled

```bash
rosservice call \
  /trajectory_publisher/set_experiment_command \
  "x_move: 0.20
cbf_active: false
Kmax: 20.0
alpha: 1.0"
```

Expected applied x target:

```text
x = 0.507
```

### Test 2: move back to the initial target with the CBF disabled

```bash
rosservice call \
  /trajectory_publisher/set_experiment_command \
  "x_move: -0.20
cbf_active: false
Kmax: 20.0
alpha: 1.0"
```

Expected applied x target:

```text
x = 0.307
```

### Test 3: move forward with the CBF enabled

```bash
rosservice call \
  /trajectory_publisher/set_experiment_command \
  "x_move: 0.20
cbf_active: true
Kmax: 0.01
alpha: 1.0"
```

Expected applied x target:

```text
x = 0.507
```

The expected service response is:

```text
success: True
message: "Pose and CBF parameters updated."
```

Verify the pose being republished:

```bash
rostopic echo -n 1 /trajectory_publisher/equilibrium_pose
```

Verify the active CBF parameters:

```bash
rosrun dynamic_reconfigure dynparam get \
  /cartesian_impedance_cbf_controller/dynamic_reconfigure_compliance_param_node
```

The final test should report values equivalent to:

```yaml
cbf_active: true
Kmax: 0.01
alpha: 1.0
```

## Command semantics and precautions

`x_move` is an increment, not an absolute x coordinate. Repeating the same command repeatedly continues to move the target:

```text
0.307 -> 0.507 -> 0.707
```

Use small increments during initial hardware testing, for example:

```bash
rosservice call \
  /trajectory_publisher/set_experiment_command \
  "x_move: 0.01
cbf_active: false
Kmax: 20.0
alpha: 1.0"
```

The combined service requires the Cartesian CBF controller to be running. If the controller dynamic-reconfigure service is unavailable, the request fails with:

```text
Failed to update the controller dynamic-reconfigure service.
```

For physical-robot tests, define and enforce suitable Cartesian workspace limits inside `linear.cpp` before using large cumulative offsets.


# Hardware preparation

Before every physical-robot launch:

1. Connect the control computer directly to the robot network.
2. Configure the Ethernet interface in the robot subnet.
3. Verify connectivity with `ping -c 4 172.16.0.2`.
4. Open Franka Desk.
5. Unlock the joints.
6. Enable FCI.
7. Confirm no other FCI client is connected.
8. Clear the workspace and keep the user-stop accessible.

For an FR3 without a Hand:

```text
robot:=fr3
load_gripper:=false
```

Optional latency test:

```bash
sudo cyclictest \
  --mlockall \
  --smp \
  --priority=80 \
  --interval=1000 \
  --duration=60s
```

---

# Run controllers

## Baseline: CBF disabled

```bash
source ~/Riccardo/franka_ws_013/env.sh

roslaunch franka_example_controllers \
  cartesian_impedance_cbf_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=fr3 \
  trajectory:=hold \
  cbf_active:=false \
  alpha:=1.0 \
  rosbag:=false
```

## Total kinetic-energy CBF enabled

```bash
source ~/Riccardo/franka_ws_013/env.sh

roslaunch franka_example_controllers \
  cartesian_impedance_cbf_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=fr3 \
  trajectory:=hold \
  Kmax:=1.0 \
  cbf_active:=true \
  alpha:=1.0 \
  rosbag:=false
```

## Directional kinetic-energy CBF

```bash
source ~/Riccardo/franka_ws_013/env.sh

roslaunch franka_example_controllers \
  cartesian_impedance_directional_kinetic_energy_cbf_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=fr3 \
  trajectory:=hold \
  Kmax:=0.1 \
  alpha:=1.0 \
  direction_x:=1.0 \
  direction_y:=0.0 \
  direction_z:=0.0 \
  cbf_active:=true \
  rosbag:=false
```

The direction vector is normalized internally. A zero vector is invalid.

---

# Manual equilibrium pose

The CBF controllers subscribe to:

```text
/trajectory_publisher/equilibrium_pose
```

The `hold` publisher continuously overwrites manual targets. Stop only that publisher:

```bash
rosnode kill /trajectory_publisher
```

Confirm the controller remains subscribed:

```bash
rostopic info /trajectory_publisher/equilibrium_pose
```

Inspect the current transform:

```bash
rosrun tf tf_echo fr3_link0 fr3_EE
```

Publish a one-shot target:

```bash
rostopic pub -1 \
  /trajectory_publisher/equilibrium_pose \
  geometry_msgs/PoseStamped \
  "{header: {stamp: now, frame_id: 'fr3_link0'},
    pose: {
      position: {x: 0.40, y: 0.00, z: 0.50},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }}"
```

For the first test, preserve the current quaternion and modify one position coordinate by approximately 0.01 m.

Publish continuously at 10 Hz:

```bash
rostopic pub -r 10 \
  /trajectory_publisher/equilibrium_pose \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'fr3_link0'},
    pose: {
      position: {x: 0.40, y: 0.00, z: 0.50},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }}"
```

---

# Diagnostics

```bash
rostopic list | grep -Ei 'cbf|equilibrium|joint|state'
rostopic echo /cbf_info
rqt_plot /cbf_info/h
rosservice call /controller_manager/list_controllers
```

For a valid CBF experiment:

- the intended controller is `running`;
- repeated solver failures are absent;
- `h` remains non-negative within an understood numerical tolerance;
- torque and torque-rate limits are respected;
- communication and collision limits are not violated.

---

# Troubleshooting

## Abseil reports C++ < 17

Typical error:

```text
The compiler defaults to or is configured for C++ < 17
```

Check active CMake assignments:

```bash
grep -RIn "CMAKE_CXX_STANDARD" \
  src/franka_ros/franka_example_controllers \
  --exclude='*.md' \
  --exclude='*.sh' \
  --exclude='*.bat'
```

Active source files must use C++17.

Then remove stale build state:

```bash
rm -rf \
  src/franka_ros/franka_example_controllers/lib/osqp-cpp/build \
  build \
  devel \
  install
```

Rebuild with the complete C++17 command from the installation section.

## Missing Abseil libraries

Typical error:

```text
libabsl_*.so: cannot open shared object file
```

Fix:

```bash
cd ~/Riccardo/franka_ws_013
cp -a devel/lib/libabsl*.so* install/lib/
source ~/Riccardo/franka_ws_013/env.sh

ldd install/lib/libfranka_example_controllers.so | grep "not found"
```

Expected: no output.

## Stale SDFormat 9.8 paths

Typical errors:

```text
No rule to make target '/usr/lib/x86_64-linux-gnu/libsdformat9.so.9.8.0'
fatal error: sdf/sdf.hh: No such file or directory
```

Inspect installed SDFormat:

```bash
pkg-config --modversion sdformat9
pkg-config --cflags sdformat9
pkg-config --libs sdformat9
```

Inspect CMake metadata:

```bash
grep -RIn 'libsdformat9' \
  /usr/lib/x86_64-linux-gnu/cmake/gazebo \
  /usr/lib/x86_64-linux-gnu/cmake/sdformat9 \
  2>/dev/null
```

If system metadata points to the installed version but the workspace build refers to 9.8, remove `build`, `devel`, and `install`, then rebuild.

If Gazebo injects stale values, apply the `list(REMOVE_ITEM ...)` and `sdformat9::sdformat9` fix documented above.

Do not create fake compatibility symlinks for `libsdformat9.so.9.8.0`.

## Wrong libfranka version

Typical symptom:

```text
libfranka: Incompatible library version
```

Check:

```bash
source ~/Riccardo/franka_ws_013/env.sh

ldd install/lib/franka_control/franka_control_node | grep libfranka
ldd install/lib/libfranka_example_controllers.so | grep libfranka
ldd install/lib/libfranka_hw_sim.so | grep libfranka
```

All relevant binaries must resolve to the isolated 0.13.3 installation.

## `roslaunch` cannot find a package

```bash
source /opt/ros/noetic/setup.bash
source ~/Riccardo/franka_ws_013/install/setup.bash
rospack find franka_example_controllers
```

## `Connection to FCI refused`

Enable FCI in Franka Desk and verify that another process is not already connected.

## FR3 resource or joint errors

Use:

```text
robot:=fr3
```

Expected effort resources are `fr3_joint1` through `fr3_joint7`.

## No Franka Hand

Use:

```text
load_gripper:=false
```

## Hold publisher overwrites a manual target

```bash
rosnode kill /trajectory_publisher
```

Do not stop the controller manager or the Cartesian controller.

## QP time-limit warnings

Typical message:

```text
QPsolver did not find optimal solution, exit code OsqpExitCode::kTimeLimitReached
```

Possible causes include an excessively small `Kmax`, infeasible CBF constraints, torque or torque-rate saturation, noisy derivatives, poor directional mobility, or excessive solver work within the 1 ms control period.

Diagnostic sequence:

1. stop motion;
2. restart with `trajectory:=hold`;
3. test `cbf_active:=false`;
4. enable the CBF with a moderate `Kmax`;
5. inspect `/cbf_info`;
6. stop if timeouts repeat.

## `rosparam set /Kmax ...` has no effect

Some parameters are read only during controller initialization. Restart the controller with a new launch argument unless the parameter is connected to a `dynamic_reconfigure` callback.

## Old workspace contaminates the environment

```bash
source ~/Riccardo/franka_ws_013/env.sh

echo "$ROS_PACKAGE_PATH" | tr ':' '\n'
echo "$LD_LIBRARY_PATH" | tr ':' '\n'
```

Remove old Franka workspace sourcing from `~/.bashrc`.

## Clean rebuild

```bash
cd ~/Riccardo/franka_ws_013
source /opt/ros/noetic/setup.bash

export CC=/usr/bin/gcc-10
export CXX=/usr/bin/g++-10
export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

rm -rf \
  src/franka_ros/franka_example_controllers/lib/osqp-cpp/build \
  build \
  devel \
  install

catkin_make install \
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

cp -a devel/lib/libabsl*.so* install/lib/
```

## Rosbag output path

Keep recording disabled during initial tests:

```text
rosbag:=false
```

Find hard-coded paths:

```bash
grep -R "/home/" -n \
  ~/Riccardo/franka_ws_013/src/franka_ros/franka_example_controllers/launch
```

Create a local output directory before enabling recording:

```bash
mkdir -p ~/Riccardo/Rundata
```

---

# Development workflow

After changing sources, messages, dynamic-reconfigure files, launch files, or dependency CMake files:

```bash
cd ~/Riccardo/franka_ws_013
source /opt/ros/noetic/setup.bash

export CC=/usr/bin/gcc-10
export CXX=/usr/bin/g++-10
export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

catkin_make install \
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

cp -a devel/lib/libabsl*.so* install/lib/
source ~/Riccardo/franka_ws_013/env.sh
```

Verify plugin registration:

```bash
rospack plugins --attrib=plugin controller_interface \
  | grep franka_example_controllers
```

Verify installed libraries:

```bash
ls -l install/lib/libfranka_example_controllers.so
ls -l install/lib/libfranka_hw_sim.so
```