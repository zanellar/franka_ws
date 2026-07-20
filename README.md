# Franka ROS 1 Workspace with Cartesian Impedance and CBF Controllers

This repository is a complete ROS 1 Catkin workspace based on `franka_ros` 0.10.1. It adds Cartesian-impedance controllers with total and directional kinetic-energy Control Barrier Functions (CBFs), OSQP-based torque filtering, trajectory publishers, diagnostics, RViz support, and experiment launch files.

The installation documented here was exercised with:

- Ubuntu 20.04.6 LTS;
- ROS Noetic;
- a PREEMPT_RT kernel;
- libfranka 0.13.3 built from source in an isolated prefix;
- a Franka Research 3 (FR3), Robot System 5.6.0;
- robot IP `172.16.0.2`;
- no Franka Hand, therefore `load_gripper:=false`.

> **Safety notice**
>
> These controllers command joint torques on physical hardware. Clear the robot workspace, keep the user-stop accessible, enable FCI only when ready, and begin with `trajectory:=hold` and `cbf_active:=false`. Repeated solver failures, communication errors, oscillation, or unexpected motion require stopping the experiment. A CBF or QP solver does not by itself make an experiment safe.

## Contents

- [Controllers](#controllers)
- [Installation step by step](#installation-step-by-step)
- [Runtime environment](#runtime-environment)
- [Robot and network preparation](#robot-and-network-preparation)
- [Run controllers](#run-controllers)
- [Command a Cartesian target manually](#command-a-cartesian-target-manually)
- [Topics and diagnostics](#topics-and-diagnostics)
- [Troubleshooting](#troubleshooting)
- [Development workflow](#development-workflow)

## Controllers

| Controller | Purpose |
|---|---|
| `cartesian_impedance_example_controller` | Standard Cartesian impedance controller driven by the RViz interactive marker. |
| `cartesian_impedance_cbf_controller` | Cartesian impedance with a total joint-space kinetic-energy CBF. |
| `cartesian_impedance_dead_zone_cbf_controller` | Total-energy CBF with a Cartesian dead zone. |
| `cartesian_impedance_cbf_interaction_controller` | Total-energy CBF with external-interaction handling. |
| `cartesian_impedance_dead_zone_cbf_interaction_controller` | Dead-zone and external-interaction variant. |
| `cartesian_impedance_cbf_interaction_power_controller` | Energy and interaction-power limiting. |
| `cartesian_impedance_directional_kinetic_energy_cbf_controller` | Limits translational kinetic energy along a selected base-frame direction. |

The directional controller constrains:

```text
K_dir = 0.5 * lambda_dir * v_dir^2
h     = Kmax - K_dir
```

where `v_dir` is the end-effector velocity projected along the selected direction and `lambda_dir` is the corresponding effective mass. The safe set is `h >= 0`.

---

## Installation step by step

### 0. Version rule

Use Ubuntu 20.04 with ROS Noetic for this branch. The workspace CMake files are pinned to **libfranka 0.13.3**.

`pylibfranka` is **not required** by this ROS 1 workspace. This branch uses the C++ library and ROS control interfaces from libfranka 0.13.3. Do not install a newer Python binding as a substitute for the required C++ library.

The commands below use this layout:

```text
~/Riccardo/
├── franka_ws_013/
└── libfranka-0.13.3/
    └── install/
```

### 1. Install ROS Noetic and base tools

Skip the ROS repository setup when ROS Noetic is already installed.

```bash
sudo apt update
sudo apt install -y curl ca-certificates gnupg2 lsb-release

sudo mkdir -p /usr/share/keyrings
sudo curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
  | sudo tee /etc/apt/sources.list.d/ros1-latest.list > /dev/null

sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  python3-rosdep \
  python3-catkin-tools \
  python3-vcstool \
  build-essential \
  cmake \
  git \
  pkg-config \
  libeigen3-dev \
  libpoco-dev \
  libfmt-dev \
  ethtool \
  rt-tests
```

Initialize `rosdep` once:

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

### 2. Clone this repository at branch `franka_ws_013`

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

### 3. Clone and build libfranka 0.13.3

```bash
mkdir -p ~/Riccardo

git clone \
  --recursive \
  --branch 0.13.3 \
  --single-branch \
  https://github.com/frankarobotics/libfranka.git \
  ~/Riccardo/libfranka-0.13.3

cd ~/Riccardo/libfranka-0.13.3
git submodule update --init --recursive
```

Configure, build, and install into an isolated local prefix:

```bash
cd ~/Riccardo/libfranka-0.13.3
rm -rf build install

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install" \
  -DBUILD_TESTS=OFF \
  -DBUILD_EXAMPLES=ON

cmake --build build -j"$(nproc)"
cmake --install build
```

Verify the library:

```bash
ls -l "$HOME/Riccardo/libfranka-0.13.3/install/lib/libfranka.so"*
```

The installed ABI should be `libfranka.so.0.13`.

### 4. Install ROS dependencies for the workspace

Use `--skip-keys libfranka` because this setup deliberately uses the custom 0.13.3 installation instead of the ROS Noetic binary package.

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

### 5. Build and install the Catkin workspace

```bash
cd ~/Riccardo/franka_ws_013
source /opt/ros/noetic/setup.bash

export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

rm -rf build devel install

catkin_make install \
  -DCMAKE_PREFIX_PATH="$FRANKA_013_PREFIX;/opt/ros/noetic" \
  -DCMAKE_BUILD_TYPE=Release
```

### 6. Copy the Abseil shared libraries

The vendored `osqp-cpp` build creates Abseil shared libraries in `devel/lib`, but Catkin does not reliably copy all of them into `install/lib`. Apply this workaround after every clean build:

```bash
cd ~/Riccardo/franka_ws_013
cp -a devel/lib/libabsl*.so* install/lib/
```

### 7. Verify the installed workspace

```bash
cd ~/Riccardo/franka_ws_013
source /opt/ros/noetic/setup.bash
source install/setup.bash

export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"
export LD_LIBRARY_PATH="$PWD/install/lib:$FRANKA_013_PREFIX/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"

rospack find franka_control
rospack find franka_example_controllers

ldd install/lib/franka_control/franka_control_node | grep "not found"
ldd install/lib/libfranka_example_controllers.so | grep "not found"

ldd install/lib/franka_control/franka_control_node | grep libfranka
ldd install/lib/libfranka_example_controllers.so | grep libfranka
```

Expected results:

- both `grep "not found"` commands print nothing;
- both Franka dependencies resolve to `~/Riccardo/libfranka-0.13.3/install/lib/libfranka.so.0.13`.

---

## Runtime environment

Run this block in every fresh terminal used for the workspace:

```bash
cd ~/Riccardo/franka_ws_013

source /opt/ros/noetic/setup.bash
source install/setup.bash

export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"
export LD_LIBRARY_PATH="$PWD/install/lib:$FRANKA_013_PREFIX/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
```

Do not source an older Franka workspace in the same terminal.

Create an optional helper script:

```bash
cat > ~/Riccardo/franka_ws_013/env.sh <<'EOF'
#!/usr/bin/env bash
cd "$HOME/Riccardo/franka_ws_013" || return 1
source /opt/ros/noetic/setup.bash
source install/setup.bash
export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"
export LD_LIBRARY_PATH="$PWD/install/lib:$FRANKA_013_PREFIX/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
EOF

chmod +x ~/Riccardo/franka_ws_013/env.sh
```

Then use:

```bash
source ~/Riccardo/franka_ws_013/env.sh
```

---

## Robot and network preparation

Before every hardware launch:

1. Connect the control computer to the robot network.
2. Configure the Ethernet interface in the same subnet as the robot.
3. Verify connectivity:

   ```bash
   ping -c 4 172.16.0.2
   ```

4. Open Franka Desk.
5. Unlock the joints.
6. Enable FCI.
7. Confirm that no other FCI client is connected.
8. Clear the physical workspace and keep the user-stop accessible.

For the tested FR3 without a Hand, always use:

```text
robot:=fr3
load_gripper:=false
```

### Real-time and communication checks

Confirm the real-time kernel:

```bash
uname -a
```

Run a latency test:

```bash
sudo cyclictest \
  --mlockall \
  --smp \
  --priority=80 \
  --interval=1000 \
  --duration=60s
```

The libfranka communication test moves the robot. Run it only with the workspace clear and the user-stop accessible:

```bash
cd ~/Riccardo/libfranka-0.13.3/build/examples
sudo chrt -f 80 ./communication_test 172.16.0.2
```

Low ping latency does not prove that the 1 kHz FCI connection is reliable. The tested `sinatra` host previously showed non-zero state loss, so this repository must be treated as a development setup until the communication success rate is consistently near 1.00.

---

## Run controllers

Enable FCI in Franka Desk before each launch. Stop the launch with `Ctrl+C` before starting another controller.

### Standard Cartesian impedance controller

This launch starts the RViz interactive marker. The robot moves when the `equilibrium_pose` marker is moved.

```bash
source ~/Riccardo/franka_ws_013/env.sh

roslaunch franka_example_controllers \
  cartesian_impedance_example_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=fr3 \
  alpha:=1.0 \
  Kmax:=20.0 \
  rosbag:=false
```

Begin with a displacement of only a few millimetres.

### Total kinetic-energy CBF controller, CBF disabled

Use this as the baseline test. In this branch, OSQP is bypassed when `cbf_active:=false`.

```bash
source ~/Riccardo/franka_ws_013/env.sh

roslaunch franka_example_controllers \
  cartesian_impedance_cbf_controller.launch \
  robot_ip:=172.16.0.2 \
  load_gripper:=false \
  robot:=fr3 \
  trajectory:=hold \
  Kmax:=20.0 \
  cbf_active:=false \
  alpha:=1.0 \
  rosbag:=false
```

Verify from another sourced terminal:

```bash
source ~/Riccardo/franka_ws_013/env.sh
rosparam get /cbf_active
rosservice call /controller_manager/list_controllers
```

### Total kinetic-energy CBF controller, CBF active

Start with a moderate energy limit:

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

Smaller values such as `Kmax:=0.1` or `Kmax:=0.01` produce a more restrictive energy limit. Repeated `OsqpExitCode::kTimeLimitReached` messages mean that the QP did not finish within the configured real-time limit for those cycles; do not treat such a run as a clean validation.

### Directional kinetic-energy CBF controller

This example constrains motion along positive base-frame x:

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

Direction examples:

```text
[ 1, 0, 0]  positive base-frame x
[-1, 0, 0]  negative base-frame x
[ 0, 1, 0]  positive base-frame y
[ 0, 0, 1]  positive base-frame z
```

The controller normalizes the vector internally. A zero vector is invalid.

### Available trajectory publishers

```bash
find ~/Riccardo/franka_ws_013/install/lib/franka_trajectory \
  -maxdepth 1 \
  -type f \
  -executable \
  -printf '%f\n'
```

Expected names include:

```text
hold
circular
chirp
square_wave
tension
```

Use `trajectory:=circular`, not `trajectory:=circle`.

---

## Command a Cartesian target manually

The CBF controllers subscribe to:

```text
/trajectory_publisher/equilibrium_pose
```

The `hold` node publishes continuously and overwrites manual commands. After the controller reaches the hold pose, stop only the trajectory publisher:

```bash
rosnode kill /trajectory_publisher
```

Check that the controller remains subscribed:

```bash
rostopic info /trajectory_publisher/equilibrium_pose
```

Publish an absolute target pose:

```bash
rostopic pub -1 \
  /trajectory_publisher/equilibrium_pose \
  geometry_msgs/PoseStamped \
  "{header: {stamp: now, frame_id: ''}, pose: {position: {x: 0.507, y: 0.0, z: 0.59}, orientation: {x: 0.9238795, y: -0.3826834, z: 0.0, w: 0.0}}}"
```

This is an absolute pose in the controller reference frame, not a relative `+0.20 m` command.

---

## Topics and diagnostics

```bash
rostopic list | grep -Ei 'cbf|equilibrium|joint|state'
rostopic echo /cbf_info
rqt_plot /cbf_info/h
rosservice call /controller_manager/list_controllers
```

The custom `franka_msgs/Cbf` message contains nominal, CBF-filtered, measured, and final torques, together with the barrier value `h` and solver status.

For a valid CBF experiment:

- the intended controller must be `running`;
- solver failures must be absent or explicitly handled;
- `h` should remain non-negative within an understood numerical tolerance;
- torque, rate, communication, and collision limits must not be violated.

---

## Troubleshooting

### Missing Abseil libraries

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

The final command should print nothing.

### Wrong libfranka version or incompatible protocol

Typical symptom:

```text
libfranka: Incompatible library version
```

Check the linked library:

```bash
source ~/Riccardo/franka_ws_013/env.sh

ldd install/lib/franka_control/franka_control_node | grep libfranka
ldd install/lib/libfranka_example_controllers.so | grep libfranka
```

Both must resolve to the local `libfranka.so.0.13`. If an older library under `/opt/ros/noetic` appears, open a fresh terminal, source only this workspace, preserve the runtime path exactly as shown above, and rebuild with the explicit `CMAKE_PREFIX_PATH`.

### `roslaunch` cannot find a package

```bash
source /opt/ros/noetic/setup.bash
source ~/Riccardo/franka_ws_013/install/setup.bash
rospack find franka_example_controllers
```

### `Connection to FCI refused`

Enable FCI in Franka Desk and make sure another process is not connected to the robot.

### FR3 model or joint errors

Use:

```text
robot:=fr3
```

The expected effort resources are `fr3_joint1` through `fr3_joint7`.

### Gripper library links to an older libfranka

This setup has no Franka Hand. Use:

```text
load_gripper:=false
```

Do not diagnose the unused `/opt/ros/noetic/lib/libfranka_gripper.so` as the controller failure unless the gripper node is actually being loaded.

### `interactive_marker.py` is missing

```bash
cd ~/Riccardo/franka_ws_013
chmod +x src/franka_ros/franka_example_controllers/scripts/interactive_marker.py
source /opt/ros/noetic/setup.bash
export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

catkin_make install \
  -DCMAKE_PREFIX_PATH="$FRANKA_013_PREFIX;/opt/ros/noetic" \
  -DCMAKE_BUILD_TYPE=Release

cp -a devel/lib/libabsl*.so* install/lib/
ls -l install/lib/franka_example_controllers/interactive_marker.py
```

### QP time-limit warnings

```text
QPsolver did not find optimal solution, exit code OsqpExitCode::kTimeLimitReached
```

Possible causes include an excessively small `Kmax`, an infeasible CBF constraint under torque/rate limits, noisy derivatives, poor directional mobility, or excessive solver work inside the 1 ms control period.

Diagnostic sequence:

1. stop robot motion;
2. restart with `trajectory:=hold`;
3. use `cbf_active:=false` to verify the nominal controller;
4. restart with a moderate `Kmax` and `alpha:=1.0`;
5. inspect `/cbf_info` and the controller terminal;
6. stop the test if timeouts repeat.

### `rosparam set /Kmax ...` does not change controller behavior

Some parameters are read during controller initialization. Updating the ROS parameter server does not automatically change a cached C++ variable. Restart the controller with the new launch argument unless the parameter has been added to the controller's `dynamic_reconfigure` callback and the workspace has been rebuilt.

### Old workspace contaminates the environment

Open a new terminal and run only:

```bash
source ~/Riccardo/franka_ws_013/env.sh
```

Check:

```bash
echo "$ROS_PACKAGE_PATH" | tr ':' '\n'
```

Remove references to older Franka workspaces from `~/.bashrc`.

### Clean rebuild

```bash
cd ~/Riccardo/franka_ws_013
source /opt/ros/noetic/setup.bash
export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

rm -rf build devel install

catkin_make install \
  -DCMAKE_PREFIX_PATH="$FRANKA_013_PREFIX;/opt/ros/noetic" \
  -DCMAKE_BUILD_TYPE=Release

cp -a devel/lib/libabsl*.so* install/lib/
```

### Rosbag output path

Several launch files contain a historical absolute output path. Keep recording disabled during initial testing:

```text
rosbag:=false
```

Find hard-coded paths with:

```bash
grep -R "/home/dlogmans" -n \
  ~/Riccardo/franka_ws_013/src/franka_ros/franka_example_controllers/launch
```

Create a local directory before enabling recording:

```bash
mkdir -p ~/Riccardo/Rundata
```

Then update the relevant launch-file path.

---

## Development workflow

After changing controller sources, messages, dynamic-reconfigure files, or launch files:

```bash
cd ~/Riccardo/franka_ws_013
source /opt/ros/noetic/setup.bash
export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

catkin_make install \
  -DCMAKE_PREFIX_PATH="$FRANKA_013_PREFIX;/opt/ros/noetic" \
  -DCMAKE_BUILD_TYPE=Release

cp -a devel/lib/libabsl*.so* install/lib/
source ~/Riccardo/franka_ws_013/env.sh
```

Verify the plugin and installed library:

```bash
rospack plugins --attrib=plugin controller_interface \
  | grep franka_example_controllers

ls -l ~/Riccardo/franka_ws_013/install/lib/libfranka_example_controllers.so
```
