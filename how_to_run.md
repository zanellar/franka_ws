# How to Run

Workspace: `~/Riccardo/franka_ws_013` - ROS Noetic / Gazebo.
 
## Compile
 
```bash  
cd ~/Riccardo/franka_ws_013

source /opt/ros/noetic/setup.bash

export CC=/usr/bin/gcc-10
export CXX=/usr/bin/g++-10
export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

catkin_make install --force-cmake \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-10 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-10 \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_CXX_STANDARD_REQUIRED=ON \
  -DCMAKE_CXX_EXTENSIONS=OFF \
  -DABSL_PROPAGATE_CXX_STD=ON \
  -DABSL_BUILD_TESTING=OFF \
  -DOSQP-CPP_BUILD_TESTS=OFF \
  -DFranka_DIR="$FRANKA_013_PREFIX/lib/cmake/Franka" \
  -DCMAKE_PREFIX_PATH="$FRANKA_013_PREFIX;/opt/ros/noetic" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_ENABLE_TESTING=ON

cp -a devel/lib/libabsl*.so* install/lib/

source /opt/ros/noetic/setup.bash
source ~/Riccardo/franka_ws_013/install/setup.bash

export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"

export LD_LIBRARY_PATH="$HOME/Riccardo/franka_ws_013/install/lib:$FRANKA_013_PREFIX/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu"

grep -R -n -E '/opt/ros/noetic/include/franka/(robot|robot_state)\.h' build/franka_ros --include='*.o.d'

ldd install/lib/libfranka_hw.so | grep -E 'libfranka\.so|not found'
```

If needed, you can remove everything before (optional): 

```bash 
 rm -rf build devel install
```


## Terminal Setup

Run in every new terminal:

```bash
source "$HOME/Riccardo/franka_ws_013/env.sh"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/ros/noetic/lib/x86_64-linux-gnu"
```

Run only one Gazebo experiment at a time. Send movement commands individually;
wait for the robot to settle between commands.

## Gazebo Tests - Total Energy

**Terminal 1 - launch:**

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

**Terminal 2 - live energy:**

```bash
rqt_plot /cbf_info/kinetic_energy /cbf_info/Kmax
```

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

**Return, CBF off:**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: -0.20
y_move: 0.0
z_move: 0.0
cbf_active: false
Kmax: 20.0
alpha: 1.0"
```

**Forward, CBF on:**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: 0.20
y_move: 0.0
z_move: 0.0
cbf_active: true
Kmax: 0.01
alpha: 1.0"
```

## Gazebo Tests - Directional Energy

**Terminal 1 - launch, including joint-limit plots:**

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
Recording starts with the first `set_experiment_command` call. Core CBF data
are recorded from then on; additional debug data are recorded with CBF active.
Output directory: `~/Riccardo/franka_ws_013/data`.


**Terminal 2 - live energy:**

```bash
rqt_plot /cbf_info/kinetic_energy /cbf_info/directional_kinetic_energy /cbf_info/Kmax
```

**Terminal 3**


**Initial Configuration**

```bash
rosservice call /trajectory_publisher/initialize_joint_pose \
"q: [0.0, -0.2, 0.0, -2.2, 0.0, 3.2, 0.785398163397]
duration: 12.0"
```

**Forward, CBF off:**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: 0.1
y_move: 0.2
z_move: 0.0
cbf_active: false
Kmax: 0.02
alpha: 1.0"
```

**Initial Configuration**

```bash
rosservice call /trajectory_publisher/initialize_joint_pose \
"q: [0.0, -0.2, 0.0, -2.2, 0.0, 3.2, 0.785398163397]
duration: 12.0"
```

**Forward, CBF on, 1st alpha:**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: 0.1
y_move: 0.2
z_move: 0.0
cbf_active: true
Kmax: 0.02
alpha: 1.0"
```

**Initial Configuration**

```bash
rosservice call /trajectory_publisher/initialize_joint_pose \
"q: [0.0, -0.2, 0.0, -2.2, 0.0, 3.2, 0.785398163397]
duration: 12.0"
```

**Forward, CBF on, 2nd alpha::**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: 0.1
y_move: 0.2
z_move: 0.0
cbf_active: true
Kmax: 0.02
alpha: 5.0"
```

**Initial Configuration**

```bash
rosservice call /trajectory_publisher/initialize_joint_pose \
"q: [0.0, -0.2, 0.0, -2.2, 0.0, 3.2, 0.785398163397]
duration: 12.0"
```

**Forward, CBF on, 3rd alpha::**

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: 0.1
y_move: 0.2
z_move: 0.0
cbf_active: true
Kmax: 0.02
alpha: 10.0"
```

## Find Good Trajectory

Use 

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
  record_cbf:=false \
  joint_limit_diagnostics:=true \
  plot_joint_limits:=true \
  enable_joint_initialization:=true
```

 
### 1. Set the initial joint configuration

In the service terminal, edit `Q_INIT`: seven angles in radians, joints 1-7.
The example is the standard starting posture, not an optimized posture.
Wait until the robot is stationary before calling.

```bash
Q_INIT='[0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397]'
T_INIT=6.0

rosservice call /trajectory_publisher/initialize_joint_pose \
"q: $Q_INIT
duration: $T_INIT"
```

Wait for `success: true`. Initialization runs without CBF and does not start recording.
To change posture, edit `Q_INIT` and repeat the call. If duration is rejected,
increase `T_INIT` to at least the minimum reported in the response.

### 2. Try a small XYZ movement without CBF

Distances are in meters in the robot base frame. Edit the three values:

```bash
DX=0.02
DY=0.03
DZ=-0.01

rosservice call /trajectory_publisher/set_experiment_command \
"x_move: $DX
y_move: $DY
z_move: $DZ
cbf_active: false
Kmax: 0.05
alpha: 1.0"
```

Watch the margins for the entire motion. Adjust the initial posture or displacement
if a margin approaches zero. Repeated commands accumulate target displacements.

### 3. Reset to the same posture

Wait until stationary, then reuse the variables in the same terminal:

```bash
rosservice call /trajectory_publisher/initialize_joint_pose \
"q: $Q_INIT
duration: $T_INIT"
```

Wait for `success: true`. Use this reset before each comparison.

### 5. Repeat the same XYZ movement with CBF

```bash
rosservice call /trajectory_publisher/set_experiment_command \
"x_move: $DX
y_move: $DY
z_move: $DZ
cbf_active: true
Kmax: 0.05
alpha: 1.0"
```

`direction_x/y/z` in the launch selects the CBF energy direction independently
of `DX/DY/DZ`. Check joint margins again with CBF enabled.

### If initialization fails

Read the service error; inspect active controllers if needed:

```bash
rosservice call /controller_manager/list_controllers "{}"
```

Correct the reported condition and retry `initialize_joint_pose`.
Resume experiments only after `success: true`.
