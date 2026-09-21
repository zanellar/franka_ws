# Directional CBF data recording and plotting

Base: branch franka_ws_013_gazebo, commit e321d9f185252c774786fe892af35134f26fd178.
Apply this patch after the five controller/Gazebo fixes already on this branch.
The local workspace can be named franka_ws_013; patch paths are relative to its root.

## Apply and rebuild the install space

Place directional_cbf_logging.patch in the root of franka_ws_013 and run:

```bash
git apply --check directional_cbf_logging.patch
git apply directional_cbf_logging.patch

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
source install/setup.bash
```

This builds the new franka_msgs/DirectionalCbfDiagnostics message and installs
both Python scripts. The existing Cbf message is unchanged. Restart Gazebo and
the controller after rebuilding. Plotting needs Python 3, NumPy and Matplotlib;
recording needs the sourced ROS Noetic environment and rospy, but not Matplotlib.

## Record

Run from the actual franka_ws_013 root. Using pwd avoids assuming its parent directory:

```bash
roslaunch franka_example_controllers \
  cartesian_impedance_directional_kinetic_energy_cbf_controller_gazebo.launch \
  start_trajectory:=true trajectory:=linear \
  cbf_active:=true Kmax:=0.1 alpha:=5.0 \
  record_cbf:=true log_dir:="$(pwd)/data/cbf"
```

The recorder prints the exact output filename, for example:

```
franka_ws_013/data/cbf/directional_cbf_20260918T120000_000000Z_a1b2c3d4.csv
```

A new unique CSV is created per recorder session; files are never overwritten.
Recording starts with incoming diagnostics and continues through CBF bypass,
errors and retries. The same CSV contains all attempts. experiment_id starts
at 0 and increments when the controller consumes an explicit experiment start.
The public set_experiment_command service works as before.

Stop normally with Ctrl+C to flush and close the file. Buffers are also flushed
once per wall-clock second; process kill or a machine failure can lose buffered
rows. A header-only CSV means no diagnostic message was received.

Without an explicit log_dir, the default is ~/.ros/directional_cbf.
Use record_cbf:=false to disable the automatic recorder. A separate recorder can
also be started manually (avoid doing both unless you want two CSV copies):

```bash
rosrun franka_example_controllers record_directional_cbf.py _output_dir:="$(pwd)/data/cbf"
```

## Plot

The plotter is installed in the package and also provided as a standalone file.
It can run without ROS once NumPy and Matplotlib are installed.

```bash
rosrun franka_example_controllers plot_directional_cbf.py data/cbf/FILE.csv
```

Or with the separately downloaded script:

```bash
python3 plot_directional_cbf.py data/cbf/FILE.csv --show
python3 plot_directional_cbf.py data/cbf/FILE.csv --experiment 2 --output data/cbf/experiment_2.pdf
```

Default output is a four-panel PNG beside the CSV. --output also supports PDF
and SVG. No interactive window is required unless --show is requested. Clock
rewinds/controller restarts create a new time_segment; use --segment 1 to select
one, otherwise segments are shown on their own relative time axes in the same
figure. Attempts and missing-sample gaps are not joined by plot lines.

## Meaning of the quantities

| CSV field | Meaning |
| --- | --- |
| cbf_h | Kmax - K_dir, in J |
| cbf_constraint_safe | Model residual a*(tau_command-coriolis)+b+alpha*h, in J/s |
| kinetic_energy_dir | 0.5*Lambda_dir*(J_dir*dq)^2, in J |
| svd_jacobian_1 ... svd_jacobian_6 | Six singular values, descending, of the full 6x7 geometric Jacobian expressed in the base frame |

The six singular values include both translational and rotational Jacobian rows;
they are not the SVD of the 1x7 directional Jacobian. Translational and angular
rows have different physical units, so this is a diagnostic of the chosen full
Jacobian, not a unit-invariant singularity metric. Values near zero identify loss
of rank in that representation.

cbf_constraint_safe is computed on the command actually sent by this controller.
For an accepted CBF solution it equals the Python expression a*usafe+b+alpha*h.
During bypass it describes the nominal command. During an abort it describes
the braking command, using u=tau_command-coriolis consistently with the compensated
model. The word safe is a field name, not a guarantee that this value is positive.
The plot marks aborted samples red and CBF-disabled samples gray.

cbf_constraint_qp separately records the finite optimal QP candidate residual,
including a candidate subsequently rejected by the residual check. It is NaN
when no such candidate exists. This lets you distinguish the rejected candidate
from the braking command actually applied. No previous candidate is carried over.

Additional columns retain Kmax, alpha, residual tolerance, direction, actual
control period dt, cbf_active, task_aborted, cbf_solution_applied and solver_status.
The energy panel includes Kmax; h and the residual have zero reference lines.
The residual panel also shows the negative acceptance tolerance.

All four requested quantities are from the same controller state. stamp_ns is
the exact integer ROS timestamp, time_s is relative simulation time in the
segment, and sample_index is the controller's own counter (not ROS header.seq).
Unavailable quantities are NaN rather than misleading zeros. The residual uses
the analytical model and numerical Jdot/Mdot estimates; it is not a measured
finite difference of h or a verification of the Gazebo-applied actuator force.

## Collection and validation limits

A realtime publisher transfers diagnostics to /directional_cbf/diagnostics.
SVD calculation is only performed after a successful publisher trylock. Disk I/O
runs in a separate Python process with a bounded queue. Both the publisher and
ROS transport can drop samples: missing_samples_before records counter gaps
between received samples, and the recorder warns about gaps/queue overflows.
Gaps before the first received sample or after the last cannot be inferred.
The stream attempts one diagnostic message per controller update; it is not a
guaranteed lossless capture. The added SVD has a computation cost that should be
measured in the actual simulation.

Validated while preparing this patch: exact patch application on the stated
commit, XML parsing, Python syntax, CSV column mapping, integer timestamps,
clock resets, sample gaps, missing values, shutdown queue draining and PNG/PDF
plot generation using synthetic data. Full ROS compilation, realtime transport
and Gazebo execution were not available and remain to be tested locally.
