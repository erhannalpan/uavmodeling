# Quadrotor MATLAB–Gazebo Co-Simulation

**Erhan Alpan — UAV project**

The working co-simulation code, arranged for GitHub with explanatory comments.
The original `uav_open_loop.slx` is unchanged. The ROS adapter, CMake, manifest,
world, and model configuration retain their supplied code/settings; comments
explain their role. `model.sdf` is the corrected model used in the project.

## Files

| Folder/file | Contents |
|---|---|
| `matlab/uav_open_loop.slx` | Original model, actuator subsystems, ROS blocks, embedded functions |
| `matlab/openloopmodel.m` | Original initialization/plotting script, comments added |
| `matlab/embedded_functions/` | Readable copies of the two functions embedded in the SLX |
| `matlab/compare_cosim.m` | Our timestamp/interpolation/plot/error steps collected into one script |
| `ros2/gazebo_matlab_cosim/` | Adapter source, CMakeLists, package.xml, model and world |
| `terminal_commands/` | The build/start/topic-check commands, one commented file per task |

Editing an extracted embedded-function `.m` alone does not edit the block inside
the SLX; edit its MATLAB Function block when changing the actual model.

## Software used

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.11.0 (Harmonic), MATLAB/Simulink R2026a,
and ROS Toolbox. The ROS installation needs `ros_gz_bridge`, `actuator_msgs`,
`std_msgs`, `nav_msgs`, `rclcpp`, `ament_cmake`, colcon, and the lint dependencies
listed in package.xml. Install the environment before building this source.
These are the project commands, not an automatic OS/software installer.

## Build and open the systems

1. Put the supplied `gazebo_matlab_cosim` directory inside `~/ros2_ws/src/`.
   For an existing installation, retain your current working source as a backup
   before replacing it with a GitHub copy.
2. From the repository root run `bash terminal_commands/01_build_adapter.sh`.
3. Open four terminals in this repository and run, one command per terminal:

```bash
bash terminal_commands/02_open_matlab.sh
```

```bash
bash terminal_commands/03_open_gazebo.sh
```

```bash
bash terminal_commands/04_start_bridge.sh
```

```bash
bash terminal_commands/05_start_adapter.sh
```

The scripts use the original project paths. Change the MATLAB executable path
if your installation differs. Keep ROS domain ID **0** in all participating
processes and in Simulink's ROS Network settings. Keep Gazebo paused while
preparing the model. The resource path lets `model://quadrotor` resolve correctly.

In MATLAB, navigate to this repository's `matlab/` folder, then run:

```matlab
openloopmodel
open_system('uav_open_loop.slx')
```

Initialization sets m, g, L, kf, km, the whole-vehicle inertias, and the motor time
constant. It also draws the static thrust/torque curves. The saved SLX has the
final pitch configuration. Pacing is already enabled; the command we used is:

```matlab
set_param('uav_open_loop','EnablePacing','on','PacingRate','1');
```

## How the parts connect

Four motor-speed references feed both the local motor models and the ROS branch.
The ROS Mux is **before** the local motor lag. Its front/right/back/left order
matches Gazebo motor indices 0/1/2/3. A Zero-Order Hold samples the vector at
0.05 s. Blank Message supplies a Float64MultiArray bus; Bus Assignment writes
`data` and `data_SL_Info.CurrentLength=uint32(4)`. The maximum data array length
is 4, and all three bus inputs have compatible 0.05 s timing.

Publish sends `/quadrotor/matlab_motor_commands`. The C++ adapter copies the four
values into `Actuators.velocity` on `/quadrotor/command/motor_speed`. The bridge
translates ROS messages to Gazebo Transport. The adapter is message conversion,
not an autopilot or controller. It checks array length and otherwise copies
values unchanged, without additional filtering or range checks.

Gazebo publishes `/quadrotor/odometry` at 200 Hz. Simulink Subscribe samples at
0.05 s and the Bus Selector extracts position, orientation, twist, and stamp.
The conversion block produces `[pn pe h u v w phi theta psi p q r]'` and Gazebo
time `double(sec)+double(nanosec)*1e-9`. Publish uses reliable QoS; the uploaded
Subscribe block uses best effort. Both are volatile, history depth 10.

The analytical axes are forward/right/down. Project north/east/up are Gazebo
world +X/-Y/+Z. The conversion assumes body-frame Gazebo twist and uses
`D=diag([1 -1 -1])`, with converted attitude `Rm=D*R*D`. Euler angles and body
angular rates are different quantities; angles are not generally integrals of
p/q/r. The extracted conversion function includes the full comments.

The SDF base mass is 0.96 kg and each rotor adds 0.01 kg. Its base-only tensor
is [0.01,0.01,0.02] kg m². Including the rotor links and parallel-axis terms gives
the analytical whole-vehicle tensor [0.01085536,0.01085536,0.02168] kg m².
The supplied initialization contains these final matching values.

## Tests we performed

| Setting | Equal-motor vertical | Differential pitch |
|---|---|---|
| Front/right/back/left references (rad/s) | 520 / 520 / 520 / 520 | 520.2 / 520 / 519.8 / 520 |
| Initial altitude | 2 m | 20 m |
| Other vehicle states | Zero | Zero |
| Actual initial motor speeds | Zero | Zero |
| Local stop time | 10 s | 5 s |
| Motor rise/fall time constant | 0.12 s | 0.12 s |

The supplied world and SLX currently store the **pitch** setup. For the vertical
test, change the quadrotor include pose in the world to `0 0 2 0 0 0`, set the
SLX state Integrator2 initial altitude to 2, set all four Step final values to
520, and set stop time to 10. The Step blocks are named `Omegra_ref1` through
`Omegra_ref4`. References step at time 0; actual motor Integrators start at zero.

The supplied world physics step is **0.001 s**. Simulink uses **ode4, 0.005 s**;
ROS command/subscription and workspace logging use **0.05 s**. Pacing at 1 is
not lockstep synchronization. Start from matching initial state and clock origin.
Run Simulink, then immediately Play Gazebo. Pause Gazebo after the run. Do not
reset while capturing data. Between tests, a full reset must restore time, pose,
velocity, and motor state; otherwise restart Gazebo with the same launch command.

## Plot and calculate errors

After the run, `out` contains `simu_states`, `gaz_states`, and `gz_time` timeseries.
Run `compare_cosim` in MATLAB. Its first settings select `pitch` or `vertical`
and the explicitly chosen startup boundary. It creates separate figure windows
and prints raw/retained RMSE and maximum absolute errors.

The processing is the same sequence used in the project: inspect acquisition
order for resets, remove uninitialized zero timestamps, retain one observation
per distinct timestamp, interpolate analytical states at Gazebo timestamps in
the common interval, and calculate Gazebo-minus-Simulink errors. No fitted time
shift or clock scaling is used. The script defines all comparison variables,
including `t_compare`, rather than depending on an earlier workspace snippet.

For the final pitch case use `comparison_start=0`. For the later reported
vertical case we used `0.060` s after inspecting a startup velocity anomaly.
Keep raw observations and document any exclusion for each new run; it is not an
automatic filter. A shorter logged Gazebo interval does not by itself prove a
constant slowdown. The script prints the actual retained interval and count.
A commented save command preserves raw numeric arrays as MAT v7.

### Final reported errors

| Case/state | RMSE | Maximum absolute error |
|---|---:|---:|
| Vertical h (m) | 0.025823 | 0.044337 |
| Vertical w (m/s) | 0.009887 | 0.019257 |
| Pitch theta (rad) | 0.000023057 | 0.000023953 |
| Pitch q (rad/s) | 0.00042427 | 0.0017937 |
| Pitch pn (m) | 0.0010628 | 0.0023509 |
| Pitch h (m) | 0.011749 | 0.019825 |

These are the final MATLAB results supplied by the owner. Their raw recordings
were not uploaded with the source, so the repository does not claim to regenerate
those exact numbers without those data. Re-running uses the same physical setup
but asynchronous communication/timing can change sample coverage and errors.

## Closing and common fixes

Stop Simulink, save the model if intentionally edited, export raw data, then
pause Gazebo and Ctrl+C the bridge/adapter/Gazebo terminals. Close MATLAB normally.
Source and the compiled workspace remain saved. Live ROS handles cannot be saved
to MAT and restored; launching recreates them.

- Model URI error: check GZ_SIM_RESOURCE_PATH points to the parent `models/`.
- Missing message bus: reselect the ROS 2 block message type, check array length
  4 in Variable Size Messages, and update the diagram after initialization.
- Bus sample mismatch: retain the 0.05 s ZOH, Blank Message, and length Constant.
- No commands/motion: check domain, bridge/adapter processes, actual values, and
  that Gazebo simulation time advances. Use `06_check_topics.sh`.
- Invalid inertia/XML: use the corrected SDF; the failed intermediate tensor is
  not part of this package. Do not replace base-only inertia with total inertia.
- Euler pitch near 90°: principal angle extraction and Euler singularity can
  make naive angle comparisons misleading; these tests stay below that region.

## Upload to GitHub

Upload the contents of this folder, keeping the subdirectories. Include `.slx`
and `.m` directly. Generated `.slxc`, `slprj`, and colcon `build/install/log` are
not needed. The original package manifest declares Apache-2.0; its license text
is included. No GitHub repository has been created or pushed automatically.

Source/XML preservation and terminal shell syntax were checked during packaging.
MATLAB/ROS/Gazebo are unavailable in that environment, so the collected
comparison script has not been executed there; the original simulations were
run on the owner's PC.
