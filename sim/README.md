# sim

Draxi in MuJoCo (C++). CMake downloads the prebuilt MuJoCo release (pinned version and SHA-256 in
[CMakeLists.txt](CMakeLists.txt)) and GLFW, so nothing needs installing beyond a C++ compiler and
CMake 3.24+.

## Build and run

```sh
git clone --recursive <repo-url>        # or: git submodule update --init --recursive
cmake -B build
cmake --build build --config Release --target draxi_sim
build/sim/Release/draxi_sim             # Windows (Visual Studio generator)
build/sim/draxi_sim                     # Linux
```

```
draxi_sim [model.xml] [--headless SECONDS] [--control-hz HZ] [--stand]
          [--walk VX] [--strafe VY] [--turn WZ] [--record FILE.mp4]
```

With no model it loads `cad/draxi.xml`. `--headless` runs without a window and exits non-zero if the
robot fell, for quick checks. `--walk`, `--strafe` and `--turn` set the command (m/s forward, m/s
left, rad/s anticlockwise) from 3 s in, and headless it reports how far it walked, strafed and
turned. `--record` renders the run offscreen to a video (front and side views, 30 fps) through
ffmpeg, which must be on the PATH.

Viewer: Space pause, Backspace reset to the `stand` keyframe, Up/Down walk faster/slower, Left/Right
turn, Q/E strafe left/right, X stop, left drag orbit, right drag pan, scroll zoom.

## How it works

The sim runs the same loop the Teensy will, through the `draxi::Hal` interface in
[control/](../control):

```
hal.readJoints -> hal.readImu -> controller.update -> hal.sendCommands -> mj_step
```

- [mujoco_hal.cpp](mujoco_hal.cpp) is the `Hal` on top of MuJoCo. It finds each joint in
  [shared/include/draxi/joints.hpp](../shared/include/draxi/joints.hpp) by name and refuses to start
  if the model's ranges or torque limits differ from it. It also stands in for the RobStride motors:
  before every physics step it runs their MIT-mode law,
  `kp*(pos - q) + kd*(vel - qd) + torqueFF`, clamped to the motor's peak torque.
- `draxi::StandController` (in `control/`) holds Draxi in the `stand` keyframe. The gait goes in
  its place later.
