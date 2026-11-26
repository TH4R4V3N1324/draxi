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
draxi_sim [model.xml] [--headless SECONDS]
```

With no model it loads `cad/draxi.xml`. `--headless` runs without a window and exits non-zero if the
robot fell, for quick checks.

Viewer: Space pause, Backspace reset to the `stand` keyframe, left drag orbit, right drag pan,
scroll zoom.

## How it works

Draxi starts in the `stand` keyframe and a PD loop holds each joint at its target through the torque
motors (`Controller` in [main.cpp](main.cpp), gains scaled by each motor's torque limit). gait_core
is already linked: driving the robot means filling `Controller::targets` from the gait engine each
tick.
