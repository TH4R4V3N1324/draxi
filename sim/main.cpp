// Draxi in MuJoCo: loads the robot, starts it in the "stand" keyframe and holds every joint there
// with a PD loop on the torque motors. This is the place gait_core plugs in: replace the fixed
// targets in Controller::targets with the gait engine's joint targets each tick.
//
//   draxi_sim [model.xml] [--headless SECONDS]
//
// Viewer keys: Space pause, Backspace reset to stand, mouse to orbit/pan/zoom.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

namespace {

struct Controller {
  std::vector<int> qposAdr;     // per actuator: its joint's index in qpos
  std::vector<int> dofAdr;      // per actuator: its joint's index in qvel
  std::vector<mjtNum> targets;  // per actuator: joint target (rad)
  std::vector<mjtNum> kp, kd;

  void init(const mjModel* m) {
    for (int i = 0; i < m->nu; ++i) {
      if (m->actuator_trntype[i] != mjTRN_JOINT) {
        mju_error("actuator %d does not drive a joint", i);
      }
      int joint = m->actuator_trnid[2 * i];
      qposAdr.push_back(m->jnt_qposadr[joint]);
      dofAdr.push_back(m->jnt_dofadr[joint]);
      // Gains scaled by the motor's torque limit, so the 17 Nm and 5.5 Nm servos feel alike
      mjtNum maxTorque = m->actuator_ctrlrange[2 * i + 1];
      kp.push_back(4.0 * maxTorque);
      kd.push_back(0.1 * maxTorque);
    }
    targets.assign(m->nu, 0);
  }

  void holdCurrentPose(const mjData* d) {
    for (size_t i = 0; i < targets.size(); ++i) targets[i] = d->qpos[qposAdr[i]];
  }

  void apply(const mjModel* m, mjData* d) const {
    for (int i = 0; i < m->nu; ++i) {
      mjtNum err = targets[i] - d->qpos[qposAdr[i]];
      mjtNum torque = kp[i] * err - kd[i] * d->qvel[dofAdr[i]];
      d->ctrl[i] = mju_clip(torque, m->actuator_ctrlrange[2 * i], m->actuator_ctrlrange[2 * i + 1]);
    }
  }
};

mjModel* m = nullptr;
mjData* d = nullptr;
Controller controller;
int standKey = -1;

void resetToStand() {
  if (standKey >= 0) {
    mj_resetDataKeyframe(m, d, standKey);
  } else {
    mj_resetData(m, d);
  }
  mj_forward(m, d);
  controller.holdCurrentPose(d);
}

void step() {
  controller.apply(m, d);
  mj_step(m, d);
}

// ---------------------------------------------------------------------------
// Viewer
// ---------------------------------------------------------------------------
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
bool paused = false;
bool buttonLeft = false, buttonMiddle = false, buttonRight = false;
double lastX = 0, lastY = 0;

void onKey(GLFWwindow*, int key, int, int act, int) {
  if (act != GLFW_PRESS) return;
  if (key == GLFW_KEY_BACKSPACE) resetToStand();
  if (key == GLFW_KEY_SPACE) paused = !paused;
}

void onMouseButton(GLFWwindow* window, int, int, int) {
  buttonLeft = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  buttonMiddle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  buttonRight = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  glfwGetCursorPos(window, &lastX, &lastY);
}

void onMouseMove(GLFWwindow* window, double x, double y) {
  if (!buttonLeft && !buttonMiddle && !buttonRight) return;
  double dx = x - lastX, dy = y - lastY;
  lastX = x;
  lastY = y;
  int width, height;
  glfwGetWindowSize(window, &width, &height);
  bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
               glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  mjtMouse action = buttonRight  ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
                    : buttonLeft ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V)
                                 : mjMOUSE_ZOOM;
  mjv_moveCamera(m, action, dx / height, dy / height, &cam);
}

void onScroll(GLFWwindow*, double, double yOffset) {
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yOffset, &cam);
}

int runViewer() {
  if (!glfwInit()) mju_error("Could not initialise GLFW");
  GLFWwindow* window = glfwCreateWindow(1280, 900, "Draxi", nullptr, nullptr);
  if (!window) mju_error("Could not create a window");
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // Follow the robot's body
  cam.type = mjCAMERA_TRACKING;
  cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "base_link");
  cam.distance = 1.0;
  cam.elevation = -20;
  cam.azimuth = 135;

  glfwSetKeyCallback(window, onKey);
  glfwSetMouseButtonCallback(window, onMouseButton);
  glfwSetCursorPosCallback(window, onMouseMove);
  glfwSetScrollCallback(window, onScroll);

  double wallStart = glfwGetTime();
  mjtNum simStart = d->time;
  while (!glfwWindowShouldClose(window)) {
    // Run the simulation in real time, catching up at most 0.1 s if rendering stalls
    if (paused) {
      wallStart = glfwGetTime();
      simStart = d->time;
    } else {
      double wallElapsed = glfwGetTime() - wallStart;
      if (wallElapsed - (d->time - simStart) > 0.1) simStart = d->time - wallElapsed;
      while (d->time - simStart < wallElapsed) step();
    }

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    char status[128];
    std::snprintf(status, sizeof(status), "t = %.2f s%s", d->time, paused ? "  (paused)" : "");
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, status,
                "Space pause  Backspace reset", &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  glfwTerminate();
  return EXIT_SUCCESS;
}

// Steps without a window and reports whether Draxi stayed up: for CI and quick checks
int runHeadless(double seconds) {
  int base = mj_name2id(m, mjOBJ_BODY, "base_link");
  mjtNum startHeight = d->xpos[3 * base + 2];
  for (double report = 0; d->time < seconds;) {
    step();
    if (d->time >= report) {
      std::printf("t = %5.2f s  base height = %.3f m\n", d->time, d->xpos[3 * base + 2]);
      report += 0.5;
    }
  }
  mjtNum endHeight = d->xpos[3 * base + 2];
  bool standing = endHeight > 0.75 * startHeight;
  std::printf("%s (base %.3f m, started at %.3f m)\n", standing ? "STANDING" : "FELL", endHeight,
              startHeight);
  return standing ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace

int main(int argc, char** argv) {
  std::string modelPath = DRAXI_DEFAULT_MODEL;
  double headlessSeconds = -1;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--headless") && i + 1 < argc) {
      headlessSeconds = std::atof(argv[++i]);
    } else {
      modelPath = argv[i];
    }
  }

  std::printf("MuJoCo %s, model %s\n", mj_versionString(), modelPath.c_str());
  char error[1000] = "";
  m = mj_loadXML(modelPath.c_str(), nullptr, error, sizeof(error));
  if (!m) {
    std::fprintf(stderr, "Could not load model: %s\n", error);
    return EXIT_FAILURE;
  }
  d = mj_makeData(m);
  controller.init(m);
  standKey = mj_name2id(m, mjOBJ_KEY, "stand");
  resetToStand();

  int result = headlessSeconds >= 0 ? runHeadless(headlessSeconds) : runViewer();

  mj_deleteData(d);
  mj_deleteModel(m);
  return result;
}
