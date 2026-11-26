// Draxi in MuJoCo: loads the robot, starts it in the "stand" keyframe and runs the same control
// loop the Teensy will: read joints and IMU through the Hal, update the controller, send commands.
// MujocoHal stands in for the hardware and the RobStride motors' own PD loop.
//
//   draxi_sim [model.xml] [--headless SECONDS]
//
// Viewer keys: Space pause, Backspace reset to stand, mouse to orbit/pan/zoom.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <string>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "draxi/stand_controller.hpp"
#include "mujoco_hal.hpp"

namespace {

mjModel* m = nullptr;
mjData* d = nullptr;
std::unique_ptr<draxi::MujocoHal> hal;
draxi::StandController controller;
draxi::JointStates jointStates{};
draxi::ImuSample imu{};
draxi::JointCommands commands{};
int standKey = -1;

void resetToStand() {
    if (standKey >= 0) {
        mj_resetDataKeyframe(m, d, standKey);
    } else {
        mj_resetData(m, d);
    }
    mj_forward(m, d);
    hal->readJoints(jointStates);
    controller.reset(jointStates);
}

// One control tick, then one physics step. The controller runs at the physics rate for now
void step() {
    hal->readJoints(jointStates);
    hal->readImu(imu);
    controller.update(jointStates, imu, static_cast<float>(m->opt.timestep), commands);
    hal->sendCommands(commands);

    hal->applyMotors();
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
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, status, "Space pause  Backspace reset",
                    &con);

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
    try {
        hal = std::make_unique<draxi::MujocoHal>(m, d);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Model does not match shared/include/draxi/joints.hpp: %s\n",
                     e.what());
        return EXIT_FAILURE;
    }
    standKey = mj_name2id(m, mjOBJ_KEY, "stand");
    resetToStand();

    int result = headlessSeconds >= 0 ? runHeadless(headlessSeconds) : runViewer();

    mj_deleteData(d);
    mj_deleteModel(m);
    return result;
}
