// Draxi in MuJoCo: loads the robot, starts it in the "stand" keyframe and runs the same control
// loop the Teensy will: read joints and IMU through the Hal, update the controller, send commands.
// MujocoHal stands in for the hardware and the RobStride motors' own PD loop.
//
//   draxi_sim [model.xml] [--headless SECONDS] [--control-hz HZ] [--stand] [--walk VX] [--strafe VY] [--turn WZ]
//             [--record FILE.mp4]
//
// The controller is GaitController (gait_core's engine with DraxiKinematics) unless --stand picks
// StandController. --walk VX walks forward at VX m/s, --strafe VY sideways (+ left) and --turn WZ
// turns (rad/s, + anticlockwise), from 3 s in
// when headless. --record renders the run (as long as --headless says, 20 s otherwise) offscreen to
// a video, front and side views side by side at 30 frames a second; it needs ffmpeg on the PATH.
//
// Viewer keys: Space pause, Backspace reset to stand, Up/Down walk faster/slower, Left/Right turn,
// Q/E strafe left/right, X stop, mouse to orbit/pan/zoom.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "draxi/gaitController.hpp"
#include "draxi/stand_controller.hpp"
#include "mujoco_hal.hpp"

namespace {

mjModel* m = nullptr;
mjData* d = nullptr;
std::unique_ptr<draxi::MujocoHal> hal;
draxi::StandController standController;
draxi::GaitController gaitController;
bool useGait = true;  // false with --stand
double walkSpeed = 0;  // m/s, from --walk
double strafeSpeed = 0;  // m/s, from --strafe
double turnRate = 0;  // rad/s, from --turn
float vx = 0, vy = 0, wz = 0;  // the velocity command
draxi::JointStates jointStates{};
draxi::ImuSample imu{};
draxi::JointCommands commands{};
int standKey = -1;

double controlHz = 500.0;  // controller rate, set by --control-hz
int controlEvery = 1;  // physics steps per control tick, set in main()
long stepCount = 0;

void resetToStand() {
    if (standKey >= 0) {
        mj_resetDataKeyframe(m, d, standKey);
    } else {
        mj_resetData(m, d);
    }
    mj_forward(m, d);
    hal->readJoints(jointStates);
    standController.reset(jointStates);
    gaitController.reset(jointStates);
    vx = vy = wz = 0;
    gaitController.setVelocity(0, 0, 0);

    stepCount = 0;
}

// The motors (applyMotors) run every physics step; the controller only every controlEvery steps,
// the way the Teensy will run slower than the RobStride's own loop
void step() {
    if (stepCount % controlEvery == 0) {
        hal->readJoints(jointStates);
        hal->readImu(imu);
        const float dt = static_cast<float>(1.0 / controlHz);
        if (useGait) {
            gaitController.update(jointStates, imu, dt, commands);
        } else {
            standController.update(jointStates, imu, dt, commands);
        }
        hal->sendCommands(commands);
    }
    ++stepCount;

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
    if (key == GLFW_KEY_UP) vx += 0.02f;
    if (key == GLFW_KEY_DOWN) vx -= 0.02f;
    if (key == GLFW_KEY_LEFT) wz += 0.2f;
    if (key == GLFW_KEY_RIGHT) wz -= 0.2f;
    if (key == GLFW_KEY_Q) vy += 0.02f;
    if (key == GLFW_KEY_E) vy -= 0.02f;
    if (key == GLFW_KEY_X) vx = vy = wz = 0;
    gaitController.setVelocity(vx, vy, wz);
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
        std::snprintf(status, sizeof(status), "t = %.2f s  vx = %.2f m/s  vy = %.2f m/s  wz = %.1f rad/s%s", d->time,
                      vx, vy, wz, paused ? "  (paused)" : "");
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
// Why Draxi has fallen, or nullptr if it hasn't: anything but a foot on the floor, the body tilted
// past 30 degrees, or the body low. Checks contacts, so a fall onto the head that leaves the body
// high still counts.
const char* fallen(int base, mjtNum startHeight) {
    static const int feet[2] = {mj_name2id(m, mjOBJ_BODY, "foot_L"), mj_name2id(m, mjOBJ_BODY, "foot_R")};
    static char reason[96];
    for (int c = 0; c < d->ncon; ++c) {
        const int b1 = m->geom_bodyid[d->contact[c].geom1], b2 = m->geom_bodyid[d->contact[c].geom2];
        if (b1 != 0 && b2 != 0) continue;  // not the floor (the world body)
        const int body = b1 + b2;
        if (body != feet[0] && body != feet[1]) {
            std::snprintf(reason, sizeof(reason), "%s touched the floor", mj_id2name(m, mjOBJ_BODY, body));
            return reason;
        }
    }
    const mjtNum tilt = std::acos(mju_clip(d->xmat[9 * base + 8], -1, 1)) * 180 / mjPI;
    if (tilt > 30) {
        std::snprintf(reason, sizeof(reason), "body tilted %.0f degrees", tilt);
        return reason;
    }
    if (d->xpos[3 * base + 2] < 0.75 * startHeight) return "body too low";
    return nullptr;
}

int runHeadless(double seconds) {
    int base = mj_name2id(m, mjOBJ_BODY, "base_link");
    mjtNum startHeight = d->xpos[3 * base + 2];
    mjtNum walkStartX = 0, walkStartY = 0, walkStartTime = 0;
    double turnedSoFar = 0, lastHeading = 0;
    bool haveHeading = false;
    for (double report = 0; d->time < seconds;) {
        if ((walkSpeed != 0 || strafeSpeed != 0 || turnRate != 0) && walkStartTime == 0 && d->time >= 3.0) {
            vx = static_cast<float>(walkSpeed);
            vy = static_cast<float>(strafeSpeed);
            wz = static_cast<float>(turnRate);
            gaitController.setVelocity(vx, vy, wz);
            walkStartX = d->xpos[3 * base];
            walkStartY = d->xpos[3 * base + 1];
            walkStartTime = d->time;
            std::printf("t = %5.2f s  walking at %.2f m/s forward, %.2f m/s left, turning %.2f rad/s\n", d->time, walkSpeed,
                        strafeSpeed, turnRate);
        }
        step();
        if (walkStartTime > 0) {
            // The body's heading, unwrapped, for how far it turned
            const mjtNum* q = d->xquat + 4 * base;
            const double heading = std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
            if (haveHeading) turnedSoFar += std::remainder(heading - lastHeading, 2 * mjPI);
            lastHeading = heading;
            haveHeading = true;
        }
        if (const char* why = fallen(base, startHeight)) {
            std::printf("FELL at t = %.2f s: %s\n", d->time, why);
            return EXIT_FAILURE;
        }
        if (d->time >= report) {
            std::printf("t = %5.2f s  base height = %.3f m  x = %+.3f m\n", d->time,
                        d->xpos[3 * base + 2], d->xpos[3 * base]);
            report += 0.5;
        }
    }
    if (walkStartTime > 0 && turnRate != 0) {
        std::printf("Turned %+.2f rad in %.0f s: %.0f%% of the %.2f rad/s asked\n", turnedSoFar, d->time - walkStartTime,
                    100 * turnedSoFar / (turnRate * (d->time - walkStartTime)), turnRate);
    }
    if (walkStartTime > 0) {
        const mjtNum distance = d->xpos[3 * base] - walkStartX;
        std::printf("Walked %+.3f m in %.0f s: %.4f m/s of %.2f\n", distance, d->time - walkStartTime,
                    distance / (d->time - walkStartTime), walkSpeed);
        if (strafeSpeed != 0) {
            const mjtNum sideways = d->xpos[3 * base + 1] - walkStartY;
            std::printf("Strafed %+.3f m in %.0f s: %.4f m/s of %.2f\n", sideways, d->time - walkStartTime,
                        sideways / (d->time - walkStartTime), strafeSpeed);
        }
    }
    std::printf("STANDING (base %.3f m, started at %.3f m)\n", d->xpos[3 * base + 2], startHeight);
    return EXIT_SUCCESS;
}

// Renders the run offscreen to a video through ffmpeg: front and side tracking views side by side,
// 30 frames a second, with the time and command. Walks and strafes as --walk and --strafe say, from
// 3 s in; carries on 1.5 s after a fall so it shows.
int runRecord(double seconds, const std::string& outPath) {
    constexpr int W = 640, H = 540;  // each view
    constexpr int FPS = 30;
    if (!glfwInit()) mju_error("Could not initialise GLFW");
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "Draxi", nullptr, nullptr);
    if (!window) mju_error("Could not create a window");
    glfwMakeContextCurrent(window);
    m->vis.global.offwidth = W;
    m->vis.global.offheight = H;
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_OFFSCREEN, &con);

    const int base = mj_name2id(m, mjOBJ_BODY, "base_link");
    const mjtNum startHeight = d->xpos[3 * base + 2];
    mjvCamera views[2];
    for (mjvCamera& v : views) {
        mjv_defaultCamera(&v);
        v.type = mjCAMERA_TRACKING;
        v.trackbodyid = base;
        v.distance = 0.85;
        v.elevation = -10;
    }
    views[0].azimuth = 180;  // from the front: the sideways rocking
    views[1].azimuth = 90;   // from the left side: the stepping
    const char* viewName[2] = {"front", "side"};

#ifdef _WIN32
#define DRAXI_POPEN _popen
#define DRAXI_PCLOSE _pclose
#else
#define DRAXI_POPEN popen
#define DRAXI_PCLOSE pclose
#endif
    const std::string cmd = "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s " + std::to_string(2 * W) + "x" +
        std::to_string(H) + " -r " + std::to_string(FPS) + " -i - -vf vflip -c:v libx264 -pix_fmt yuv420p -crf 20 \"" +
        outPath + "\"";
    FILE* pipe = DRAXI_POPEN(cmd.c_str(), "wb");
    if (!pipe) {
        std::fprintf(stderr, "Could not start ffmpeg\n");
        return EXIT_FAILURE;
    }
    std::vector<unsigned char> view(static_cast<size_t>(W) * H * 3), frame(static_cast<size_t>(2 * W) * H * 3);
    double nextFrame = 0, fellAt = -1;
    std::string fellWhy;
    bool started = false;
    while (d->time < seconds && (fellAt < 0 || d->time < fellAt + 1.5)) {
        if ((walkSpeed != 0 || strafeSpeed != 0 || turnRate != 0) && !started && d->time >= 3.0) {
            vx = static_cast<float>(walkSpeed);
            vy = static_cast<float>(strafeSpeed);
            wz = static_cast<float>(turnRate);
            gaitController.setVelocity(vx, vy, wz);
            started = true;
        }
        step();
        if (fellAt < 0) {
            if (const char* why = fallen(base, startHeight)) {
                fellAt = d->time;
                fellWhy = why;
                std::printf("FELL at t = %.2f s: %s\n", d->time, why);
            }
        }
        if (d->time + 1e-9 < nextFrame) continue;
        nextFrame += 1.0 / FPS;
        for (int k = 0; k < 2; ++k) {
            const mjrRect viewport = {0, 0, W, H};
            mjv_updateScene(m, d, &opt, nullptr, &views[k], mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);
            char status[160];
            if (fellAt >= 0) {
                std::snprintf(status, sizeof(status), "t = %.1f s   FELL at %.1f s: %s", d->time, fellAt, fellWhy.c_str());
            } else {
                std::snprintf(status, sizeof(status), "t = %.1f s   vx %.2f  vy %.2f m/s  wz %.1f rad/s", d->time, vx, vy, wz);
            }
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, viewName[k], nullptr, &con);
            mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, status, nullptr, &con);
            mjr_readPixels(view.data(), nullptr, viewport, &con);
            for (int row = 0; row < H; ++row) {
                std::memcpy(&frame[(static_cast<size_t>(row) * 2 * W + k * W) * 3], &view[static_cast<size_t>(row) * W * 3],
                            static_cast<size_t>(W) * 3);
            }
        }
        std::fwrite(frame.data(), 1, frame.size(), pipe);
    }
    DRAXI_PCLOSE(pipe);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwTerminate();
    std::printf("Recorded %s\n", outPath.c_str());
    return fellAt < 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace

int main(int argc, char** argv) {
    std::string modelPath = DRAXI_DEFAULT_MODEL;
    double headlessSeconds = -1;
    std::string recordPath;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--headless") && i + 1 < argc) {
            headlessSeconds = std::atof(argv[++i]);
        } else if (!std::strcmp(argv[i], "--stand")) {
            useGait = false;
        } else if (!std::strcmp(argv[i], "--walk") && i + 1 < argc) {
            walkSpeed = std::atof(argv[++i]);
        } else if (!std::strcmp(argv[i], "--strafe") && i + 1 < argc) {
            strafeSpeed = std::atof(argv[++i]);
        } else if (!std::strcmp(argv[i], "--turn") && i + 1 < argc) {
            turnRate = std::atof(argv[++i]);
        } else if (!std::strcmp(argv[i], "--record") && i + 1 < argc) {
            recordPath = argv[++i];
        } else if (!std::strcmp(argv[i], "--control-hz") && i + 1 < argc) {
            controlHz = std::atof(argv[++i]);
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

    if (!(controlHz > 0)) {
        std::fprintf(stderr, "--control-hz must be a positive number\n");
        return EXIT_FAILURE;
    }
    controlEvery = static_cast<int>(std::lround(1.0 / (controlHz * m->opt.timestep)));
    if (controlEvery < 1 || std::abs(controlEvery * m->opt.timestep * controlHz - 1.0) > 1e-6) {
        std::fprintf(stderr, "Control rate %.0f Hz is not a whole number of %.4f s physics steps\n",
                    controlHz, m->opt.timestep);
        return EXIT_FAILURE;
    }
    std::printf("Physics %.0f Hz, control %.0f Hz (every %d steps)\n",
                1.0 / m->opt.timestep, controlHz, controlEvery);

    gaitController.engine().log = [](gait_core::LogLevel, const char* message) {
        std::printf("[gait %.2f s] %s\n", d->time, message);
    };
    std::printf("Controller: %s\n", useGait ? "gait_core (GaitController)" : "StandController");
    if (!useGait) walkSpeed = strafeSpeed = turnRate = 0;
    resetToStand();

    int result = !recordPath.empty() ? runRecord(headlessSeconds >= 0 ? headlessSeconds : 20, recordPath)
               : headlessSeconds >= 0 ? runHeadless(headlessSeconds) : runViewer();

    mj_deleteData(d);
    mj_deleteModel(m);
    return result;
}
