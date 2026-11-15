#include "walk_sim.hpp"

#include <GLFW/glfw3.h>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <type_traits>

namespace dog::sim {
namespace {
struct View {
    Simulation& simulation;
    mjvCamera camera{};
    mjvOption option{};
    mjvScene* scene = nullptr;
    bool paused = false;
    bool left = false, middle = false, right = false;
    double mouseX = 0, mouseY = 0;
};
View& view(GLFWwindow* window) { return *static_cast<View*>(glfwGetWindowUserPointer(window)); }

template <class Function> void moveCamera(Function function, View& v, int action, double dx, double dy) {
    // Recent MuJoCo releases removed the scene argument from this function.
    if constexpr (std::is_invocable_v<Function, const mjModel*, int, mjtNum, mjtNum, mjvCamera*>) {
        function(v.simulation.model(), action, dx, dy, &v.camera);
    } else {
        function(v.simulation.model(), action, dx, dy, v.scene, &v.camera);
    }
}

void keyboard(GLFWwindow* window, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;
    auto& v = view(window);
    if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, GLFW_TRUE);
    if (key == GLFW_KEY_SPACE) v.paused = !v.paused;
    if (key == GLFW_KEY_R) v.simulation.reset();
}
void buttons(GLFWwindow* window, int, int, int) {
    auto& v = view(window);
    v.left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    v.middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    v.right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window, &v.mouseX, &v.mouseY);
}
void mouse(GLFWwindow* window, double x, double y) {
    auto& v = view(window);
    const double dx = x-v.mouseX, dy = y-v.mouseY;
    v.mouseX = x; v.mouseY = y;
    if (!v.left && !v.middle && !v.right) return;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    if (height <= 0) return;
    const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    const int action = v.right ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
                     : v.left ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V) : mjMOUSE_ZOOM;
    moveCamera(&mjv_moveCamera, v, action, dx/height, dy/height);
}
void scroll(GLFWwindow* window, double, double dy) {
    auto& v = view(window);
    moveCamera(&mjv_moveCamera, v, mjMOUSE_ZOOM, 0, -0.05*dy);
}
}  // namespace

void runViewer(Simulation& simulation, double duration, double speed,
               const std::function<void()>& observe, const std::function<bool()>& keepRunning) {
    if (!glfwInit()) throw std::runtime_error("GLFW initialization failed; use --headless without a desktop");
    GLFWwindow* window = glfwCreateWindow(1280, 800, "Dog walk simulation", nullptr, nullptr);
    if (!window) { glfwTerminate(); throw std::runtime_error("Cannot create OpenGL window"); }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    View v{simulation};
    mjv_defaultCamera(&v.camera);
    mjv_defaultOption(&v.option);
    v.option.geomgroup[3] = 0;  // hide duplicate CAD collision meshes
    v.camera.distance = 1.35;
    v.camera.azimuth = 135;
    v.camera.elevation = -22;
    const int base = mj_name2id(simulation.model(), mjOBJ_BODY, "base_link");
    Eigen::Vector3d previousBase = Eigen::Map<const Eigen::Vector3d>(simulation.data()->xpos+3*base);
    for (int i = 0; i < 3; ++i) v.camera.lookat[i] = previousBase[i];
    v.camera.lookat[2] -= 0.1;
    glfwSetWindowUserPointer(window, &v);
    glfwSetKeyCallback(window, keyboard);
    glfwSetMouseButtonCallback(window, buttons);
    glfwSetCursorPosCallback(window, mouse);
    glfwSetScrollCallback(window, scroll);

    mjvScene scene{};
    mjrContext context{};
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    mjv_makeScene(simulation.model(), &scene, 3000);
    v.scene = &scene;
    mjr_makeContext(simulation.model(), &context, mjFONTSCALE_150);
    try {
        while (keepRunning() && !glfwWindowShouldClose(window) && simulation.data()->time < duration-1e-10) {
            const auto frameStart = std::chrono::steady_clock::now();
            if (!v.paused) {
                const double targetTime = std::min(duration, simulation.data()->time+speed/60);
                while (keepRunning() && simulation.data()->time < targetTime-1e-10) {
                    simulation.step();
                    observe();
                }
            }
            const Eigen::Vector3d position = Eigen::Map<const Eigen::Vector3d>(simulation.data()->xpos+3*base);
            for (int i = 0; i < 3; ++i) v.camera.lookat[i] += position[i]-previousBase[i];
            previousBase = position;
            mjv_updateScene(simulation.model(), simulation.data(), &v.option, nullptr,
                            &v.camera, mjCAT_ALL, &scene);
            using M3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
            const Eigen::Map<const M3> rotation(simulation.data()->xmat+9*base);
            for (const auto& leg : simulation.legs()) {
                if (scene.ngeom == scene.maxgeom) break;
                const auto target = footTarget(simulation.options().gait, leg.startsWithSwing, simulation.phase());
                const Eigen::Vector3d robotPoint(leg.closureX, leg.hip.y()+target.position.x(),
                                                leg.hip.z()+target.position.y());
                const Eigen::Vector3d worldPoint = position+rotation*simulation.modelToRobot().transpose()*robotPoint;
                const mjtNum size[3]{0.006, 0, 0};
                const float color[4]{target.support ? 0.1f : 1.0f, 0.65f, target.support ? 1.0f : 0.1f, 0.9f};
                mjv_initGeom(&scene.geoms[scene.ngeom++], mjGEOM_SPHERE, size, worldPoint.data(), nullptr, color);
            }
            int width, height;
            glfwGetFramebufferSize(window, &width, &height);
            const mjrRect viewport{0, 0, width, height};
            mjr_render(viewport, &scene, &context);
            const auto metrics = simulation.metrics();
            char text[512];
            std::snprintf(text, sizeof(text),
                "%s%s\nt=%.2f  phase=%.2f  kp=%.2f kd=%.2f\n"
                "closure=%.3g m  joint RMS=%.3f rad\nFF=%s  max tau_ff=%.2f Nm\ncontacts=%d  fallen=%s\n"
                "Space: pause | R: reset | Esc: quit\nBlue: support target | Orange: swing target",
                simulation.options().kinematic ? "KINEMATIC - fixed body, prescribed joints" : "DYNAMICS - floating body, PD motors",
                v.paused ? " [PAUSED]" : "", simulation.data()->time, simulation.phase(),
                simulation.options().kp, simulation.options().kd, metrics.closureError,
                metrics.trackingError, simulation.options().gravityFeedforward ? "on" : "off",
                metrics.maxFeedforwardTorque, metrics.groundContacts, metrics.fallen ? "yes" : "no");
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, text, nullptr, &context);
            glfwSwapBuffers(window);
            glfwPollEvents();
            std::this_thread::sleep_until(frameStart+std::chrono::microseconds(16667));
        }
    } catch (...) {
        mjr_freeContext(&context); mjv_freeScene(&scene);
        glfwDestroyWindow(window); glfwTerminate();
        throw;
    }
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwDestroyWindow(window);
    glfwTerminate();
}

}  // namespace dog::sim
