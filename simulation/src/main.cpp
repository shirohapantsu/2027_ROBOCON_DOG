#include "walk_sim.hpp"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
volatile std::sig_atomic_t running = 1;
void requestStop(int) { running = 0; }

void help() {
    std::cout <<
        "Usage: walk_sim [options]\n"
        "  --mode dynamics|kinematic  PD physics (default) or fixed-body trajectory preview\n"
        "  --headless                 Run without an OpenGL window\n"
        "  --model PATH               CAD robot.xml (default: bundled realModel)\n"
        "  --duration SECONDS         Simulation duration (default: 10)\n"
        "  --period SECONDS           Gait period (default: 1)\n"
        "  --kp VALUE --kd VALUE      Output-shaft PD gains (default: 0.4, 0.02)\n"
        "  --feedforward on|off       Gravity + planned support force feedforward (default: on)\n"
        "  --step-length METRES       Step length (default: 0.14)\n"
        "  --step-height METRES       Swing height (default: 0.03)\n"
        "  --stand-z METRES           Hip-relative target Z (default: -0.274)\n"
        "  --speed FACTOR             Viewer playback speed (default: 1)\n"
        "  --csv PATH                 Save metrics every 10 ms\n"
        "  --snapshot PATH.mjb        Save final-pose model for rendering/inspection\n"
        "Viewer: Space pause, R reset, Esc quit, mouse rotate/pan/zoom.\n";
}
}  // namespace

int main(int argc, char** argv) {
    try {
        dog::sim::Options options;
        std::string modelPath = DOG_DEFAULT_MODEL, csvPath, snapshotPath;
        bool headless = false;
        double duration = 10, speed = 1;
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--help" || argument == "-h") { help(); return EXIT_SUCCESS; }
            if (argument == "--headless") { headless = true; continue; }
            auto value = [&]() -> std::string {
                if (i+1 == argc) throw std::invalid_argument("Missing value after " + argument);
                return argv[++i];
            };
            auto number = [&]() {
                const std::string text = value();
                std::size_t end = 0;
                const double result = std::stod(text, &end);
                if (end != text.size() || !std::isfinite(result)) {
                    throw std::invalid_argument("Invalid number for " + argument);
                }
                return result;
            };
            if (argument == "--model") modelPath = value();
            else if (argument == "--csv") csvPath = value();
            else if (argument == "--snapshot") snapshotPath = value();
            else if (argument == "--duration") duration = number();
            else if (argument == "--speed") speed = number();
            else if (argument == "--period") options.gait.period = number();
            else if (argument == "--kp") options.kp = number();
            else if (argument == "--kd") options.kd = number();
            else if (argument == "--feedforward") {
                const std::string setting = value();
                if (setting != "on" && setting != "off") {
                    throw std::invalid_argument("Feedforward must be on or off");
                }
                options.gravityFeedforward = setting == "on";
            }
            else if (argument == "--step-length") options.gait.stepLength = number();
            else if (argument == "--step-height") options.gait.stepHeight = number();
            else if (argument == "--stand-z") options.gait.standZ = number();
            else if (argument == "--mode") {
                const std::string mode = value();
                if (mode != "dynamics" && mode != "kinematic") {
                    throw std::invalid_argument("Mode must be dynamics or kinematic");
                }
                options.kinematic = mode == "kinematic";
            } else throw std::invalid_argument("Unknown option: " + argument);
        }
        if (duration <= 0 || speed <= 0) throw std::invalid_argument("Duration and speed must be positive");
        std::signal(SIGINT, requestStop);
        std::signal(SIGTERM, requestStop);

        dog::sim::Simulation simulation(modelPath, options);
        std::cout << "MuJoCo " << mj_versionString() << ", mode="
                  << (options.kinematic ? "kinematic (prescribed pose, fixed body)" : "dynamics (free body, PD torques)")
                  << ", mass=" << mju_sum(simulation.model()->body_mass, simulation.model()->nbody)
                  << " kg, period=" << options.gait.period << " s, kp=" << options.kp
                  << ", kd=" << options.kd << ", gravity_feedforward="
                  << (options.gravityFeedforward ? "on" : "off") << '\n';
        std::cout << "CAD-to-robot rotation (right/front/up):\n" << simulation.modelToRobot() << '\n';
        for (const auto& leg : simulation.legs()) for (const auto& branch : leg.branches) {
            std::cout << branch.name << " <- " << (branch.ikSign > 0 ? "theta1" : "theta2")
                      << ", active axis sign=" << branch.activeSign
                      << ", passive axis sign=" << branch.passiveSign << '\n';
        }

        std::ofstream csv;
        if (!csvPath.empty()) {
            csv.open(csvPath);
            if (!csv) throw std::runtime_error("Cannot open CSV: " + csvPath);
            csv << "time,phase,base_right,base_forward,base_up,upright,closure_error_m,"
                   "joint_tracking_rms_rad,max_torque_nm,ground_contacts,fallen,"
                   "max_feedforward_nm,desired_support_z_n\n";
            csv << std::setprecision(12);
        }
        double lastSample = -1, lastPrint = -1;
        bool everFallen = false;
        double maxClosure = 0, peakTorque = 0, peakFeedforward = 0;
        auto observe = [&]() {
            const double time = simulation.data()->time;
            if (time < lastSample) {
                lastSample = lastPrint = -1;
                everFallen = false;
                maxClosure = peakTorque = peakFeedforward = 0;
            }
            if (lastSample >= 0 && time-lastSample < 0.01-1e-9) return;
            lastSample = time;
            const auto m = simulation.metrics();
            everFallen = everFallen || m.fallen;
            maxClosure = std::max(maxClosure, m.closureError);
            peakTorque = std::max(peakTorque, m.maxTorque);
            peakFeedforward = std::max(peakFeedforward, m.maxFeedforwardTorque);
            if (csv.is_open()) {
                csv << time << ',' << simulation.phase() << ',' << m.base.x() << ',' << m.base.y()
                    << ',' << m.base.z() << ',' << m.upright << ',' << m.closureError << ','
                    << m.trackingError << ',' << m.maxTorque << ',' << m.groundContacts << ',' << m.fallen
                    << ',' << m.maxFeedforwardTorque << ',' << m.desiredSupportZ << '\n';
            }
            if (time-lastPrint >= 1-1e-9) {
                lastPrint = time;
                std::cout << std::fixed << std::setprecision(3) << "t=" << time
                          << " forward=" << m.base.y() << " z=" << m.base.z()
                          << " tracking=" << m.trackingError << " tau_ff=" << m.maxFeedforwardTorque
                          << " contacts=" << m.groundContacts
                          << " fallen=" << std::boolalpha << m.fallen << '\n';
            }
        };
        observe();
        if (headless) {
            while (running && simulation.data()->time < duration-1e-10) {
                simulation.step();
                observe();
            }
        } else {
#ifdef DOG_SIM_HAS_VIEWER
            dog::sim::runViewer(simulation, duration, speed, observe, [] { return running != 0; });
#else
            throw std::runtime_error("Viewer was not built; use --headless or install GLFW/OpenGL and rebuild");
#endif
        }
        if (!snapshotPath.empty()) simulation.saveSnapshot(snapshotPath);
        const auto final = simulation.metrics();
        std::cout << "Completed: time=" << simulation.data()->time
                  << ", forward_m=" << final.base.y() << ", ever_fallen=" << everFallen
                  << ", max_closure_m=" << std::scientific << maxClosure
                  << ", peak_torque_nm=" << peakTorque << ", peak_feedforward_nm=" << peakFeedforward << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
