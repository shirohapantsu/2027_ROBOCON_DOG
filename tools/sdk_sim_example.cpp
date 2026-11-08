#include "motor_control.hpp"
#include "unitreeMotor/unitreeMotor.h"

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

// Example-local simulation backend. This is not a Unitree SDK class.
// IDs 0..7 are virtual IDs in the order dog::motor::kNames, not hardware IDs.
class SimMotorBus {
public:
    SimMotorBus(const mjModel* model, mjData* data)
        : model_(model), data_(data), controller_(model),
          foc_(queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC)) {
        if (!data || std::abs(queryGearRatio(MotorType::GO_M8010_6) -
                              dog::motor::kGearRatio) > 1e-5) {
            throw std::runtime_error("Invalid simulation data or SDK/model gear ratio mismatch");
        }
    }

    void step(const std::array<MotorCmd, 8>& commands, std::array<MotorData, 8>& feedback) {
        dog::motor::RotorCommands frame;
        std::array<bool, 8> seen{};
        for (const auto& command : commands) {
            if (command.motorType != MotorType::GO_M8010_6 || command.mode != foc_ ||
                command.id >= 8 || seen[command.id]) {
                throw std::invalid_argument("Expected eight unique virtual IDs 0..7 in GO FOC mode");
            }
            seen[command.id] = true;
            frame.set(command.id, {command.tau, command.q, command.dq, command.kp, command.kd});
        }
        // One complete eight-motor frame advances the world by ONE timestep.
        controller_.apply(data_, frame);
        mj_step(model_, data_);
        mj_forward(model_, data_);  // Refresh sensors at the updated state without advancing time.

        const auto rotors = controller_.feedback(data_);
        for (std::size_t id = 0; id < feedback.size(); ++id) {
            auto& result = feedback[id];
            result.motorType = MotorType::GO_M8010_6;
            result.motor_id = static_cast<unsigned char>(id);
            result.mode = static_cast<unsigned char>(foc_);
            result.q = static_cast<float>(rotors.q[id]);
            result.dq = static_cast<float>(rotors.dq[id]);
            result.tau = static_cast<float>(rotors.tau[id]);
            result.correct = true;  // Simulation step succeeded; no serial/CRC check occurred.
            result.merror = 0;      // Placeholder: hardware faults are not simulated.
            result.hex_len = 0;    // No feedback packet was generated.
            result.temp = result.footForce = result.Acc = 0;  // Unmodelled placeholders.
            result.LW = 0;
            for (int axis = 0; axis < 3; ++axis) result.gyro[axis] = result.acc[axis] = 0;
        }
    }

private:
    const mjModel* model_;
    mjData* data_;
    dog::motor::HipMotorController controller_;
    int foc_;
};

int main(int argc, char** argv) {
    try {
        const char* path = argc > 1 ? argv[1] : "simulation/generated_robot-mjcf_v2/robot.xml";
        char error[2048]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(path, nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) throw std::runtime_error(error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        if (!data) throw std::runtime_error("Cannot allocate simulation state");

        // A zero-gravity bench demonstration, not a standing/gait controller.
        // This changes only this process's model; robot.xml is never rewritten.
        mju_zero(model->opt.gravity, 3);
        const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
        if (home < 0) throw std::runtime_error("Missing home keyframe");
        mj_resetDataKeyframe(model.get(), data.get(), home);
        mj_forward(model.get(), data.get());
        SimMotorBus bus(model.get(), data.get());
        std::array<MotorCmd, 8> commands;
        std::array<MotorData, 8> feedback;
        for (std::size_t id = 0; id < commands.size(); ++id) {
            auto& command = commands[id];
            // The SDK constructors are empty: explicitly initialize command fields.
            command.motorType = MotorType::GO_M8010_6;
            command.id = static_cast<unsigned short>(id);
            command.mode = static_cast<unsigned short>(
                queryMotorMode(command.motorType, MotorMode::FOC));
            command.hex_len = 0;
            command.Res.L = 0;
            command.tau = command.q = command.dq = command.kp = command.kd = 0;
        }

        std::cout << "Zero-gravity simulation; virtual ID 0 = RR_thigh_rear; SDK gear="
                  << queryGearRatio(MotorType::GO_M8010_6) << '\n';
        std::cout << "time_s,target_output_rad,measured_output_rad,output_rad_s,rotor_torque_Nm\n";
        const int steps = static_cast<int>(std::lround(2.0 / model->opt.timestep));
        for (int step = 0; step < steps; ++step) {
            const double t = data->time;
            for (std::size_t id = 0; id < commands.size(); ++id) {
                // ID 0 follows a smooth 0 -> 0.08 -> 0 rad target over two seconds;
                // other hips hold CAD home. These are output-side targets/gains.
                const double q = id == 0 ? 0.04 * (1 - std::cos(dog::motor::kPi * t)) : 0;
                const double dq = id == 0 ? 0.04 * dog::motor::kPi * std::sin(dog::motor::kPi * t) : 0;
                const auto rotor = dog::motor::fromOutput(0, q, dq, 8.0, 0.4);
                commands[id].tau = static_cast<float>(rotor.tau);
                commands[id].q = static_cast<float>(rotor.q);
                commands[id].dq = static_cast<float>(rotor.dq);
                commands[id].kp = static_cast<float>(rotor.kp);
                commands[id].kd = static_cast<float>(rotor.kd);
            }
            bus.step(commands, feedback);
            if (step % 200 == 0 || step + 1 == steps) {
                const auto& rotor = feedback[0];
                std::cout << std::fixed << std::setprecision(6) << data->time << ','
                          << commands[0].q / dog::motor::kGearRatio << ','
                          << rotor.q / dog::motor::kGearRatio << ','
                          << rotor.dq / dog::motor::kGearRatio << ',' << rotor.tau << '\n';
            }
            for (int q = 0; q < model->nq; ++q) {
                if (!std::isfinite(data->qpos[q])) throw std::runtime_error("Nonfinite simulation position");
            }
            for (int v = 0; v < model->nv; ++v) {
                if (!std::isfinite(data->qvel[v])) throw std::runtime_error("Nonfinite simulation velocity");
            }
        }
        for (int warning = 0; warning < mjNWARNING; ++warning) {
            if (data->warning[warning].number) throw std::runtime_error("MuJoCo warning during example");
        }
        if (std::abs(data->time - steps * model->opt.timestep) > 1e-8) {
            throw std::runtime_error("Simulation time mismatch");
        }
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
