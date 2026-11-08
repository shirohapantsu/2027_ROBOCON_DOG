#include "motor_control.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
using Model = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using Data = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const std::string& message, double tolerance = 1e-9) {
    require(std::abs(actual - expected) < tolerance,
            message + ": actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected));
}

Model load(const char* path, const mjVFS* vfs = nullptr) {
    char error[2048]{};
    Model model(mj_loadXML(path, vfs, error, sizeof(error)), mj_deleteModel);
    require(model != nullptr, error);
    return model;
}

// Isolate gearbox + friction from gravity, linkage loads and mesh contacts.
// All motor/solver parameters below are copied from the compiled robot model.
Model frictionRig(const mjModel* robot, int actuator, int dof) {
    const std::string xml = R"(<mujoco>
      <worldbody><body><joint name="shaft" type="hinge" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      </body></worldbody>
      <actuator><motor joint="shaft"/></actuator>
    </mujoco>)";
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    require(mj_addBufferVFS(&vfs, "friction.xml", xml.data(), static_cast<int>(xml.size())) == 0,
            "Cannot construct friction fixture");
    auto model = load("friction.xml", &vfs);
    mj_deleteVFS(&vfs);
    model->opt = robot->opt;
    mju_zero(model->opt.gravity, 3);
    model->dof_frictionloss[0] = robot->dof_frictionloss[dof];
    mju_copy(model->dof_solref, robot->dof_solref + mjNREF * dof, mjNREF);
    mju_copy(model->dof_solimp, robot->dof_solimp + mjNIMP * dof, mjNIMP);
    mju_copy(model->actuator_gear, robot->actuator_gear + 6 * actuator, 6);
    model->actuator_ctrllimited[0] = robot->actuator_ctrllimited[actuator];
    model->actuator_forcelimited[0] = robot->actuator_forcelimited[actuator];
    mju_copy(model->actuator_ctrlrange, robot->actuator_ctrlrange + 2 * actuator, 2);
    mju_copy(model->actuator_forcerange, robot->actuator_forcerange + 2 * actuator, 2);
    return model;
}

void checkFriction(const mjModel* robot, int actuator, int dof) {
    auto model = frictionRig(robot, actuator, dof);
    Data data(mj_makeData(model.get()), mj_deleteData);
    for (double sign : {-1.0, 1.0}) {
        for (double magnitude : {0.0, 0.019, 0.024, 0.026, 0.031}) {
            mj_resetData(model.get(), data.get());
            data->ctrl[0] = sign * magnitude;
            for (int step = 0; step < 200; ++step) mj_step(model.get(), data.get());
            const double speed = sign * data->qvel[0];
            if (magnitude < 0.025) {
                require(std::abs(data->qvel[0]) < 0.002 && std::abs(data->qpos[0]) < 0.0005,
                        "Below-threshold friction drift exceeds solver tolerance");
            } else {
                require(speed > 0.05, "Above-threshold command must move the unloaded shaft");
            }
        }
        // Friction must also decelerate a moving unpowered shaft.
        mj_resetData(model.get(), data.get());
        data->qvel[0] = sign * 0.5;
        for (int step = 0; step < 200; ++step) mj_step(model.get(), data.get());
        require(std::abs(data->qvel[0]) < 0.002, "Zero command must retain passive friction");
    }
}

void checkModel(mjModel* model, mjData* data) {
    using namespace dog::motor;
    require(model->nu == 8 && model->nsensor == 24 && model->neq == 4,
            "Expected eight motors, 24 feedback channels and four linkage closures");
    require(model->nq == 23 && model->nv == 22 && model->njnt == 17,
            "Free base and 16 hinge joints must be preserved");
    near(mj_getTotalmass(model), 9.36, "Confirmed motor mass must not be counted twice");
    const int torqueCurve = mj_name2id(model, mjOBJ_NUMERIC, "motor_curve_rotor_torque");
    const int speedCurve = mj_name2id(model, mjOBJ_NUMERIC, "motor_curve_rotor_rpm");
    require(torqueCurve >= 0 && speedCurve >= 0, "Missing XML performance curve metadata");
    require(model->numeric_size[torqueCurve] == static_cast<int>(kTorqueSpeedCurve.rows()) &&
            model->numeric_size[speedCurve] == static_cast<int>(kTorqueSpeedCurve.rows()),
            "XML/C++ performance curve lengths differ");
    for (Eigen::Index i = 0; i < kTorqueSpeedCurve.rows(); ++i) {
        near(model->numeric_data[model->numeric_adr[torqueCurve] + i],
             kTorqueSpeedCurve(i, 0), "XML/C++ curve torque agreement");
        near(model->numeric_data[model->numeric_adr[speedCurve] + i],
             kTorqueSpeedCurve(i, 1), "XML/C++ curve speed agreement");
    }
    int passive = 0;
    for (int joint = 0; joint < model->njnt; ++joint) {
        const char* name = mj_id2name(model, mjOBJ_JOINT, joint);
        if (!name || std::string(name).find("_calf_") == std::string::npos) continue;
        ++passive;
        near(model->dof_frictionloss[model->jnt_dofadr[joint]], 0, "Passive calf friction");
        require(!model->jnt_actfrclimited[joint], "Passive calf has an actuator force limit");
        for (int actuator = 0; actuator < model->nu; ++actuator) {
            require(model->actuator_trnid[2 * actuator] != joint, "Calf must not be actuated");
        }
    }
    require(passive == 8, "Expected eight passive calf joints");
    HipMotorController controller(model);
    RotorCommands commands;
    for (int i = 0; i < commands.size(); ++i) commands.tau[i] = (i % 2 ? -0.03 : 0.03);
    controller.apply(data, commands);
    mj_forward(model, data);
    for (int i = 0; i < commands.size(); ++i) {
        const std::string stem(kNames[i]);
        const int actuator = mj_name2id(model, mjOBJ_ACTUATOR, (stem + "_motor").c_str());
        const int joint = mj_name2id(model, mjOBJ_JOINT, (stem + "_joint").c_str());
        const int dof = model->jnt_dofadr[joint];
        near(model->dof_frictionloss[dof], 0.15825, "Rotor/output friction conversion");
        near(model->jnt_actfrcrange[2 * joint], -23.7, "Joint lower torque limit");
        near(model->jnt_actfrcrange[2 * joint + 1], 23.7, "Joint upper torque limit");
        near(model->actuator_ctrlrange[2 * actuator + 1], kRotorTorqueLimit, "Control limit");
        near(model->actuator_forcerange[2 * actuator + 1], kRotorTorqueLimit, "Force limit");
        near(data->qfrc_actuator[dof], kGearRatio * commands.tau[i], "Gear torque conversion");
        near(controller.feedback(data, i).tau, commands.tau[i], "Rotor feedback");
        near(controller.feedback(data, i).estimated_phase_current,
             kGearRatio * commands.tau[i] / 0.63895, "Phase current estimate");
        for (const std::string suffix : {"_position", "_velocity", "_rotor_torque"}) {
            const int sensor = mj_name2id(model, mjOBJ_SENSOR, (stem + suffix).c_str());
            require(sensor >= 0 && model->sensor_dim[sensor] == 1, "Missing motor sensor");
            const double expected = suffix == "_rotor_torque" ? commands.tau[i] : 0;
            near(data->sensordata[model->sensor_adr[sensor]], expected, "Sensor value");
        }
        checkFriction(model, actuator, dof);
    }
    std::cout << "PASS: eight hip motors, eight passive calves, sensors, gearbox, friction (+/-)\n";

    // 用每台电机不同的目标、增益和状态，检查 Eigen 运算没有混合电机分量。
    const MotorVector outputTau = MotorVector::LinSpaced(-0.1, 0.1);
    const MotorVector outputQ = MotorVector::LinSpaced(-0.03, 0.04);
    const MotorVector outputDq = MotorVector::LinSpaced(-0.2, 0.15);
    const MotorVector outputKp = MotorVector::LinSpaced(1.0, 4.0);
    const MotorVector outputKd = MotorVector::LinSpaced(0.1, 0.4);
    const RotorCommands vectorCommands = fromOutput(outputTau, outputQ, outputDq, outputKp, outputKd);
    for (int i = 0; i < kMotorCount; ++i) {
        const int joint = mj_name2id(model, mjOBJ_JOINT, (std::string(kNames[i]) + "_joint").c_str());
        data->qpos[model->jnt_qposadr[joint]] = 0.002 * (i + 1);
        data->qvel[model->jnt_dofadr[joint]] = -0.01 * (i + 1);
    }
    controller.apply(data, vectorCommands);
    mj_forward(model, data);
    const RotorFeedbacks vectorFeedback = controller.feedback(data);
    for (int i = 0; i < kMotorCount; ++i) {
        const double q = 0.002 * (i + 1);
        const double dq = -0.01 * (i + 1);
        const double expected = (outputTau[i] + outputKp[i] * (outputQ[i] - q) +
                                 outputKd[i] * (outputDq[i] - dq)) / kGearRatio;
        near(vectorFeedback.tau[i], expected, "Eigen per-motor PD equivalence");
        near(vectorFeedback.q[i], q * kGearRatio, "Eigen position gather mapping");
        near(vectorFeedback.dq[i], dq * kGearRatio, "Eigen velocity gather mapping");
        near(vectorFeedback.estimated_phase_current[i], expected * kGearRatio / kOutputTorqueConstant,
             "Eigen estimated current vector");
        near(vectorFeedback.tau[i], controller.feedback(data, i).tau, "Batch/scalar feedback agreement");
    }
    mj_resetData(model, data);
    std::cout << "PASS: Eigen batch conversion, distinct per-motor PD targets/gains and vector feedback\n";

    for (double sign : {-1.0, 1.0}) {
        commands.fill({sign * 100, 0, 0, 0, 0});
        controller.apply(data, commands);
        mj_forward(model, data);
        for (int actuator = 0; actuator < model->nu; ++actuator) {
            near(data->actuator_force[actuator], sign * kRotorTorqueLimit, "Hybrid saturation");
            // Also verify MuJoCo's own clamp for direct writes to ctrl.
            data->ctrl[actuator] = sign * 100;
        }
        mj_forward(model, data);
        for (int actuator = 0; actuator < model->nu; ++actuator) {
            const int joint = model->actuator_trnid[2 * actuator];
            near(data->qfrc_actuator[model->jnt_dofadr[joint]], sign * 23.7, "Native saturation");
        }
    }
    commands.fill(fromOutput(0.1, 0.02, 0.3, 2.0, 0.2));
    controller.apply(data, commands);
    mj_forward(model, data);
    near(controller.feedback(data, 0).tau * kGearRatio, 0.2, "Hybrid output PD equivalence");
    const double previous = data->ctrl[0];
    commands.tau[7] = std::numeric_limits<double>::quiet_NaN();
    bool rejected = false;
    try { controller.apply(data, commands); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Nonfinite commands must be rejected");
    near(data->ctrl[0], previous, "Invalid frames must not partially write ctrl");
    std::cout << "PASS: hybrid PD, rotor/output gains, saturation and invalid-command rejection\n";

    near(kRotorTorqueLimit, 23.7 / 6.33, "Peak torque conversion");
    near(kRotorSpeedLimit, 189.9, "Peak speed conversion");
    near(kOutputTorqueLimit / kOutputTorqueConstant, 37.092104233508095,
         "Peak equivalent phase current");
    require(kOutputTorqueLimit / kOutputTorqueConstant <= 40.0, "Phase current exceeds rating");
    for (double sign : {-1.0, 1.0}) {
        near(limitRotorTorque(sign * 100, 0), sign * kRotorTorqueLimit, "Stall peak torque cap");
        near(limitRotorTorque(sign * 100, sign * 1340 * 2 * kPi / 60), sign * 2.0,
             "Digitized speed/load curve knot");
        near(limitRotorTorque(sign * 100, sign * 1390 * 2 * kPi / 60), sign * 1.75,
             "Interpolated speed/load curve interval");
        near(limitRotorTorque(-sign * 100, sign * 1340 * 2 * kPi / 60), -sign * kRotorTorqueLimit,
             "Braking must not use motoring curve");
        require(std::abs(limitRotorTorque(sign * 100, sign * (kRotorSpeedLimit - 0.1))) < 0.01,
                "Near-no-load drive torque must taper toward zero");
        double previousLimit = kRotorTorqueLimit;
        for (int speedIndex = 0; speedIndex <= 310; ++speedIndex) {
            const double outputSpeed = speedIndex * 0.1;
            const double limit = availableRotorDriveTorque(sign * outputSpeed * kGearRatio);
            require(limit >= 0 && limit <= previousLimit + 1e-12,
                    "Drive torque envelope must be nonnegative and monotone");
            previousLimit = limit;
        }
        mj_resetData(model, data);
        for (std::size_t i = 0; i < kNames.size(); ++i) {
            const std::string jointName = std::string(kNames[i]) + "_joint";
            const int joint = mj_name2id(model, mjOBJ_JOINT, jointName.c_str());
            data->qvel[model->jnt_dofadr[joint]] = sign * 1340 * 2 * kPi / (60 * kGearRatio);
            commands.set(i, {sign * 100, 0, 0, 0, 0});
        }
        controller.apply(data, commands);
        mj_forward(model, data);
        for (std::size_t i = 0; i < kNames.size(); ++i) {
            near(controller.feedback(data, i).tau, sign * 2.0, "Eight-motor drive curve application");
        }
        for (double speed : {30.0, 31.0}) {
            mj_resetData(model, data);
            for (std::size_t i = 0; i < kNames.size(); ++i) {
                const std::string jointName = std::string(kNames[i]) + "_joint";
                const int joint = mj_name2id(model, mjOBJ_JOINT, jointName.c_str());
                data->qvel[model->jnt_dofadr[joint]] = sign * speed;
                commands.set(i, {sign * 100, 0, 0, 0, 0});
            }
            controller.apply(data, commands);
            mj_forward(model, data);
            for (int actuator = 0; actuator < model->nu; ++actuator) {
                near(data->actuator_force[actuator], 0, "Overspeed same-direction drive disabled");
            }
            commands.tau.setConstant(-sign * 100);
            controller.apply(data, commands);
            mj_forward(model, data);
            for (std::size_t i = 0; i < kNames.size(); ++i) {
                near(controller.feedback(data, i).tau, -sign * kRotorTorqueLimit,
                     "Overspeed braking torque retained");
                require(std::abs(controller.feedback(data, i).estimated_phase_current) <= 40,
                        "Overspeed braking exceeds phase-current ceiling");
            }
        }
        mj_resetData(model, data);
        commands.fill({0, 0, sign * 1000, 0, 0.001});
        controller.apply(data, commands);
        mj_forward(model, data);
        for (std::size_t i = 0; i < kNames.size(); ++i) {
            near(controller.feedback(data, i).tau, sign * 189.9 * 0.001,
                 "Rotor speed setpoint clamp");
        }
    }
    std::cout << "PASS: 23.7 Nm peak cap, phase-current estimate, +/- digitized torque-speed curve, braking\n";

    mj_resetData(model, data);
    mj_forward(model, data);
    double closureError = 0;
    for (int eq = 0; eq < model->neq; ++eq) {
        const auto* site1 = data->site_xpos + 3 * model->eq_obj1id[eq];
        const auto* site2 = data->site_xpos + 3 * model->eq_obj2id[eq];
        mjtNum residual[3];
        mju_sub3(residual, site1, site2);
        closureError = std::max(closureError, mju_norm3(residual));
    }
    std::cout << "Home max closure distance (m): " << closureError
              << "; mesh contacts: " << data->ncon << '\n';
    // This checks finite numerical evolution, not standing/gait stability.
    for (int step = 0; step < 1000; ++step) {
        mj_step(model, data);
        for (int q = 0; q < model->nq; ++q) require(std::isfinite(data->qpos[q]), "Nonfinite qpos");
        for (int v = 0; v < model->nv; ++v) require(std::isfinite(data->qvel[v]), "Nonfinite qvel");
    }
    for (int warning = 0; warning < mjNWARNING; ++warning) {
        require(data->warning[warning].number == 0, "MuJoCo warning during numerical smoke test");
    }
    near(data->time, 1.0, "Simulation time after 1000 steps", 1e-8);
    std::cout << "PASS: one-second free-body numerical smoke test (no ground/gait test)\n";
}
}  // namespace

int main(int argc, char** argv) {
    try {
        const char* path = argc > 1 ? argv[1] : "simulation/generated_robot-mjcf_v2/robot.xml";
        auto model = load(path);
        Data data(mj_makeData(model.get()), mj_deleteData);
        require(data != nullptr, "Cannot allocate MuJoCo data");
        std::cout << "MuJoCo " << mj_versionString() << '\n';
        checkModel(model.get(), data.get());
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
