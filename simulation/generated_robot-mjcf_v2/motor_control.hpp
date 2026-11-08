#pragma once

#include <mujoco/mujoco.h>
#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

// C++17 仿真电机适配器：将五参数命令转换为 MuJoCo 执行器输入。
// 本文件不打开串口；所有位置、速度、力矩命令默认采用转子侧物理量。
namespace dog::motor {

inline constexpr int kMotorCount = 8;
inline constexpr int kCurvePointCount = 12;
using MotorVector = Eigen::Matrix<double, kMotorCount, 1>;
using MotorIndexVector = Eigen::Matrix<int, kMotorCount, 1>;
using TorqueSpeedMatrix = Eigen::Matrix<double, kCurvePointCount, 2, Eigen::RowMajor>;

inline constexpr double kGearRatio = 6.33;
// 补充参数表给出的最大值，不代表连续工作额定值。
inline constexpr double kPeakOutputTorque = 23.7;
inline constexpr double kMaxPhaseCurrent = 40.0;
// 扭矩常数暂按输出轴侧解释；由此折算的是相电流，不是直流母线电流。
inline constexpr double kOutputTorqueConstant = 0.63895;
inline constexpr double kCurrentLimitedOutputTorque = kMaxPhaseCurrent * kOutputTorqueConstant;
inline constexpr double kOutputTorqueLimit =
    kPeakOutputTorque < kCurrentLimitedOutputTorque ? kPeakOutputTorque : kCurrentLimitedOutputTorque;
inline constexpr double kRotorTorqueLimit = kOutputTorqueLimit / kGearRatio;
inline constexpr double kOutputSpeedLimit = 30.0;  // 仅对应标称 24 V 工况。
inline constexpr double kRotorSpeedLimit = kOutputSpeedLimit * kGearRatio;
inline constexpr double kNominalVoltage = 24.0;
inline constexpr double kPi = 3.14159265358979323846;

// 由原图绿色转速曲线近似读取得到；矩阵每行依次为转子力矩（N·m）、转速（rpm）。
// 第 0 行是由输出侧 30 rad/s 参数补充的零力矩锚点，不是原图采样点。
// 转子轴侧与 24 V 工况为推断；这组数据不表示连续热额定能力。
inline const TorqueSpeedMatrix kTorqueSpeedCurve = [] {
    TorqueSpeedMatrix curve;
    curve << 0.0, kRotorSpeedLimit * 60.0 / (2.0 * kPi),
             0.2, 1700, 0.4, 1660, 0.6, 1620, 0.8, 1580, 1.0, 1550,
             1.5, 1440, 2.0, 1340, 2.5, 1240, 3.0, 1130, 3.5, 1030, 3.98, 930;
    return curve;
}();
// 名称是非数值标签，仍用字符串数组保存；顺序对应所有八维向量的分量。
inline constexpr std::array<const char*, kMotorCount> kNames = {
    "RR_thigh_rear", "RR_thigh_front", "FR_thigh_front", "FR_thigh_rear",
    "FL_thigh_rear", "FL_thigh_front", "RL_thigh_front", "RL_thigh_rear"};

// 单台电机命令，与 GO-M8010-6 SDK 的 MotorCmd 使用相同的转子侧单位。
// q=0 表示 CAD 初始姿态，不是实机编码器的上电零位。
struct RotorCommand {
    double tau = 0.0;  // 前馈力矩，N·m。
    double q = 0.0;    // 目标角度，rad。
    double dq = 0.0;   // 目标角速度，rad/s。
    double kp = 0.0;   // 位置增益，N·m/rad。
    double kd = 0.0;   // 速度增益，N·m·s/rad。
};

// 一帧八电机命令：五个固定尺寸 Eigen 列向量，每个分量对应一台电机。
// 显式零初始化，避免 Eigen 默认构造时数值未初始化。
struct RotorCommands {
    MotorVector tau = MotorVector::Zero();
    MotorVector q = MotorVector::Zero();
    MotorVector dq = MotorVector::Zero();
    MotorVector kp = MotorVector::Zero();
    MotorVector kd = MotorVector::Zero();

    static constexpr int size() { return kMotorCount; }

    // 将单台电机的标量命令写入对应的向量分量，供 SDK 等标量接口使用。
    void set(Eigen::Index index, const RotorCommand& command) {
        if (index < 0 || index >= kMotorCount) throw std::out_of_range("Invalid motor index");
        tau[index] = command.tau;
        q[index] = command.q;
        dq[index] = command.dq;
        kp[index] = command.kp;
        kd[index] = command.kd;
    }

    // 对全部电机设置相同命令；也可以直接操作五个 Eigen 向量。
    void fill(const RotorCommand& command) {
        tau.setConstant(command.tau);
        q.setConstant(command.q);
        dq.setConstant(command.dq);
        kp.setConstant(command.kp);
        kd.setConstant(command.kd);
    }
};

struct RotorFeedback {
    double q;   // 转子角度，rad。
    double dq;  // 转子角速度，rad/s。
    double tau; // 减速器及关节摩擦作用前的转子驱动力矩，不是净关节力矩。
    double estimated_phase_current;  // 按 tau * 减速比 / 输出侧扭矩常数折算，非实测信号。
};

// 批量反馈采用与命令相同的八维列向量顺序。
struct RotorFeedbacks {
    MotorVector q = MotorVector::Zero();
    MotorVector dq = MotorVector::Zero();
    MotorVector tau = MotorVector::Zero();
    MotorVector estimated_phase_current = MotorVector::Zero();
};

inline void validate(const RotorCommand& command) {
    if (!std::isfinite(command.tau) || !std::isfinite(command.q) ||
        !std::isfinite(command.dq) || !std::isfinite(command.kp) ||
        !std::isfinite(command.kd) || command.kp < 0 || command.kd < 0) {
        throw std::invalid_argument("Motor commands must be finite; gains must be nonnegative");
    }
}

// 校验完整命令帧；只有全部分量通过检查后，才允许写入仿真控制量。
inline void validate(const RotorCommands& commands) {
    if (!commands.tau.allFinite() || !commands.q.allFinite() || !commands.dq.allFinite() ||
        !commands.kp.allFinite() || !commands.kd.allFinite() ||
        (commands.kp.array() < 0).any() || (commands.kd.array() < 0).any()) {
        throw std::invalid_argument("Motor commands must be finite; gains must be nonnegative");
    }
}

// 将单台电机的输出轴侧目标和 PD 增益换算为 SDK 使用的转子侧物理量。
inline RotorCommand fromOutput(double tau, double q, double dq,
                               double kp, double kd) {
    RotorCommand command{tau / kGearRatio, q * kGearRatio, dq * kGearRatio,
                         kp / (kGearRatio * kGearRatio),
                         kd / (kGearRatio * kGearRatio)};
    validate(command);
    return command;
}

// 批量换算八台电机：力矩除以减速比，位置和速度乘以减速比，增益除以减速比平方。
inline RotorCommands fromOutput(const MotorVector& tau, const MotorVector& q,
                                const MotorVector& dq, const MotorVector& kp,
                                const MotorVector& kd) {
    RotorCommands commands;
    commands.tau = tau / kGearRatio;
    commands.q = q * kGearRatio;
    commands.dq = dq * kGearRatio;
    commands.kp = kp / (kGearRatio * kGearRatio);
    commands.kd = kd / (kGearRatio * kGearRatio);
    validate(commands);
    return commands;
}

// 按当前转速反向插值绿色负载曲线，并施加峰值力矩上限。
// 将测试工况曲线作为最大驱动包络使用，是当前模型的近似假设。
inline double availableRotorDriveTorque(double rotorSpeed) {
    if (!std::isfinite(rotorSpeed)) throw std::invalid_argument("Nonfinite motor speed");
    const double rpm = std::abs(rotorSpeed) * 60.0 / (2.0 * kPi);
    if (rpm >= kTorqueSpeedCurve(0, 1)) return 0.0;
    for (Eigen::Index i = 1; i < kTorqueSpeedCurve.rows(); ++i) {
        const auto fast = kTorqueSpeedCurve.row(i - 1);
        const auto slow = kTorqueSpeedCurve.row(i);
        if (rpm >= slow[1]) {
            const double fraction = (fast[1] - rpm) / (fast[1] - slow[1]);
            return std::min(kRotorTorqueLimit,
                            fast[0] + fraction * (slow[0] - fast[0]));
        }
    }
    return kRotorTorqueLimit;
}

// 与旋转方向相同的力矩受驱动包络限制；制动力矩仅受峰值力矩/等效相电流限制。
// 原图没有再生制动曲线，因此不将正向驱动曲线套用于制动工况。
// 本函数不会硬截断关节速度，外载荷仍可能使转轴超速。
inline double limitRotorTorque(double torque, double rotorSpeed) {
    if (!std::isfinite(torque) || !std::isfinite(rotorSpeed)) {
        throw std::invalid_argument("Motor torque and speed must be finite");
    }
    torque = std::clamp(torque, -kRotorTorqueLimit, kRotorTorqueLimit);
    if ((rotorSpeed > 0 && torque > 0) || (rotorSpeed < 0 && torque < 0)) {
        const double available = availableRotorDriveTorque(rotorSpeed);
        torque = std::clamp(torque, -available, available);
    }
    return torque;
}

class HipMotorController {
public:
    explicit HipMotorController(const mjModel* model) {
        if (!model || model->nu != static_cast<int>(kNames.size())) {
            throw std::invalid_argument("Expected the eight-motor dog model");
        }
        for (std::size_t i = 0; i < kNames.size(); ++i) {
            const std::string stem(kNames[i]);
            const int actuator = mj_name2id(model, mjOBJ_ACTUATOR, (stem + "_motor").c_str());
            const int joint = mj_name2id(model, mjOBJ_JOINT, (stem + "_joint").c_str());
            if (actuator < 0 || joint < 0 || model->jnt_type[joint] != mjJNT_HINGE ||
                model->actuator_trntype[actuator] != mjTRN_JOINT ||
                model->actuator_trnid[2 * actuator] != joint ||
                std::abs(model->actuator_gear[6 * actuator] - kGearRatio) > 1e-9) {
                throw std::invalid_argument("Invalid hip motor mapping: " + stem);
            }
            // 检查 XML 与控制器限幅是否一致，避免新旧电机配置混用。
            if (!model->actuator_ctrllimited[actuator] || !model->actuator_forcelimited[actuator] ||
                !model->jnt_actfrclimited[joint] ||
                std::abs(model->actuator_ctrlrange[2 * actuator] + kRotorTorqueLimit) > 1e-9 ||
                std::abs(model->actuator_ctrlrange[2 * actuator + 1] - kRotorTorqueLimit) > 1e-9 ||
                std::abs(model->actuator_forcerange[2 * actuator] + kRotorTorqueLimit) > 1e-9 ||
                std::abs(model->actuator_forcerange[2 * actuator + 1] - kRotorTorqueLimit) > 1e-9 ||
                std::abs(model->jnt_actfrcrange[2 * joint] + kOutputTorqueLimit) > 1e-9 ||
                std::abs(model->jnt_actfrcrange[2 * joint + 1] - kOutputTorqueLimit) > 1e-9) {
                throw std::invalid_argument("XML/controller torque limits differ: " + stem);
            }
            actuators_[i] = actuator;
            qpos_[i] = model->jnt_qposadr[joint];
            dofs_[i] = model->jnt_dofadr[joint];
        }
    }

    // 在一帧内部完成校验、向量 PD 运算和逐电机非线性限幅，再统一写入 ctrl。
    void apply(mjData* data, const RotorCommands& commands) const {
        if (!data) throw std::invalid_argument("Null simulation data");
        validate(commands);
        MotorVector positions;
        MotorVector velocities;
        // 主动关节在 MuJoCo 状态中并不连续，先按索引采集到 Eigen 向量。
        for (int i = 0; i < kMotorCount; ++i) {
            positions[i] = data->qpos[qpos_[i]];
            velocities[i] = data->qvel[dofs_[i]];
        }
        const MotorVector rotorSpeed = kGearRatio * velocities;
        const MotorVector targetSpeed = commands.dq.cwiseMax(-kRotorSpeedLimit).cwiseMin(kRotorSpeedLimit);
        // 各电机使用自身的增益，cwiseProduct 表示对应分量相乘。
        MotorVector torques = commands.tau +
            commands.kp.cwiseProduct(commands.q - kGearRatio * positions) +
            commands.kd.cwiseProduct(targetSpeed - rotorSpeed);
        if (!torques.allFinite()) throw std::invalid_argument("Motor torque overflow");
        for (int i = 0; i < kMotorCount; ++i) {
            torques[i] = limitRotorTorque(torques[i], rotorSpeed[i]);
        }
        for (int i = 0; i < kMotorCount; ++i) {
            data->ctrl[actuators_[i]] = torques[i];
        }
    }

    // 修改状态或控制量后应先调用 mj_forward；也可在 mj_step 后读取反馈。
    RotorFeedbacks feedback(const mjData* data) const {
        if (!data) throw std::invalid_argument("Null simulation data");
        RotorFeedbacks result;
        for (int i = 0; i < kMotorCount; ++i) {
            result.q[i] = data->qpos[qpos_[i]];
            result.dq[i] = data->qvel[dofs_[i]];
            result.tau[i] = data->actuator_force[actuators_[i]];
        }
        result.q *= kGearRatio;
        result.dq *= kGearRatio;
        result.estimated_phase_current = result.tau * (kGearRatio / kOutputTorqueConstant);
        return result;
    }

    // 保留单台电机的标量读取接口，便于对接 SDK 的 MotorData。
    RotorFeedback feedback(const mjData* data, std::size_t index) const {
        if (!data || index >= kNames.size()) throw std::out_of_range("Invalid motor feedback");
        return {kGearRatio * data->qpos[qpos_[index]],
                kGearRatio * data->qvel[dofs_[index]],
                data->actuator_force[actuators_[index]],
                data->actuator_force[actuators_[index]] * kGearRatio / kOutputTorqueConstant};
    }

private:
    MotorIndexVector actuators_ = MotorIndexVector::Zero();  // 执行器索引。
    MotorIndexVector qpos_ = MotorIndexVector::Zero();       // 关节位置在 qpos 中的索引。
    MotorIndexVector dofs_ = MotorIndexVector::Zero();        // 关节速度在 qvel 中的索引。
};

}  // 电机仿真命名空间。
