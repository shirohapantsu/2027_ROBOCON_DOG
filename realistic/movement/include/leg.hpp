#pragma once

#include "motor.hpp"
#include <vector>
#include <cmath>

#define pi 3.141593

using FloatVector = std::vector<float>;
using FloatMatrix = std::vector<std::vector<float>>;

namespace leg {

enum class StepState {
    Swing,
    Support,
};

class leg {
private:

    motor::Motor motor1;    // 前腿电机
    motor::Motor motor2;    // 后腿电机
    float period_;   // 步态周期

    static constexpr float L1 = 0.135f;   // 大腿，单位m
    static constexpr float L2 = 0.260f;   // 小腿，单位m
    static constexpr float PACE_LENGHT = 0.14f; // 单步步长14cm
    static constexpr float STAND_Z = -0.274f;   // 行走腿长27.4cm
    static constexpr float MAX_HEIGHT = 0.35f;  // 最大腿长
    static constexpr float MIN_HEIGHT = 0.14f;  // 最小腿长
    static constexpr float STEP_HEIGHT = 0.03f;  // 迈步高度
    static constexpr float SUPPORT_RATIO = 0.6f;    // 支撑相占比

    // ik
    FloatVector ik(FloatVector &footPosYZ);
    // 计算电机角度和速度
    FloatMatrix getMotorPV(const FloatVector &footPosYZ, const FloatVector &footVelYZ);
    // 一次运动控制
    void to_footPos(FloatVector &footPosYZ, FloatVector &footVelYZ);
    // 摆动相，返回 {{y, z}, {dy, dz}}
    FloatMatrix step_swing(float u_sw, float T_sw, float T_sp);    // u为归一化时间
    // 支撑相，返回 {{y, z}, {dy, dz}}
    FloatMatrix step_support(float u_sp, float T_sp);

public:
    // 构造函数
    leg(float period,
    std::shared_ptr<SerialPort> serial, int id1, float offset1, bool direction1,
    int id2, float offset2, bool direction2);
    // 一步步态
    void onestep(StepState startstate, float u);
};
}
