#pragma once

#include "motor.hpp"
#include <cmath>
#include <vector>

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
    static constexpr float L1 = 0.135f;   // 大腿，单位m
    static constexpr float L2 = 0.260f;   // 小腿，单位m
    static constexpr float PACE_LENGHT = 0.14f; // 单步步长14cm
    static constexpr float STAND_Z = -0.274f;   // 行走腿长27.4cm
    static constexpr float MAX_HEIGHT = 0.35f;  // 最大腿长
    static constexpr float MIN_HEIGHT = 0.14f;  // 最小腿长
    static constexpr float STEP_HEIGHT = 0.03f;  // 迈步高度

    // ik
    FloatVector ik(FloatVector &footPos);
    // 计算电机角度和速度
    FloatMatrix getMotorPV(const FloatVector &footPos, const FloatVector &footVel);

public:
    leg();
    // 一次运动控制
    void to_footPos(FloatVector &footPos, FloatVector &footVel);
    // 摆动相
    FloatMatrix step_swing(float u_sw, float T_sw, float T_sp);    // u为归一化时间
    // 支撑相
    FloatMatrix step_support(float u_sp, float T_sp);
};
}
