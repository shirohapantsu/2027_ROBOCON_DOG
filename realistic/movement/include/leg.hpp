#pragma once

#include "motor.hpp"
#include <cmath>
#include <vector>

#define pi 3.141593

using FloatVector = std::vector<float>;

namespace leg {

class leg {
private:
    enum class StepState {
        Swing,
        Support,
    };

    bool flag;  // 这个值为true时，左侧电机对应的腿在前；否则在后
    motor::Motor Lmotor;
    motor::Motor Rmotor;
    float L1 = 13.5f;   // 大腿，单位cm
    float L2 = 26.0f;   // 小腿，单位cm

    // ik
    FloatVector ik(FloatVector &footPos);
    // 求足端速度
    FloatVector getFootVel(float u, StepState state);   // u为归一化时间
    // 求电机速度
    FloatVector getMotorVel(FloatVector &footVel);
    // 摆动相
    FloatVector step_swing(float u);
    // 支撑相
    FloatVector step_support(float u);

public:
    leg();
    // 一次运动剪辑控制
    void to_footPos(FloatVector &footPos, FloatVector &footVel);
};
}