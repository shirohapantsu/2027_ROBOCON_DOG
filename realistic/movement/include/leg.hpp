#pragma once

#include "motor.hpp"
#include <cmath>
#include <vector>

using FloatMatrix = std::vector<std::vector<float>>;
using FloatVector = std::vector<float>;

namespace leg {

class leg {
private:
    std::vector<motor::Motor> motors;
    float L1 = 13.5f;   // 大腿，单位cm
    float L2 = 26.0f;   // 小腿，单位cm
    FloatVector ik(FloatVector &footPos);
    FloatMatrix  jcb(FloatVector &footPos);

public:
    leg();
    FloatVector get_footPos();
    void to_footPos(FloatVector &footPos);
};
}