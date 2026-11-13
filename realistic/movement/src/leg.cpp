/********
 * leg.cpp 实现leg类：控制腿到达某个位置，控制足端接触力
 */

#include "../include/leg.hpp"

namespace leg {

FloatVector leg::ik(FloatVector &footPos) {
    float x = footPos[0];
    float z = footPos[1];

    float sqr_r = x*x + z*z;
    float sqrt_r = sqrt(sqr_r);
    float eta = atan2(z, x);  
    float alpha = acos((L1*L1 + sqr_r - L2*L2) / (2.0f*L1*sqrt_r));
    return FloatVector{eta + alpha, eta - alpha};   // 第一项为在前的腿，第二项为在后的腿
}

FloatVector leg::step_support(float u) {
    // 
}


}


