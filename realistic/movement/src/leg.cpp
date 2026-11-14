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

FloatMatrix leg::getMotorPV(const FloatVector &footPos, const FloatVector &footVel) {
    float x = footPos[0];  float z = footPos[1];
    float dx = footVel[0]; float dz = footVel[1];

    float sqr_r = x*x + z*z;
    float r = sqrt(sqr_r);
    // 奇异点分析
    if(r >= MAX_HEIGHT || r <= MIN_HEIGHT) 
        throw std::runtime_error("腿部高度超出设计范围");

    float eta = atan2(z, x);  
    float alpha = acos((L1*L1 + sqr_r - L2*L2) / (2.0f*L1*r));
    float theta1 = eta + alpha;
    float theta2 = eta - alpha;

    float c1 = cos(theta1), c2 = cos(theta2);
    float s1 = sin(theta1), s2 = sin(theta2);

    float d_theta1 = ((x - L1*c1)*dx + (z - L1*s1)*dz)
                     / (L1*(z*c1 - x*s1));
    float d_theta2 = ((x - L1*c2)*dx + (z - L1*s2)*dz)
                     / (L1*(z*c2 - x*s2));
    
    // 速度限幅
    if(d_theta1 > 28.0f) d_theta1 = 28.0f;
    if(d_theta1 < -28.0f) d_theta1 = -28.0f;
    if(d_theta2 > 28.0f) d_theta2 = 28.0f;
    if(d_theta2 < -28.0f) d_theta2 = -28.0f;

    return {
        {theta1, d_theta1},
        {theta2, d_theta2}
    };
}

FloatMatrix leg::step_support(float u_sp, float T_sp) {
    // 支撑项，依次返回位置和速度
    float z = STAND_Z;
    float x = PACE_LENGHT / 2.0f - PACE_LENGHT * u_sp;

    float dx = (-PACE_LENGHT) / T_sp;
    return {
        {x, z},
        {dx, 0.0f}
    };
}

FloatMatrix leg::step_swing(float u_sw, float T_sw, float T_sp) {
    // 摆动相，返回位置和速度
    // 水平方向采用五次多项式设计，速度和加速度连续
    float uu = u_sw * u_sw;
    float uuu = uu * u_sw;
    float uuuu = uuu * u_sw;
    float uuuuu = uuuu * u_sw;

    float v_sp = PACE_LENGHT / T_sp;

    float x = (-PACE_LENGHT/2) - v_sp*T_sw*u_sw + (PACE_LENGHT + v_sp*T_sw)*(10.0f*uuu - 15.0f*uuuu + 6.0f*uuuuu);
    float dx = -v_sp + (PACE_LENGHT + v_sp*T_sw)/T_sw * 30.0f * uu * (1.0f - u_sw) * (1.0-u_sw);

    float z = STAND_Z + STEP_HEIGHT * sin(pi*u_sw) * sin(pi*u_sw);
    float dz = (pi*STEP_HEIGHT) / T_sw * sin(2.0f*pi*u_sw);

    return {
        {x, z},
        {dx, dz}
    };
}

leg::leg() {
    
}

void leg::to_footPos(FloatVector &footPos, FloatVector &footVel) {
    FloatMatrix motorparams = getMotorPV(footPos, footVel);\
    motor::MotorCtrl cmd1 = {
        motor::CtrlMode::MITMode,
        motorparams[0][0],
        motorparams[0][1],
        0.4f,
        0.2f,
        0.0f
    };
    motor::MotorCtrl cmd2 = {
        motor::CtrlMode::MITMode,
        motorparams[1][0],
        motorparams[1][1],
        0.4f,
        0.2f,
        0.0f
    };
    motor1.editCmd(cmd1);
    motor2.editCmd(cmd2);
    if(!motor1.sendCmd() || !motor2.sendCmd()) std::cerr << "本轮控制失败";
}

}


