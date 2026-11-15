/********
 * leg.cpp 实现leg类：控制腿到达某个位置，控制足端接触力
 */

#include "../include/leg.hpp"

namespace leg {

// 右手机体系：+x 向右、+y 向前、+z 向上。
// 足端平面向量顺序为 [y, z]；连杆角度从 +y 朝 +z 为正（绕 +x）。

FloatVector leg::ik(FloatVector &footPosYZ) {
    float y = footPosYZ[0];
    float z = footPosYZ[1];

    float sqr_r = y*y + z*z;
    float sqrt_r = sqrt(sqr_r);
    float eta = atan2(z, y);
    float alpha = acos((L1*L1 + sqr_r - L2*L2) / (2.0f*L1*sqrt_r));
    return FloatVector{eta + alpha, eta - alpha};   // 第一项为在前的腿，第二项为在后的腿
}

FloatMatrix leg::getMotorPV(const FloatVector &footPosYZ, const FloatVector &footVelYZ) {
    float y = footPosYZ[0];  float z = footPosYZ[1];
    float dy = footVelYZ[0]; float dz = footVelYZ[1];

    float sqr_r = y*y + z*z;
    float r = sqrt(sqr_r);
    // 奇异点分析
    if(r >= MAX_HEIGHT || r <= MIN_HEIGHT) 
        throw std::runtime_error("腿部高度超出设计范围");

    float eta = atan2(z, y);
    float alpha = acos((L1*L1 + sqr_r - L2*L2) / (2.0f*L1*r));
    float theta1 = eta + alpha;
    float theta2 = eta - alpha;

    float c1 = cos(theta1), c2 = cos(theta2);
    float s1 = sin(theta1), s2 = sin(theta2);

    float d_theta1 = ((y - L1*c1)*dy + (z - L1*s1)*dz)
                     / (L1*(z*c1 - y*s1));
    float d_theta2 = ((y - L1*c2)*dy + (z - L1*s2)*dz)
                     / (L1*(z*c2 - y*s2));
    
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
    float y = PACE_LENGHT / 2.0f - PACE_LENGHT * u_sp;

    float dy = (-PACE_LENGHT) / T_sp;
    return {
        {y, z},
        {dy, 0.0f}
    };
}

FloatMatrix leg::step_swing(float u_sw, float T_sw, float T_sp) {
    // 摆动相，返回位置和速度
    // 前进方向（y）采用五次多项式设计，速度和加速度连续
    float uu = u_sw * u_sw;
    float uuu = uu * u_sw;
    float uuuu = uuu * u_sw;
    float uuuuu = uuuu * u_sw;

    float v_sp = PACE_LENGHT / T_sp;

    float y = (-PACE_LENGHT/2) - v_sp*T_sw*u_sw + (PACE_LENGHT + v_sp*T_sw)*(10.0f*uuu - 15.0f*uuuu + 6.0f*uuuuu);
    float dy = -v_sp + (PACE_LENGHT + v_sp*T_sw)/T_sw * 30.0f * uu * (1.0f - u_sw) * (1.0-u_sw);

    float z = STAND_Z + STEP_HEIGHT * sin(pi*u_sw) * sin(pi*u_sw);
    float dz = (pi*STEP_HEIGHT) / T_sw * sin(2.0f*pi*u_sw);

    return {
        {y, z},
        {dy, dz}
    };
}

leg::leg(float period,
    std::shared_ptr<SerialPort> serial, 
    int id1, float offset1, bool direction1,
    int id2, float offset2, bool direction2)
    : motor1(serial, id1, offset1, direction1),
      motor2(serial, id2, offset2, direction2),
      period_(period) {
        
    
}

void leg::to_footPos(FloatVector &footPosYZ, FloatVector &footVelYZ) {
    FloatMatrix motorparams = getMotorPV(footPosYZ, footVelYZ);\
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

void  leg::onestep(StepState startstate ,float u) {
    FloatMatrix target;
    float T_sw = period_ * (1.0f - SUPPORT_RATIO);
    float T_sp = period_ * SUPPORT_RATIO;
    if(startstate == StepState::Support) {
        if(u <= SUPPORT_RATIO) {
            u = u / SUPPORT_RATIO;
            target =  step_support(u, T_sp);
        }
        else{
            u = (u - SUPPORT_RATIO) / (1.0f - SUPPORT_RATIO);
            target = step_swing(u, T_sw, T_sp);
        }
    }
    if(startstate == StepState::Swing) {
        if(u > (1.0f - SUPPORT_RATIO)) {
            u = (u - (1.0f - SUPPORT_RATIO)) / SUPPORT_RATIO;
            target =  step_support(u, T_sp);
        }
        else{
            u = u / (1.0f - SUPPORT_RATIO);
            target = step_swing(u, T_sw, T_sp);
        }
    }
    to_footPos(target[0], target[1]);
}
}


