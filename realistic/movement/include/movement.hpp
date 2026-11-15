#pragma once

#include <csignal>  // 提供信号处理函数 std::signal 和 sig_atomic_t
#include <cstdint>  // 提供 int64_t、uint64_t 等固定宽度整数类型，防止时间计算溢出
#include <cstdlib>  // 提供 std::abort()，用于严重错误时立即终止程序
#include <ctime>  // 提供高精度时间结构体 timespec 及 clock_gettime / clock_nanosleep

#include "leg.hpp"

namespace movement {
class movement {
private:
    leg::leg fl;    // 左前
    leg::leg fr;    // 右前
    leg::leg rl;    // 左后
    leg::leg rr;    // you hou

public:
    movement();
    void standup();
    void walkstright(float period);
    void turnto();
    void jump();
    void crouch();
};

// 时间函数
int64_t now();
timespec toTimespec(int64_t ns);


}  // namespace movement