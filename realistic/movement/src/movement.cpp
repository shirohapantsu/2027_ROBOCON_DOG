#include "../include/movement.hpp"

namespace movement {
/************movement*************/

void movement::walkstright(float period) {
    const int64_t periodNs = 1000000LL; // 1ms
    const double maxDt = 0.005;

    int64_t nowtime, previous = now();
    int64_t next = previous + periodNs;

    double dt;
    float u = 0.0f;

    while(u < 1.0f) {
        timespec target = toTimespec(next);
        int error;
        do {
            error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, nullptr);
        } while(error == EINTR);

        nowtime = now();
        dt = (nowtime - previous) * 1e-9;
        previous = nowtime;

        if(dt > 0.0 && dt < maxDt) {
            u += dt / period;
            if(u > 1.0f) u=1.0f;
            fl.onestep(leg::StepState::Swing, u);
            rr.onestep(leg::StepState::Swing, u);
            fr.onestep(leg::StepState::Support, u);
            rl.onestep(leg::StepState::Support, u);
        }
        else throw std::runtime_error("步态计算下一相位超时");

        next += periodNs;
        // 跳过错过的控制周期
        int64_t finished = now();
        if (finished >= next) {
            const uint64_t skipped =(finished - next) / periodNs + 1;
            next += skipped * periodNs;
        }

    }
}

/************clock****************/

int64_t now() {
    timespec timestamp;
    if(clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0) 
        throw std::runtime_error("时间获取失败");
    
     return int64_t(timestamp.tv_sec) * 1000000000ll + timestamp.tv_nsec;
}

timespec toTimespec(int64_t ns) {
    timespec ts;
    ts.tv_sec = ns / 1000000000LL;
    ts.tv_nsec = ns % 1000000000LL;
    return ts;
}

}