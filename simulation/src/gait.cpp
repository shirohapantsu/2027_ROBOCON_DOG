#include "walk_sim.hpp"

#include <cmath>
#include <stdexcept>

namespace dog::sim {

FootTarget footTarget(const Gait& g, bool startsWithSwing, double phase) {
    if (!std::isfinite(g.period) || g.period <= 0 || !std::isfinite(g.supportRatio) ||
        g.supportRatio <= 0 || g.supportRatio >= 1 || !std::isfinite(phase) ||
        phase < 0 || phase > 1 || !std::isfinite(g.stepLength) || g.stepLength < 0 ||
        !std::isfinite(g.standZ) || !std::isfinite(g.stepHeight) || g.stepHeight < 0) {
        throw std::invalid_argument("Invalid gait parameters or phase");
    }
    const double swingRatio = 1 - g.supportRatio;
    const bool support = startsWithSwing ? phase > swingRatio : phase <= g.supportRatio;
    const double u = startsWithSwing
        ? (support ? (phase - swingRatio) / g.supportRatio : phase / swingRatio)
        : (support ? phase / g.supportRatio : (phase - g.supportRatio) / swingRatio);
    const double supportTime = g.period * g.supportRatio;
    const double swingTime = g.period * swingRatio;
    const double v = g.stepLength / supportTime;
    if (support) {
        return {{g.stepLength / 2 - g.stepLength * u, g.standZ}, {-v, 0}, true};
    }

    // Same quintic horizontal and sin^2 vertical trajectory as leg::step_swing.
    constexpr double pi = 3.141593;
    const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
    const double distance = g.stepLength + v * swingTime;
    const double sine = std::sin(pi * u);
    return {
        {-g.stepLength / 2 - v * swingTime * u + distance * (10*u3 - 15*u4 + 6*u5),
         g.standZ + g.stepHeight * sine * sine},
        {-v + distance / swingTime * 30 * u2 * (1-u) * (1-u),
         pi * g.stepHeight / swingTime * std::sin(2*pi*u)},
        false
    };
}

}  // namespace dog::sim
