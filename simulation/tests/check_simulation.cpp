#include "walk_sim.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
double wrap(double q) { return std::remainder(q, 2*std::acos(-1.0)); }

void setLegPose(dog::sim::Simulation& sim, std::size_t index, const dog::sim::FootTarget& foot) {
    for (const auto& branch : sim.legs()[index].branches) {
        const auto target = sim.inverse(branch, foot);
        sim.data()->qpos[sim.model()->jnt_qposadr[branch.active]] = target.q;
        sim.data()->qpos[sim.model()->jnt_qposadr[branch.passive]] = target.passiveQ;
    }
    mj_forward(sim.model(), sim.data());
}

double virtualPotential(const dog::sim::Simulation& sim, const dog::sim::FeedforwardFrame& frame) {
    const Eigen::Map<const Eigen::Vector3d> gravity(sim.model()->opt.gravity);
    double value = 0;
    for (int body = 1; body < sim.model()->nbody; ++body) {
        const Eigen::Map<const Eigen::Vector3d> com(sim.data()->xipos+3*body);
        value -= sim.model()->body_mass[body]*gravity.dot(com);
    }
    for (std::size_t i = 0; i < sim.legs().size(); ++i) {
        const Eigen::Map<const Eigen::Vector3d> foot(sim.data()->geom_xpos+3*sim.legs()[i].footGeom);
        value -= frame.supportForces[i].dot(foot);
    }
    return value;
}

double checkFeedforwardVirtualWork(dog::sim::Simulation& sim) {
    double maxError = 0;
    sim.reset();
    for (double phase : {0.2, 0.5, 0.8}) {
        for (int joint = 0; joint < sim.model()->njnt; ++joint) {
            if (sim.model()->jnt_type[joint] != mjJNT_FREE) continue;
            const mjtNum axis[3]{1, 0, 0};
            mju_axisAngle2Quat(sim.data()->qpos+sim.model()->jnt_qposadr[joint]+3,
                              axis, phase == 0.5 ? 0.15 : 0.0);
        }
        for (std::size_t i = 0; i < sim.legs().size(); ++i) {
            setLegPose(sim, i, dog::sim::footTarget(sim.options().gait, sim.legs()[i].startsWithSwing, phase));
        }
        const auto frame = sim.feedforward(phase);
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < sim.legs().size(); ++i) {
            sum += frame.supportForces[i];
            if (!dog::sim::footTarget(sim.options().gait, sim.legs()[i].startsWithSwing, phase).support) {
                require(frame.supportForces[i].norm() < 1e-12, "Swing foot received body support force");
            }
        }
        const Eigen::Vector3d weight = mju_sum(sim.model()->body_mass, sim.model()->nbody)
            * Eigen::Map<const Eigen::Vector3d>(sim.model()->opt.gravity);
        require((sum+weight).norm() < 1e-10, "Planned support force does not balance total weight");

        // Independent finite-difference check: actuator virtual work must equal
        // the change in gravitational potential minus external-force work.
        // This detects reversed signs and omission of passive-joint gravity.
        for (std::size_t i = 0; i < sim.legs().size(); ++i) for (int axis = 0; axis < 2; ++axis) {
            constexpr double epsilon = 1e-6;
            const auto neutral = dog::sim::footTarget(sim.options().gait, sim.legs()[i].startsWithSwing, phase);
            auto plus = neutral, minus = neutral;
            plus.position[axis] += epsilon;
            minus.position[axis] -= epsilon;
            setLegPose(sim, i, plus);
            const double positive = virtualPotential(sim, frame);
            setLegPose(sim, i, minus);
            const double negative = virtualPotential(sim, frame);
            double work = 0;
            for (std::size_t j = 0; j < 2; ++j) {
                const auto& branch = sim.legs()[i].branches[j];
                const double tangent = wrap(sim.inverse(branch, plus).q-sim.inverse(branch, minus).q)/(2*epsilon);
                work += frame.tau[i][j]*tangent;
            }
            maxError = std::max(maxError, std::abs((positive-negative)/(2*epsilon)-work));
            setLegPose(sim, i, neutral);
        }
    }
    require(maxError < 1e-5, "Closed-chain gravity/support torque failed upright/tilted virtual-work check");
    const Eigen::Vector3d originalGravity = Eigen::Map<const Eigen::Vector3d>(sim.model()->opt.gravity);
    mju_zero(sim.model()->opt.gravity, 3);
    const auto zero = sim.feedforward(0.2);
    for (const auto& leg : zero.tau) for (double tau : leg) {
        require(std::abs(tau) < 1e-12, "Zero gravity produced nonzero gravity feedforward");
    }
    mju_copy(sim.model()->opt.gravity, originalGravity.data(), 3);
    sim.reset();
    return maxError;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "Pass the realModel robot.xml path");
        dog::sim::Gait gait;
        for (bool swing : {true, false}) {
            const auto start = dog::sim::footTarget(gait, swing, 0);
            const auto end = dog::sim::footTarget(gait, swing, 1);
            require((start.position-end.position).norm() < 1e-7 &&
                    (start.velocity-end.velocity).norm() < 1e-6, "Gait period is discontinuous");
        }
        require(!dog::sim::footTarget(gait, true, 0.2).support &&
                 dog::sim::footTarget(gait, false, 0.2).support &&
                 dog::sim::footTarget(gait, true, 0.5).support &&
                 dog::sim::footTarget(gait, false, 0.5).support &&
                 dog::sim::footTarget(gait, true, 0.8).support &&
                !dog::sim::footTarget(gait, false, 0.8).support, "Four-leg phase schedule changed");
        bool rejected = false;
        try { auto invalid = gait; invalid.period = 0; dog::sim::footTarget(invalid, true, 0); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Zero gait period must be rejected");

        dog::sim::Options options;
        options.gravityFeedforward = false;  // retain the original PD-only regression
        options.kinematic = true;
        dog::sim::Simulation sim(argv[1], options);
        require(sim.model()->nu == 8 && sim.model()->nv == 22 && sim.model()->neq == 4,
                "Model topology changed");
        require(std::abs(mju_sum(sim.model()->body_mass, sim.model()->nbody)-8.108) < 1e-6,
                "Scene adaptation changed the CAD mass");
        double maxFootError = 0, maxVelocityError = 0, maxClosure = 0, maxNominalError = 0;
        for (int sample = 0; sample <= 100; ++sample) {
            const double phase = sample/100.0;
            const auto commands = sim.targets(phase);
            for (std::size_t i = 0; i < sim.legs().size(); ++i) for (std::size_t j = 0; j < 2; ++j) {
                const auto& b = sim.legs()[i].branches[j];
                sim.data()->qpos[sim.model()->jnt_qposadr[b.active]] = commands[i][j].q;
                sim.data()->qpos[sim.model()->jnt_qposadr[b.passive]] = commands[i][j].passiveQ;
            }
            mj_forward(sim.model(), sim.data());
            maxClosure = std::max(maxClosure, sim.metrics().closureError);
            const int base = mj_name2id(sim.model(), mjOBJ_BODY, "base_link");
            const Eigen::Vector3d basePosition = Eigen::Map<const Eigen::Vector3d>(sim.data()->xpos+3*base);
            for (const auto& leg : sim.legs()) {
                const auto foot = dog::sim::footTarget(gait, leg.startsWithSwing, phase);
                const double r = foot.position.norm();
                const double eta = std::atan2(foot.position.y(), foot.position.x());
                const double alpha = std::acos((0.135*0.135+r*r-0.260*0.260)/(2*0.135*r));
                for (const auto& b : leg.branches) {
                    const auto target = sim.inverse(b, foot);
                    const Eigen::Vector3d actual = sim.modelToRobot() *
                        (Eigen::Map<const Eigen::Vector3d>(sim.data()->site_xpos+3*b.site)-basePosition);
                    const Eigen::Vector3d desired(leg.closureX, leg.hip.y()+foot.position.x(),
                                                  leg.hip.z()+foot.position.y());
                    maxFootError = std::max(maxFootError, (actual-desired).norm());
                    maxNominalError = std::max(maxNominalError,
                        std::abs(wrap(b.alpha+b.activeSign*target.q-(eta+b.ikSign*alpha))));
                    if (phase > 0.001 && phase < 0.999 &&
                        std::abs(phase-0.4) > 0.001 && std::abs(phase-0.6) > 0.001) {
                        constexpr double epsilon = 1e-6;
                        const auto plus = sim.inverse(b, dog::sim::footTarget(gait, leg.startsWithSwing, phase+epsilon));
                        const auto minus = sim.inverse(b, dog::sim::footTarget(gait, leg.startsWithSwing, phase-epsilon));
                        maxVelocityError = std::max({maxVelocityError,
                            std::abs(wrap(plus.q-minus.q)/(2*epsilon*gait.period)-target.dq),
                            std::abs(wrap(plus.passiveQ-minus.passiveQ)/(2*epsilon*gait.period)-target.passiveDq)});
                    }
                }
            }
        }
        require(maxFootError < 1e-7 && maxClosure < 1e-7, "IK does not close the actual CAD mechanism");
        require(maxNominalError < 0.002, "CAD mapping disagrees with getMotorPV angle convention");
        require(maxVelocityError < 1e-5, "Analytic joint velocity disagrees with finite differences");
        sim.reset();
        for (int step = 0; step < 2000; ++step) sim.step();
        require(sim.metrics().trackingError < 1e-10 && sim.metrics().closureError < 1e-7,
                "Kinematic preview drifted from its target");
        const double maxVirtualWorkError = checkFeedforwardVirtualWork(sim);

        options.kinematic = false;
        dog::sim::Simulation physics(argv[1], options);
        double maxDynamicClosure = 0, peakTorque = 0;
        bool hadGroundContact = false, fell = false;
        for (int step = 0; step < 2000; ++step) {
            physics.step();
            const auto m = physics.metrics();
            maxDynamicClosure = std::max(maxDynamicClosure, m.closureError);
            peakTorque = std::max(peakTorque, m.maxTorque);
            hadGroundContact = hadGroundContact || m.groundContacts > 0;
            fell = fell || m.fallen;
        }
        require(hadGroundContact, "Dynamics never generated ground contacts");
        require(peakTorque <= 23.7+1e-8, "Motor torque exceeded its output-side limit");
        require(maxDynamicClosure < 0.01, "Dynamic closed-chain constraint error is excessive");

        options.kd = 100;  // Deliberately request more than peak torque at startup.
        dog::sim::Simulation saturated(argv[1], options);
        saturated.step();
        const double limitedTorque = saturated.metrics().maxTorque;
        require(limitedTorque > 23 && limitedTorque <= 23.7+1e-8,
                "High demand did not exercise the peak-torque clamp");

        options.kp = 0.4;
        options.kd = 0.2;
        options.gait.stepLength = options.gait.stepHeight = 0;
        options.gravityFeedforward = true;
        dog::sim::Simulation supported(argv[1], options);
        options.gravityFeedforward = false;
        dog::sim::Simulation unsupported(argv[1], options);
        for (int step = 0; step < 2000; ++step) { supported.step(); unsupported.step(); }
        const auto withFeedforward = supported.metrics(), withoutFeedforward = unsupported.metrics();
        require(withFeedforward.base.z()-withoutFeedforward.base.z() > 0.08 &&
                withFeedforward.trackingError < withoutFeedforward.trackingError/5,
                "Gravity feedforward did not improve low-gain static weight support");
        require(withFeedforward.maxFeedforwardTorque > 0.5 &&
                withoutFeedforward.maxFeedforwardTorque == 0,
                "Feedforward switch did not control the applied tau command");
        std::cout << "PASS: gait continuity, diagonal schedule, 101 CAD poses, IK/closure, velocities, "
                     "finite dynamics, torque limits, support force balance, closed-chain virtual work, "
                     "zero gravity and low-gain weight compensation\n"
                  << "max_foot_error_m=" << maxFootError << '\n'
                  << "max_velocity_error_rad_s=" << maxVelocityError << '\n'
                  << "max_nominal_angle_error_rad=" << maxNominalError << '\n'
                  << "max_dynamic_closure_m=" << maxDynamicClosure << '\n'
                  << "max_feedforward_virtual_work_error=" << maxVirtualWorkError << '\n'
                  << "standing_height_with_ff_m=" << withFeedforward.base.z() << '\n'
                  << "standing_height_without_ff_m=" << withoutFeedforward.base.z() << '\n'
                  << "existing_gains_ever_fallen=" << std::boolalpha << fell << '\n';
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
