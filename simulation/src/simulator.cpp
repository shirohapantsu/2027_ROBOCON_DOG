#include "walk_sim.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace dog::sim {
namespace {
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTorqueLimit = 23.7;
constexpr double kSpeedLimit = 30.0;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

int id(const mjModel* model, mjtObj type, const std::string& name) {
    const int result = mj_name2id(model, type, name.c_str());
    require(result >= 0, "Model is missing " + name);
    return result;
}

double wrap(double angle) { return std::remainder(angle, 2 * kPi); }

V3 cardinalAxis(const V3& direction) {
    Eigen::Index component;
    direction.cwiseAbs().maxCoeff(&component);
    require(direction.norm() > 1e-6, "Cannot determine model orientation");
    V3 result = V3::Zero();
    result[component] = direction[component] > 0 ? 1 : -1;
    return result;
}

// Locate the rubber sole by its CAD mesh dimensions, rather than its part number.
// Only this collision mesh is retained on each leg; overlapping bolts/housings
// in the raw CAD export are visual geometry, not independent contact surfaces.
int rubberGeom(const mjModel* model, const mjData* data, const std::string& leg) {
    const int body = id(model, mjOBJ_BODY, leg + "_calf_rear");
    for (int geom = model->body_geomadr[body];
         geom < model->body_geomadr[body] + model->body_geomnum[body]; ++geom) {
        if (model->geom_group[geom] != 3 || model->geom_type[geom] != mjGEOM_MESH) continue;
        const int mesh = model->geom_dataid[geom];
        V3 low = V3::Constant(std::numeric_limits<double>::infinity());
        V3 high = -low;
        const Eigen::Map<const M3> rotation(data->geom_xmat + 9 * geom);
        for (int vertex = 0; vertex < model->mesh_vertnum[mesh]; ++vertex) {
            const auto* raw = model->mesh_vert + 3 * (model->mesh_vertadr[mesh] + vertex);
            const V3 point = rotation * V3(raw[0], raw[1], raw[2]);
            low = low.cwiseMin(point);
            high = high.cwiseMax(point);
        }
        const V3 size = high - low;
        if (size.minCoeff() > 0.04 && size.maxCoeff() < 0.08) return geom;
    }
    throw std::runtime_error("Cannot identify rubber foot mesh for " + leg);
}

// MuJoCo 3.14 represents damping as a polynomial; earlier mjSpec versions use a scalar.
template <class T> void setDamping(T& damping, double value) {
    if constexpr (std::is_array_v<T>) damping[0] = value;
    else damping = value;
}

ModelPtr prepareModel(const std::string& path, double timestep) {
    char error[2048]{};
    SpecPtr spec(mj_parseXML(path.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
    require(spec != nullptr, error);
    ModelPtr original(mj_compile(spec.get(), nullptr), mj_deleteModel);
    require(original != nullptr, mjs_getError(spec.get()));
    require(original->nu == 0, "Expected the unactuated realModel CAD export");
    DataPtr home(mj_makeData(original.get()), mj_deleteData);
    require(home != nullptr, "Cannot allocate model inspection data");
    mj_forward(original.get(), home.get());

    std::array<std::string, 4> feet;
    const std::array<std::string, 4> names{{"FL", "FR", "RL", "RR"}};
    for (std::size_t i = 0; i < names.size(); ++i) {
        feet[i] = mj_id2name(original.get(), mjOBJ_GEOM,
                            rubberGeom(original.get(), home.get(), names[i]));
    }

    spec->option.timestep = timestep;
    spec->option.integrator = mjINT_IMPLICITFAST;
    spec->option.solver = mjSOL_NEWTON;
    spec->option.iterations = 80;
    spec->option.tolerance = 1e-10;
    for (auto* e = mjs_firstElement(spec.get(), mjOBJ_GEOM); e;
         e = mjs_nextElement(spec.get(), e)) {
        auto* geom = mjs_asGeom(e);
        geom->contype = geom->conaffinity = 0;
    }
    for (const auto& name : feet) {
        auto* geom = mjs_asGeom(mjs_findElement(spec.get(), mjOBJ_GEOM, name.c_str()));
        geom->contype = 2;
        geom->conaffinity = 1;  // contact with ground, not with other legs
        geom->condim = 3;
        geom->friction[0] = 0.9;
        geom->friction[1] = 0.005;
        geom->friction[2] = 0.0001;
    }

    auto* world = mjs_findBody(spec.get(), "world");
    require(world != nullptr, "Model has no world body");
    auto* ground = mjs_addGeom(world, nullptr);
    mjs_setName(ground->element, "sim_ground");
    ground->type = mjGEOM_PLANE;
    ground->size[0] = ground->size[1] = 5;
    ground->size[2] = 0.1;
    ground->contype = 1;
    ground->conaffinity = 2;
    ground->rgba[0] = 0.22f;
    ground->rgba[1] = 0.25f;
    ground->rgba[2] = 0.29f;
    ground->rgba[3] = 1;
    for (int side : {-1, 1}) {
        auto* light = mjs_addLight(world, nullptr);
        light->pos[0] = 1.5*side;
        light->pos[1] = -1.5*side;
        light->pos[2] = 2.5;
        light->dir[0] = -0.3*side;
        light->dir[1] = 0.3*side;
        light->dir[2] = -1;
        for (int channel = 0; channel < 3; ++channel) {
            light->ambient[channel] = 0.15f;
            light->diffuse[channel] = 0.7f;
            light->specular[channel] = 0.2f;
        }
    }

    auto* base = mjs_findBody(spec.get(), "base_link");
    require(base != nullptr, "Model has no base_link");
    auto* box = mjs_addGeom(base, nullptr);
    mjs_setName(box->element, "sim_base_collision");
    box->type = mjGEOM_BOX;
    box->size[0] = 0.13;
    box->size[1] = 0.22;
    box->size[2] = 0.04;
    box->group = 3;
    box->density = 0;  // retain the CAD body's explicit mass/inertia
    box->contype = 2;
    box->conaffinity = 1;

    for (const auto& leg : names) for (const std::string suffix : {"front", "rear"}) {
        const std::string jointName = leg + "_thigh_" + suffix + "_joint";
        auto* joint = mjs_asJoint(mjs_findElement(spec.get(), mjOBJ_JOINT, jointName.c_str()));
        require(joint != nullptr, "Missing active joint " + jointName);
        joint->actfrclimited = mjLIMITED_TRUE;
        joint->actfrcrange[0] = -kTorqueLimit;
        joint->actfrcrange[1] = kTorqueLimit;
        joint->frictionloss = 0.15825;  // documented GO-M8010-6 output-side friction
        setDamping(joint->damping, 0.0);

        auto* actuator = mjs_addActuator(spec.get(), nullptr);
        mjs_setName(actuator->element, (leg + "_thigh_" + suffix + "_motor").c_str());
        mjs_setString(actuator->target, jointName.c_str());
        actuator->trntype = mjTRN_JOINT;
        actuator->gear[0] = 1;  // ctrl is output-shaft N*m, not rotor torque
        actuator->gaintype = mjGAIN_FIXED;
        actuator->gainprm[0] = 1;
        actuator->biastype = mjBIAS_NONE;
        actuator->dyntype = mjDYN_NONE;
        actuator->ctrllimited = actuator->forcelimited = mjLIMITED_TRUE;
        actuator->ctrlrange[0] = actuator->forcerange[0] = -kTorqueLimit;
        actuator->ctrlrange[1] = actuator->forcerange[1] = kTorqueLimit;
    }
    ModelPtr result(mj_compile(spec.get(), nullptr), mj_deleteModel);
    require(result != nullptr, mjs_getError(spec.get()));
    require(result->nu == 8 && result->neq == 4, "Expected eight motors and four leg closures");
    return result;
}
}  // namespace

Simulation::Simulation(const std::string& modelPath, Options options) : options_(options) {
    require(mj_version() == mjVERSION_HEADER, "MuJoCo headers/library version mismatch");
    require(std::isfinite(options_.timestep) && options_.timestep > 0 && options_.timestep <= 0.002,
            "Timestep must be in (0, 0.002] seconds");
    require(std::isfinite(options_.kp) && options_.kp >= 0 &&
            std::isfinite(options_.kd) && options_.kd >= 0, "Gains must be finite and nonnegative");
    footTarget(options_.gait, false, 0);  // validate before loading the model
    model_ = prepareModel(modelPath, options_.timestep);
    data_.reset(mj_makeData(model_.get()));
    require(data_ != nullptr, "Cannot allocate simulation data");
    gravityData_.reset(mj_makeData(model_.get()));
    require(gravityData_ != nullptr, "Cannot allocate gravity compensation data");
    readGeometry();
    reset();
}

void Simulation::readGeometry() {
    auto* m = model();
    auto* d = data();
    mj_resetData(m, d);
    mj_forward(m, d);
    baseBody_ = id(m, mjOBJ_BODY, "base_link");
    for (int joint = 0; joint < m->njnt; ++joint) {
        if (m->jnt_type[joint] == mjJNT_FREE) rootQpos_ = m->jnt_qposadr[joint];
    }
    require(rootQpos_ >= 0, "Expected a floating base");
    const Eigen::Map<const M3> baseRotation(d->xmat + 9 * baseBody_);
    const V3 baseOrigin = Eigen::Map<const V3>(d->xpos + 3 * baseBody_);
    auto hip = [&](const std::string& name) -> V3 {
        return Eigen::Map<const V3>(d->xpos + 3 * id(m, mjOBJ_BODY, name + "_thigh_front"));
    };
    const V3 right = cardinalAxis(baseRotation.transpose() * (hip("FR") + hip("RR") - hip("FL") - hip("RL")));
    const V3 forward = cardinalAxis(baseRotation.transpose() * (hip("FR") + hip("FL") - hip("RR") - hip("RL")));
    modelToRobot_.row(0) = right.transpose();
    modelToRobot_.row(1) = forward.transpose();
    modelToRobot_.row(2) = right.cross(forward).transpose();
    require((modelToRobot_ * modelToRobot_.transpose() - Eigen::Matrix3d::Identity()).norm() < 1e-9 &&
            modelToRobot_(2, 2) > 0.99, "Expected an axis-aligned, Z-up CAD model");
    const Eigen::Matrix3d worldToRobot = modelToRobot_ * baseRotation.transpose();
    const std::array<std::string, 4> names{{"FL", "FR", "RL", "RR"}};
    for (std::size_t i = 0; i < legs_.size(); ++i) {
        auto& leg = legs_[i];
        leg.name = names[i];
        leg.startsWithSwing = leg.name == "FL" || leg.name == "RR";
        leg.footGeom = rubberGeom(m, d, leg.name);
        std::array<V3, 2> hips;
        for (std::size_t j = 0; j < 2; ++j) {
            auto& b = leg.branches[j];
            b.name = leg.name + (j == 0 ? "_front" : "_rear");
            const std::string suffix = j == 0 ? "front" : "rear";
            const std::string thigh = leg.name + "_thigh_" + suffix;
            const std::string calf = leg.name + "_calf_" + suffix;
            b.active = id(m, mjOBJ_JOINT, thigh + "_joint");
            b.passive = id(m, mjOBJ_JOINT, calf + "_joint");
            b.actuator = id(m, mjOBJ_ACTUATOR, thigh + "_motor");
            const int calfBody = id(m, mjOBJ_BODY, calf);
            for (int site = 0; site < m->nsite; ++site) {
                if (m->site_bodyid[site] == calfBody) b.site = site;
            }
            require(b.site >= 0, "Missing closure site for " + b.name);
            const V3 hipWorld = Eigen::Map<const V3>(d->xanchor + 3 * b.active);
            const V3 kneeWorld = Eigen::Map<const V3>(d->xanchor + 3 * b.passive);
            const V3 footWorld = Eigen::Map<const V3>(d->site_xpos + 3 * b.site);
            leg.closureX = (worldToRobot * (footWorld - baseOrigin)).x();
            hips[j] = worldToRobot * (hipWorld - baseOrigin);
            const V3 a = worldToRobot * (kneeWorld - hipWorld);
            const V3 lower = worldToRobot * (footWorld - kneeWorld);
            const V3 foot = worldToRobot * (footWorld - hipWorld);
            b.l1 = a.tail<2>().norm();
            b.l2 = lower.tail<2>().norm();
            // Two rear CAD branches measure 0.260164 m, rather than the nominal 0.260 m.
            require(std::abs(b.l1 - 0.135) < 1e-3 && std::abs(b.l2 - 0.260) < 1e-3,
                    "CAD link dimensions disagree with the existing leg controller");
            b.alpha = std::atan2(a.z(), a.y());
            b.beta = std::atan2(lower.z(), lower.y());
            b.activeSign = (worldToRobot * Eigen::Map<const V3>(d->xaxis + 3 * b.active)).x();
            b.passiveSign = (worldToRobot * Eigen::Map<const V3>(d->xaxis + 3 * b.passive)).x();
            require(std::abs(std::abs(b.activeSign) - 1) < 1e-8 &&
                    std::abs(std::abs(b.passiveSign) - 1) < 1e-8, "Joint axes must be transverse");
            b.ikSign = wrap(b.alpha - std::atan2(foot.z(), foot.y())) > 0 ? 1 : -1;
        }
        leg.hip = (hips[0] + hips[1]) / 2;
        for (std::size_t j = 0; j < 2; ++j) {
            leg.branches[j].hipOffset = (hips[j] - leg.hip).tail<2>();
        }
        require(leg.branches[0].ikSign != leg.branches[1].ikSign,
                "Both CAD branches select the same IK solution");
    }
}

JointTarget Simulation::inverse(const Branch& b, const FootTarget& foot) const {
    const Eigen::Vector2d p = foot.position - b.hipOffset;
    const double y = p.x(), z = p.y(), r = p.norm();
    require(r > 0.14 && r < 0.35, b.name + ": target outside the existing leg workspace");
    const double cosine = (b.l1*b.l1 + r*r - b.l2*b.l2) / (2*b.l1*r);
    require(std::abs(cosine) <= 1 + 1e-10, b.name + ": unreachable target");
    const double theta = std::atan2(z, y) + b.ikSign * std::acos(std::clamp(cosine, -1.0, 1.0));
    const double c = std::cos(theta), s = std::sin(theta);
    const double denominator = b.l1 * (z*c - y*s);
    require(std::abs(denominator) > 1e-9, b.name + ": singular velocity inverse");
    const double dy = foot.velocity.x(), dz = foot.velocity.y();
    // Same differential inverse as getMotorPV, before applying its speed clamp.
    const double thetaDot = ((y-b.l1*c)*dy + (z-b.l1*s)*dz) / denominator;
    const double lowerY = y-b.l1*c, lowerZ = z-b.l1*s;
    const double lowerAngle = std::atan2(lowerZ, lowerY);
    const double lowerDot = (lowerY*(dz-b.l1*c*thetaDot) -
                             lowerZ*(dy+b.l1*s*thetaDot)) / (b.l2*b.l2);
    JointTarget target;
    target.q = wrap((theta-b.alpha) / b.activeSign);
    target.dq = std::clamp(thetaDot, -28.0, 28.0) / b.activeSign;
    target.passiveQ = wrap((lowerAngle-theta-(b.beta-b.alpha)) / b.passiveSign);
    target.passiveDq = (lowerDot-thetaDot) / b.passiveSign;
    for (const auto& item : {std::make_pair(b.active, target.q), std::make_pair(b.passive, target.passiveQ)}) {
        require(!model()->jnt_limited[item.first] ||
                (item.second >= model()->jnt_range[2*item.first] - 1e-8 &&
                 item.second <= model()->jnt_range[2*item.first+1] + 1e-8),
                b.name + ": IK exceeds CAD joint limits");
    }
    return target;
}

std::array<std::array<JointTarget, 2>, 4> Simulation::targets(double phaseValue) const {
    std::array<std::array<JointTarget, 2>, 4> result;
    for (std::size_t i = 0; i < legs_.size(); ++i) {
        const auto foot = footTarget(options_.gait, legs_[i].startsWithSwing, phaseValue);
        for (std::size_t j = 0; j < 2; ++j) result[i][j] = inverse(legs_[i].branches[j], foot);
    }
    return result;
}

FeedforwardFrame Simulation::feedforward(double phaseValue) const {
    FeedforwardFrame frame;
    const auto* m = model();
    auto* g = gravityData_.get();
    // Evaluate gravity at the measured pose with zero velocity. This excludes
    // Coriolis/centrifugal terms and does not apply compensation to the floating base.
    mju_copy(g->qpos, data()->qpos, m->nq);
    mju_zero(g->qvel, m->nv);
    mj_fwdPosition(m, g);
    mj_fwdVelocity(m, g);

    // Zero-amplitude trajectories are a standing experiment: all four feet support.
    const bool standing = options_.gait.stepLength == 0 && options_.gait.stepHeight == 0;
    std::array<bool, 4> support{};
    int supportCount = 0;
    for (std::size_t i = 0; i < legs_.size(); ++i) {
        support[i] = standing || footTarget(options_.gait, legs_[i].startsWithSwing, phaseValue).support;
        if (support[i]) ++supportCount;
    }
    require(supportCount > 0, "Gravity feedforward needs at least one planned support foot");
    const double mass = mju_sum(m->body_mass, m->nbody);
    const V3 force = -mass/supportCount * Eigen::Map<const V3>(m->opt.gravity);
    const Eigen::Map<const M3> baseRotation(g->xmat+9*baseBody_);
    const Eigen::Matrix3d worldToRobot = modelToRobot_*baseRotation.transpose();
    using Jacobian = Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>;
    Jacobian first(3, m->nv), second(3, m->nv), foot(3, m->nv);

    for (std::size_t i = 0; i < legs_.size(); ++i) {
        const auto& leg = legs_[i];
        if (support[i]) frame.supportForces[i] = force;
        mj_jacSite(m, g, first.data(), nullptr, leg.branches[0].site);
        mj_jacSite(m, g, second.data(), nullptr, leg.branches[1].site);
        const Jacobian closure = worldToRobot*(first-second);
        Eigen::Matrix2d active, passive;
        Eigen::Vector2d activeGravity, passiveGravity;
        for (std::size_t j = 0; j < 2; ++j) {
            const int a = m->jnt_dofadr[leg.branches[j].active];
            const int p = m->jnt_dofadr[leg.branches[j].passive];
            active.col(j) = closure.col(a).tail<2>();
            passive.col(j) = closure.col(p).tail<2>();
            activeGravity[j] = g->qfrc_bias[a];
            passiveGravity[j] = g->qfrc_bias[p];
        }
        const auto solver = passive.fullPivLu();
        require(solver.isInvertible(), leg.name+": singular passive-joint closure Jacobian");
        // Closure enforces dq_passive = coupling * dq_active. Virtual work folds
        // BOTH passive-joint gravity and foot forces back into the two actuators.
        const Eigen::Matrix2d coupling = solver.solve(-active);
        const Eigen::Vector2d gravity = activeGravity+coupling.transpose()*passiveGravity;
        mj_jacGeom(m, g, foot.data(), nullptr, leg.footGeom);
        Eigen::Matrix<double, 3, 2> footActive, footPassive;
        for (std::size_t j = 0; j < 2; ++j) {
            footActive.col(j) = foot.col(m->jnt_dofadr[leg.branches[j].active]);
            footPassive.col(j) = foot.col(m->jnt_dofadr[leg.branches[j].passive]);
        }
        const Eigen::Matrix<double, 3, 2> reducedJacobian = footActive+footPassive*coupling;
        // f is ground-on-foot, hence the minus sign. World force and world Jacobian
        // are paired; output torques already follow each CAD actuator's axis.
        const Eigen::Vector2d tau = gravity-reducedJacobian.transpose()*frame.supportForces[i];
        require(tau.allFinite(), leg.name+": non-finite gravity feedforward");
        // A fallen or nearly singular leg can request infeasible torque. Bound
        // feedforward itself, then bound the complete FF+PD command in step().
        for (std::size_t j = 0; j < 2; ++j) {
            frame.tau[i][j] = std::clamp(tau[j], -kTorqueLimit, kTorqueLimit);
        }
    }
    return frame;
}

double Simulation::phase() const { return std::fmod(data()->time / options_.gait.period, 1.0); }

void Simulation::applyPose(double phaseValue) {
    const auto commands = targets(phaseValue);
    for (std::size_t i = 0; i < legs_.size(); ++i) for (std::size_t j = 0; j < 2; ++j) {
        const auto& b = legs_[i].branches[j];
        const auto& command = commands[i][j];
        data()->qpos[model()->jnt_qposadr[b.active]] = command.q;
        data()->qpos[model()->jnt_qposadr[b.passive]] = command.passiveQ;
        data()->qvel[model()->jnt_dofadr[b.active]] = command.dq;
        data()->qvel[model()->jnt_dofadr[b.passive]] = command.passiveDq;
    }
    data()->qpos[rootQpos_+2] = baseHeight_;
    lastControlPhase_ = phaseValue;
    mj_forward(model(), data());
}

void Simulation::reset() {
    mj_resetData(model(), data());
    lastFeedforward_ = {};
    baseHeight_ = 0;
    applyPose(0);
    double minimumZ = std::numeric_limits<double>::infinity();
    for (const auto& leg : legs_) {
        const int geom = leg.footGeom, mesh = model()->geom_dataid[geom];
        const Eigen::Map<const M3> rotation(data()->geom_xmat + 9 * geom);
        const Eigen::Map<const V3> position(data()->geom_xpos + 3 * geom);
        for (int vertex = 0; vertex < model()->mesh_vertnum[mesh]; ++vertex) {
            const auto* raw = model()->mesh_vert + 3 * (model()->mesh_vertadr[mesh] + vertex);
            minimumZ = std::min(minimumZ, (position + rotation * V3(raw[0],raw[1],raw[2])).z());
        }
    }
    baseHeight_ = -minimumZ + 0.001;
    applyPose(0);
    mju_zero(data()->qvel, model()->nv);
    mj_forward(model(), data());
    checkState();
}

void Simulation::step() {
    if (options_.kinematic) {
        data()->time += model()->opt.timestep;
        applyPose(phase());
    } else {
        lastControlPhase_ = phase();
        auto commands = targets(lastControlPhase_);
        lastFeedforward_ = options_.gravityFeedforward ? feedforward(lastControlPhase_) : FeedforwardFrame{};
        for (std::size_t i = 0; i < legs_.size(); ++i) for (std::size_t j = 0; j < 2; ++j) {
            const auto& b = legs_[i].branches[j];
            auto& command = commands[i][j];
            command.tau = lastFeedforward_.tau[i][j];
            const double q = data()->qpos[model()->jnt_qposadr[b.active]];
            const double dq = data()->qvel[model()->jnt_dofadr[b.active]];
            double torque = command.tau + options_.kp*(command.q-q) + options_.kd*(command.dq-dq);
            // Simple documented 24 V torque/speed envelope; full braking torque is retained.
            const double limit = torque*dq > 0
                ? kTorqueLimit * std::clamp(1-std::abs(dq)/kSpeedLimit, 0.0, 1.0) : kTorqueLimit;
            torque = std::clamp(torque, -limit, limit);
            data()->ctrl[b.actuator] = torque;
        }
        mj_step(model(), data());
        mj_forward(model(), data());
    }
    checkState();
}

void Simulation::checkState() const {
    for (int i = 0; i < model()->nq; ++i) require(std::isfinite(data()->qpos[i]), "Non-finite qpos");
    for (int i = 0; i < model()->nv; ++i) require(std::isfinite(data()->qvel[i]), "Non-finite qvel");
    require(data()->warning[mjWARN_BADQPOS].number == 0 &&
            data()->warning[mjWARN_BADQVEL].number == 0 &&
            data()->warning[mjWARN_BADQACC].number == 0, "MuJoCo reported an unstable state");
}

Metrics Simulation::metrics() const {
    Metrics result;
    result.base = modelToRobot_ * Eigen::Map<const V3>(data()->xpos + 3 * baseBody_);
    result.upright = data()->xmat[9*baseBody_+8];
    result.fallen = result.base.z() < 0.12 || result.upright < 0.5;
    const auto commands = targets(lastControlPhase_);
    double sumSquared = 0;
    for (std::size_t i = 0; i < legs_.size(); ++i) {
        const auto& leg = legs_[i];
        result.desiredSupportZ += lastFeedforward_.supportForces[i].z();
        const V3 a = Eigen::Map<const V3>(data()->site_xpos + 3*leg.branches[0].site);
        const V3 b = Eigen::Map<const V3>(data()->site_xpos + 3*leg.branches[1].site);
        result.closureError = std::max(result.closureError, (a-b).norm());
        for (std::size_t j = 0; j < 2; ++j) {
            const auto& branch = leg.branches[j];
            const double error = commands[i][j].q-data()->qpos[model()->jnt_qposadr[branch.active]];
            sumSquared += error*error;
            result.maxTorque = std::max(result.maxTorque, std::abs(data()->actuator_force[branch.actuator]));
            result.maxFeedforwardTorque = std::max(result.maxFeedforwardTorque, std::abs(lastFeedforward_.tau[i][j]));
        }
    }
    result.trackingError = std::sqrt(sumSquared/8);
    const int ground = id(model(), mjOBJ_GEOM, "sim_ground");
    for (int i = 0; i < data()->ncon; ++i) {
        if (data()->contact[i].geom1 == ground || data()->contact[i].geom2 == ground) ++result.groundContacts;
    }
    return result;
}

void Simulation::saveSnapshot(const std::string& path) const {
    ModelPtr snapshot(mj_copyModel(nullptr, model()), mj_deleteModel);
    require(snapshot != nullptr, "Cannot copy snapshot model");
    mju_copy(snapshot->qpos0, data()->qpos, model()->nq);
    mj_saveModel(snapshot.get(), path.c_str(), nullptr, 0);
}

}  // namespace dog::sim
