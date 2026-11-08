#include <mujoco/mujoco.h>
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
using V2 = Eigen::Vector2d;
using V3 = Eigen::Vector3d;
using V4 = Eigen::Vector4d;
using M2 = Eigen::Matrix2d;
using M3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

int id(const mjModel* m, mjtObj type, const std::string& name) {
    const int result = mj_name2id(m, type, name.c_str());
    require(result >= 0, "Model object not found");
    return result;
}

V2 unit(double angle) { return {std::cos(angle), std::sin(angle)}; }
V2 perpendicular(const V2& value) { return {-value.y(), value.x()}; }
double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

struct Branch {
    int active, passive, site;
    double l1, l2, alpha, beta;
    V3 hip;
};

struct Leg {
    std::array<Branch, 2> branches;  // 顺序固定为前支链、后支链。
    double direction, closureY, footY, assembly;
    int footGeom;
};

Leg readLeg(const mjModel* m, const std::string& name) {
    Leg leg{};
    leg.direction = name.back() == 'L' ? 1 : -1;
    for (int i = 0; i < 2; ++i) {
        const std::string suffix = i == 0 ? "front" : "rear";
        const std::string thigh = name + "_thigh_" + suffix;
        const std::string calf = name + "_calf_" + suffix;
        const int thighBody = id(m, mjOBJ_BODY, thigh);
        const int calfBody = id(m, mjOBJ_BODY, calf);
        auto& branch = leg.branches[i];
        branch.active = id(m, mjOBJ_JOINT, thigh + "_joint");
        branch.passive = id(m, mjOBJ_JOINT, calf + "_joint");
        branch.site = -1;
        for (int site = 0; site < m->nsite; ++site) {
            if (m->site_bodyid[site] == calfBody) branch.site = site;
        }
        require(branch.site >= 0, "Missing closure site");
        branch.hip = Eigen::Map<const V3>(m->body_pos + 3 * thighBody);
        const V3 a = Eigen::Map<const V3>(m->body_pos + 3 * calfBody);
        const V3 b = Eigen::Map<const V3>(m->site_pos + 3 * branch.site);
        branch.l1 = a.head<2>().norm();
        branch.l2 = b.head<2>().norm();
        branch.alpha = std::atan2(leg.direction * a.y(), a.x());
        branch.beta = std::atan2(leg.direction * b.y(), b.x());
        if (i == 1) {
            leg.closureY = branch.hip.y() + leg.direction * (a.z() + b.z());
            leg.footGeom = id(m, mjOBJ_GEOM, calf + "_mesh_2");
            const V3 center = Eigen::Map<const V3>(m->geom_pos + 3 * leg.footGeom);
            leg.footY = branch.hip.y() + leg.direction * (a.z() + center.z());
        }
    }
    const auto& f = leg.branches[0];
    const auto& r = leg.branches[1];
    const V2 kf = f.l1 * unit(f.alpha), kr = r.l1 * unit(r.alpha);
    const V2 home = kf + f.l2 * unit(f.beta);
    leg.assembly = perpendicular(kr - kf).dot(home - kf) >= 0 ? 1 : -1;
    return leg;
}

// 闭链正解：两个小腿长度圆的交点，装配分支沿用 home 姿态。
V2 forward(const Leg& leg, const V2& active) {
    const auto& f = leg.branches[0];
    const auto& r = leg.branches[1];
    const V2 kf = f.l1 * unit(f.alpha + leg.direction * active[0]);
    const V2 kr = r.l1 * unit(r.alpha + leg.direction * active[1]);
    const V2 delta = kr - kf;
    const double distance = delta.norm();
    require(distance > 1e-9, "Coincident knees: forward kinematics is not unique");
    const double a = (f.l2 * f.l2 - r.l2 * r.l2 + distance * distance) / (2 * distance);
    const double h2 = f.l2 * f.l2 - a * a;
    require(h2 >= -1e-12, "Lower-link circles do not intersect");
    const V2 direction = delta / distance;
    return kf + a * direction + leg.assembly * std::sqrt(std::max(0.0, h2)) * perpendicular(direction);
}

// 闭合点逆解；返回 [前髋、后髋、前被动关节、后被动关节]，沿用 home 肘形分支。
V4 inverse(const mjModel* m, const Leg& leg, const V2& target) {
    V4 result;
    for (int i = 0; i < 2; ++i) {
        const auto& branch = leg.branches[i];
        double cosine = (target.squaredNorm() - branch.l1 * branch.l1 - branch.l2 * branch.l2) /
                        (2 * branch.l1 * branch.l2);
        require(std::abs(cosine) <= 1 + 1e-10, "Unreachable closure target");
        cosine = std::clamp(cosine, -1.0, 1.0);
        const double delta0 = wrap(branch.beta - branch.alpha);
        const double delta = std::copysign(std::acos(cosine), delta0);
        const double theta = std::atan2(target.y(), target.x()) -
            std::atan2(branch.l2 * std::sin(delta), branch.l1 + branch.l2 * std::cos(delta));
        result[i] = wrap(leg.direction * (theta - branch.alpha));
        result[i + 2] = wrap(leg.direction * (delta - delta0));
        const Eigen::Vector2i joints(branch.active, branch.passive);
        for (int j = 0; j < 2; ++j) {
            const double q = result[i + 2 * j];
            require(q >= m->jnt_range[2 * joints[j]] - 1e-9 &&
                    q <= m->jnt_range[2 * joints[j] + 1] + 1e-9, "IK joint limit exceeded");
        }
    }
    return result;
}

// 闭链约束 A * 足端速度 = B * 髋关节速度，使用线性方程求解而非显式求逆。
M2 jacobian(const Leg& leg, const V2& active) {
    const V2 c = forward(leg, active);
    M2 a, b = M2::Zero();
    for (int i = 0; i < 2; ++i) {
        const auto& branch = leg.branches[i];
        const double theta = branch.alpha + leg.direction * active[i];
        const V2 knee = branch.l1 * unit(theta);
        const V2 lower = (c - knee) / branch.l2;
        a.row(i) = lower.transpose();
        b(i, i) = leg.direction * branch.l1 * lower.dot(perpendicular(unit(theta)));
    }
    const auto solver = a.fullPivLu();
    require(solver.isInvertible(), "Singular parallel mechanism Jacobian");
    return solver.solve(b);
}
}  // 局部运动学验证函数。

int main(int argc, char** argv) {
    try {
        char error[2048]{};
        const char* path = argc > 1 ? argv[1] : "simulation/generated_robot-mjcf_v2/robot.xml";
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(mj_loadXML(path, nullptr, error, sizeof(error)), mj_deleteModel);
        require(m != nullptr, error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
        require(d != nullptr, "Cannot allocate MuJoCo state");
        double closureError = 0, centerError = 0, jacobianError = 0, meshContactError = 0;
        int count = 0;
        for (const std::string name : {"RR", "FR", "FL", "RL"}) {
            const auto leg = readLeg(m.get(), name);
            for (int i = -2; i <= 2; ++i) for (int j = -2; j <= 2; ++j) {
                const V2 active(i * 0.04, j * 0.04);
                const V2 c = forward(leg, active);
                const V4 q = inverse(m.get(), leg, c);
                require((q.head<2>() - active).norm() < 1e-7, "FK/IK round trip mismatch");
                mj_resetData(m.get(), d.get());
                for (int branch = 0; branch < 2; ++branch) {
                    d->qpos[m->jnt_qposadr[leg.branches[branch].active]] = q[branch];
                    d->qpos[m->jnt_qposadr[leg.branches[branch].passive]] = q[branch + 2];
                }
                mj_forward(m.get(), d.get());
                const V3 expected(leg.branches[0].hip.x() + c.x(), leg.closureY, -c.y());
                for (const auto& branch : leg.branches) {
                    const V3 actual = Eigen::Map<const V3>(d->site_xpos + 3 * branch.site);
                    closureError = std::max(closureError, (expected - actual).norm());
                }
                V3 expectedCenter = expected;
                expectedCenter.y() = leg.footY;
                const V3 actualCenter = Eigen::Map<const V3>(d->geom_xpos + 3 * leg.footGeom);
                centerError = std::max(centerError, (expectedCenter - actualCenter).norm());
                const M2 analytic = jacobian(leg, active);
                for (int column = 0; column < 2; ++column) {
                    V2 plus = active, minus = active;
                    plus[column] += 1e-6;
                    minus[column] -= 1e-6;
                    const V2 finite = (forward(leg, plus) - forward(leg, minus)) / 2e-6;
                    jacobianError = std::max(jacobianError, (finite - analytic.col(column)).norm());
                }
                // 编译后的足部三角网格，验证理想圆柱底点与实际多边形底点的误差。
                const int mesh = m->geom_dataid[leg.footGeom];
                const Eigen::Map<const M3> rotation(d->geom_xmat + 9 * leg.footGeom);
                double minimumZ = 1e9;
                for (int vertex = 0; vertex < m->mesh_vertnum[mesh]; ++vertex) {
                    const auto* raw = m->mesh_vert + 3 * (m->mesh_vertadr[mesh] + vertex);
                    const V3 local(raw[0], raw[1], raw[2]);
                    minimumZ = std::min(minimumZ, (actualCenter + rotation * local).z());
                }
                const double contactError = minimumZ - (actualCenter.z() - 0.031);
                require(contactError >= -1e-7 && contactError < 0.000227, "Foot cylinder support approximation mismatch");
                meshContactError = std::max(meshContactError, contactError);
                ++count;
            }
        }
        require(closureError < 1e-7 && centerError < 1e-7 && jacobianError < 1e-7,
                "Analytic kinematics does not match model");
        std::cout << std::setprecision(10) << "PASS: " << count << " poses, FK/IK/closure/foot/Jacobian\n"
                  << "max_closure_position_error_m=" << closureError << '\n'
                  << "max_foot_center_error_m=" << centerError << '\n'
                  << "max_jacobian_finite_difference_error=" << jacobianError << '\n'
                  << "max_polygon_vs_cylinder_bottom_error_m=" << meshContactError << '\n';
    } catch (const std::exception& exception) {
        std::cerr << "FAIL: " << exception.what() << '\n';
        return 1;
    }
}
