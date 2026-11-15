#pragma once

#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include <array>
#include <functional>
#include <memory>
#include <string>

namespace dog::sim {

// Matches realistic/movement/include/leg.hpp. SI units, robot axes: right/front/up.
struct Gait {
    double period = 1.0;
    double stepLength = 0.14;
    double standZ = -0.274;
    double stepHeight = 0.03;
    double supportRatio = 0.6;
};

struct FootTarget {
    Eigen::Vector2d position;  // [y, z] relative to the hip
    Eigen::Vector2d velocity;
    bool support;
};

FootTarget footTarget(const Gait& gait, bool startsWithSwing, double phase);

struct Options {
    Gait gait;
    bool kinematic = false;
    bool gravityFeedforward = true;
    double kp = 0.4;  // output-shaft gains from leg::to_footPos
    double kd = 0.02;
    double timestep = 0.001;
};

struct JointTarget {
    double q = 0;
    double dq = 0;
    double passiveQ = 0;
    double passiveDq = 0;
    double tau = 0;  // feedforward output-shaft N*m in this CAD joint's direction
};

struct Branch {
    std::string name;
    int active = -1;
    int passive = -1;
    int site = -1;
    int actuator = -1;
    double l1 = 0;
    double l2 = 0;
    double alpha = 0;  // upper-link absolute angle at CAD q=0
    double beta = 0;   // lower-link absolute angle at CAD q=0
    double activeSign = 0;
    double passiveSign = 0;
    double ikSign = 0;  // +1 selects eta+alpha (theta1), -1 selects theta2
    Eigen::Vector2d hipOffset = Eigen::Vector2d::Zero();
};

struct Leg {
    std::string name;
    bool startsWithSwing = false;
    std::array<Branch, 2> branches;  // CAD front/rear names; NOT necessarily theta1/theta2
    Eigen::Vector3d hip = Eigen::Vector3d::Zero();
    double closureX = 0;
    int footGeom = -1;
};

struct Metrics {
    Eigen::Vector3d base = Eigen::Vector3d::Zero();
    double closureError = 0;
    double trackingError = 0;
    double maxTorque = 0;
    double maxFeedforwardTorque = 0;
    double desiredSupportZ = 0;
    double upright = 1;
    int groundContacts = 0;
    bool fallen = false;
};

struct FeedforwardFrame {
    std::array<std::array<double, 2>, 4> tau{};
    std::array<Eigen::Vector3d, 4> supportForces;
    FeedforwardFrame() {
        for (auto& force : supportForces) force.setZero();
    }
};

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

class Simulation {
public:
    Simulation(const std::string& modelPath, Options options = {});
    void reset();
    void step();
    JointTarget inverse(const Branch& branch, const FootTarget& target) const;
    std::array<std::array<JointTarget, 2>, 4> targets(double phase) const;
    FeedforwardFrame feedforward(double phase) const;
    Metrics metrics() const;
    void saveSnapshot(const std::string& path) const;

    mjModel* model() const { return model_.get(); }
    mjData* data() const { return data_.get(); }
    const std::array<Leg, 4>& legs() const { return legs_; }
    const Options& options() const { return options_; }
    const Eigen::Matrix3d& modelToRobot() const { return modelToRobot_; }
    double phase() const;

private:
    void readGeometry();
    void applyPose(double phase);
    void checkState() const;
    Options options_;
    ModelPtr model_{nullptr, mj_deleteModel};
    DataPtr data_{nullptr, mj_deleteData};
    DataPtr gravityData_{nullptr, mj_deleteData};
    FeedforwardFrame lastFeedforward_;
    std::array<Leg, 4> legs_;
    Eigen::Matrix3d modelToRobot_ = Eigen::Matrix3d::Identity();
    int rootQpos_ = -1;
    int baseBody_ = -1;
    double baseHeight_ = 0;
    double lastControlPhase_ = 0;
};

void runViewer(Simulation& simulation, double duration, double speed,
               const std::function<void()>& observe, const std::function<bool()>& keepRunning);

}  // namespace dog::sim
